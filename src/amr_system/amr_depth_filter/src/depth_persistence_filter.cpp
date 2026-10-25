// Republishes the depth cloud for costmap marking without obstacle points that
// do not persist across frames (glossy-floor outliers, 2026-10-25 bags).
// Floor and other non-candidate points pass unchanged.
#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "amr_depth_filter/persistence.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_depth_filter
{
class DepthPersistenceFilter : public rclcpp::Node
{
public:
  DepthPersistenceFilter()
  : Node("depth_persistence_filter"),
    fixed_frame_(declare_parameter("fixed_frame", std::string("odom"))),
    base_frame_(declare_parameter("base_frame", std::string("base_footprint"))),
    min_height_(declare_parameter("min_height", .06)),
    max_height_(declare_parameter("max_height", 1.)),
    max_range_(declare_parameter("max_range", 3.)),
    max_range_squared_(max_range_ * max_range_),
    max_gap_(declare_parameter("max_gap", .5)),
    filter_(declare_parameter("cell_size", .05), declare_parameter("frames", 3),
      declare_parameter("neighbor_cells", 1)),
    buffer_(get_clock()), listener_(buffer_)
  {
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      declare_parameter("output_topic", std::string("/camera/depth/points_filtered")),
      rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter("input_topic", std::string("/camera/depth/points")), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::UniquePtr cloud) {on_cloud(std::move(cloud));});
  }

private:
  static tf2::Transform to_tf(const geometry_msgs::msg::Transform & t)
  {
    return tf2::Transform(
      tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
      tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
  }

  int offset(const sensor_msgs::msg::PointCloud2 & cloud, const char * name) const
  {
    for (const auto & field : cloud.fields) {
      if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        return static_cast<int>(field.offset);
      }
    }
    return -1;
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::UniquePtr cloud)
  {
    if (publisher_->get_subscription_count() == 0) {
      // A later subscriber starts with a full warm-up, even across clock gaps.
      filter_.reset();
      last_stamp_.reset();
      return;
    }
    const int ox = offset(*cloud, "x"), oy = offset(*cloud, "y"), oz = offset(*cloud, "z");
    tf2::Transform to_base, to_fixed;
    try {
      if (ox < 0 || oy < 0 || oz < 0 || cloud->is_bigendian) {
        throw std::runtime_error("cloud needs little-endian FLOAT32 x, y, z");
      }
      const rclcpp::Time stamp(cloud->header.stamp);
      auto cached = to_base_cache_.find(cloud->header.frame_id);
      if (cached == to_base_cache_.end()) {
        const auto transform = to_tf(buffer_.lookupTransform(
            base_frame_, cloud->header.frame_id, stamp,
            rclcpp::Duration::from_seconds(.1)).transform);
        cached = to_base_cache_.emplace(cloud->header.frame_id, transform).first;
      }
      to_base = cached->second;
      to_fixed = to_tf(buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id, stamp,
        rclcpp::Duration::from_seconds(.1)).transform);
    } catch (const std::exception & error) {
      // Fail toward marking: forward the unfiltered cloud and restart persistence.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Forwarding unfiltered depth cloud: %s", error.what());
      filter_.reset();
      publisher_->publish(std::move(cloud));
      return;
    }
    const rclcpp::Time stamp(cloud->header.stamp);
    if (last_stamp_ && ((stamp - *last_stamp_).seconds() > max_gap_ ||
      (stamp - *last_stamp_).seconds() <= 0.))
    {
      filter_.reset();
    }
    last_stamp_ = stamp;

    const size_t count = static_cast<size_t>(cloud->width) * cloud->height;
    const size_t step = cloud->point_step;
    candidates_.clear();
    candidates_.reserve(count);
    xy_.clear();
    xy_.reserve(count);
    keep_.clear();
    keep_.reserve(count);
    keep_.resize(count, true);
    for (size_t i = 0; i < count; ++i) {
      const uint8_t * point = cloud->data.data() + i * step;
      float x, y, z;
      std::memcpy(&x, point + ox, 4);
      std::memcpy(&y, point + oy, 4);
      std::memcpy(&z, point + oz, 4);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
      const tf2::Vector3 p(x, y, z), base = to_base * p;
      if (base.z() < min_height_ || base.z() > max_height_ ||
        beyond_range(base.x(), base.y()))
      {
        continue;
      }
      const tf2::Vector3 fixed = to_fixed * p;
      candidates_.push_back(i);
      xy_.emplace_back(fixed.x(), fixed.y());
    }
    filter_.apply(xy_, persistent_);
    for (size_t k = 0; k < candidates_.size(); ++k) {
      keep_[candidates_[k]] = persistent_[k];
    }

    size_t kept = 0;
    for (size_t i = 0; i < count; ++i) {
      if (keep_[i]) {
        if (kept != i) {
          std::memmove(cloud->data.data() + kept * step,
            cloud->data.data() + i * step, step);
        }
        ++kept;
      }
    }
    cloud->data.resize(kept * step);
    cloud->height = 1;
    cloud->width = static_cast<uint32_t>(kept);
    cloud->row_step = static_cast<uint32_t>(cloud->data.size());
    publisher_->publish(std::move(cloud));
  }

  bool beyond_range(double x, double y) const
  {
    const double squared = x * x + y * y;
    // Keep hypot's overflow/underflow, NaN and negative-parameter behaviour
    // outside the ordinary finite range. At the boundary, rounding can differ.
    if (max_range_ < 0. || !std::isnormal(max_range_squared_) || !std::isnormal(squared)) {
      return std::hypot(x, y) > max_range_;
    }
    return squared > max_range_squared_;
  }

  std::string fixed_frame_, base_frame_;
  double min_height_, max_height_, max_range_, max_range_squared_, max_gap_;
  PersistenceFilter filter_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::optional<rclcpp::Time> last_stamp_;
  std::unordered_map<std::string, tf2::Transform> to_base_cache_;
  std::vector<size_t> candidates_;
  std::vector<std::pair<double, double>> xy_;
  std::vector<uint8_t> keep_, persistent_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
}  // namespace amr_depth_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_depth_filter::DepthPersistenceFilter>());
  rclcpp::shutdown();
  return 0;
}

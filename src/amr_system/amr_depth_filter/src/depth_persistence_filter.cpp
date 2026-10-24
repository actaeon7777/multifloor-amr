// Republishes the depth cloud for costmap marking without obstacle points that
// do not persist across frames (glossy-floor outliers, 2026-10-25 bags).
// Floor and other non-candidate points pass unchanged.
#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
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
    const int ox = offset(*cloud, "x"), oy = offset(*cloud, "y"), oz = offset(*cloud, "z");
    tf2::Transform to_base, to_fixed;
    try {
      if (ox < 0 || oy < 0 || oz < 0 || cloud->is_bigendian) {
        throw std::runtime_error("cloud needs little-endian FLOAT32 x, y, z");
      }
      const rclcpp::Time stamp(cloud->header.stamp);
      to_base = to_tf(buffer_.lookupTransform(base_frame_, cloud->header.frame_id, stamp,
        rclcpp::Duration::from_seconds(.1)).transform);
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
    std::vector<size_t> candidates;
    std::vector<std::pair<double, double>> xy;
    std::vector<bool> keep(count, true);
    for (size_t i = 0; i < count; ++i) {
      const uint8_t * point = cloud->data.data() + i * step;
      float x, y, z;
      std::memcpy(&x, point + ox, 4);
      std::memcpy(&y, point + oy, 4);
      std::memcpy(&z, point + oz, 4);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
      const tf2::Vector3 p(x, y, z), base = to_base * p;
      if (base.z() < min_height_ || base.z() > max_height_ ||
        std::hypot(base.x(), base.y()) > max_range_)
      {
        continue;
      }
      const tf2::Vector3 fixed = to_fixed * p;
      candidates.push_back(i);
      xy.emplace_back(fixed.x(), fixed.y());
    }
    const auto persistent = filter_.apply(xy);
    for (size_t k = 0; k < candidates.size(); ++k) {
      keep[candidates[k]] = persistent[k];
    }

    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out->header = cloud->header;
    out->fields = cloud->fields;
    out->is_bigendian = cloud->is_bigendian;
    out->point_step = cloud->point_step;
    out->is_dense = cloud->is_dense;
    out->height = 1;
    out->data.reserve(cloud->data.size());
    for (size_t i = 0; i < count; ++i) {
      if (keep[i]) {
        const uint8_t * point = cloud->data.data() + i * step;
        out->data.insert(out->data.end(), point, point + step);
      }
    }
    out->width = static_cast<uint32_t>(out->data.size() / step);
    out->row_step = static_cast<uint32_t>(out->data.size());
    publisher_->publish(std::move(out));
  }

  std::string fixed_frame_, base_frame_;
  double min_height_, max_height_, max_range_, max_gap_;
  PersistenceFilter filter_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::optional<rclcpp::Time> last_stamp_;
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

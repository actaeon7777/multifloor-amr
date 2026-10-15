#include "amr_rviz_plugins/single_threaded_tf.hpp"

#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace amr_rviz_plugins
{
void SingleThreadedTF::initialize(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node,
  rclcpp::Clock::SharedPtr clock)
{
  auto wrapper = std::dynamic_pointer_cast<
    rviz_default_plugins::transformation::TFWrapper>(getConnector().lock());
  // Do not call the base initialize(): it creates a dedicated TF thread.
  // RViz's SingleThreadedExecutor now serializes TF receipt and filter requests.
  wrapper->initialize(clock, node, false);
  RCLCPP_INFO(node.lock()->get_raw_node()->get_logger(),
    "AMR TF: using RViz executor to avoid the Humble TF/MessageFilter deadlock");
}

geometry_msgs::msg::PoseStamped SingleThreadedTF::transform(
  const geometry_msgs::msg::PoseStamped & pose, const std::string & target_frame)
{
  // A synchronous wait must not block the executor that receives TF. Use the
  // inherited immediate lookup (including its FrameTransformerException handling).
  // MessageFilter still waits asynchronously for missing transforms.
  const auto stamp = tf2_ros::fromRclcpp(rclcpp::Time(pose.header.stamp));
  const auto tf = lookupTransform(target_frame, pose.header.frame_id, stamp);
  geometry_msgs::msg::PoseStamped result;
  tf2::doTransform(pose, result, tf);
  return result;
}
}  // namespace amr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(
  amr_rviz_plugins::SingleThreadedTF, rviz_common::transformation::FrameTransformer)

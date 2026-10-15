#ifndef AMR_RVIZ_PLUGINS__SINGLE_THREADED_TF_HPP_
#define AMR_RVIZ_PLUGINS__SINGLE_THREADED_TF_HPP_

#include "rviz_default_plugins/transformation/tf_frame_transformer.hpp"

namespace amr_rviz_plugins
{
// Run TF subscriptions, timers and display subscriptions on RViz's executor.
// This avoids geometry2 #992 without replacing the system TF libraries.
class SingleThreadedTF : public rviz_default_plugins::transformation::TFFrameTransformer
{
public:
  void initialize(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node,
    rclcpp::Clock::SharedPtr clock) override;

  geometry_msgs::msg::PoseStamped transform(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & target_frame) override;
};
}  // namespace amr_rviz_plugins
#endif

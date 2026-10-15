#include "amr_nav2_plugins/wheel_speed_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amr_nav2_plugins
{
void WheelSpeedCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("DWB lifecycle node expired");}
  auto read = [&](const std::string & key) {
      const std::string full = dwb_plugin_name_ + "." + name_ + "." + key;
      if (!node->has_parameter(full)) {
        node->declare_parameter(full, rclcpp::ParameterType::PARAMETER_DOUBLE);
      }
      return node->get_parameter(full).as_double();
    };
  // Required launch overrides come from the existing control_mux YAML.
  limit_ = std::make_unique<WheelSpeedLimit>(
    read("wheel_diameter"), read("track_width"), read("max_wheel_rev_s"));
}

double WheelSpeedCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory)
{
  const auto & velocity = trajectory.velocity;
  if (!limit_ || !std::isfinite(velocity.y) || std::abs(velocity.y) > 1e-9 ||
    !limit_->allows(velocity.x, velocity.theta))
  {
    throw dwb_core::IllegalTrajectoryException(name_, "AMR wheel speed limit exceeded");
  }
  return 0.0;
}
}  // namespace amr_nav2_plugins

PLUGINLIB_EXPORT_CLASS(amr_nav2_plugins::WheelSpeedCritic, dwb_core::TrajectoryCritic)

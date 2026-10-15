#include <limits>
#include "gtest/gtest.h"
#include "amr_nav2_plugins/wheel_speed_critic.hpp"
#include "dwb_core/exceptions.hpp"

TEST(WheelSpeed, MatchesIndependentLeftAndRightWheelChecks)
{
  amr_nav2_plugins::WheelSpeedLimit limit(.172, .499, 1.8);
  const double circumference = std::acos(-1.0) * .172;
  for (int i = -100; i <= 100; ++i) {
    for (int j = -80; j <= 80; ++j) {
      double v = i / 100.0, w = j / 100.0;
      double left = (v - .499 * w / 2) / circumference;
      double right = (v + .499 * w / 2) / circumference;
      EXPECT_EQ(limit.allows(v, w), std::max(std::abs(left), std::abs(right)) <= 1.8);
    }
  }
  EXPECT_FALSE(limit.allows(1., 0.));
  EXPECT_TRUE(limit.allows(.97, 0.));
  EXPECT_FALSE(limit.allows(.9, .8));
  EXPECT_TRUE(limit.allows(.7, .8));
  EXPECT_FALSE(limit.allows(std::numeric_limits<double>::quiet_NaN(), 0.));
  EXPECT_THROW(amr_nav2_plugins::WheelSpeedLimit(.172, 0., 1.8), std::invalid_argument);
}

TEST(WheelSpeed, CriticRejectsThroughDwbException)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<nav2_util::LifecycleNode>("wheel_critic_test", "",
    rclcpp::NodeOptions().parameter_overrides({
      {"FollowPath.WheelSpeed.wheel_diameter", .172},
      {"FollowPath.WheelSpeed.track_width", .499},
      {"FollowPath.WheelSpeed.max_wheel_rev_s", 1.8}}));
  amr_nav2_plugins::WheelSpeedCritic critic;
  critic.initialize(node, "WheelSpeed", "FollowPath", nullptr);
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.velocity.x = .5;
  trajectory.velocity.theta = .5;
  EXPECT_EQ(critic.scoreTrajectory(trajectory), 0.);
  trajectory.velocity.x = 1.;
  EXPECT_THROW(critic.scoreTrajectory(trajectory), dwb_core::IllegalTrajectoryException);
  node.reset();
  rclcpp::shutdown();
}

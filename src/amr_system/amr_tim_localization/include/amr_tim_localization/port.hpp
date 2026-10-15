// ROS 2 adapter utilities for the GPL-3.0 TIM upstream core.
#pragma once
#include <chrono>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace tim_port {
using Clock = std::chrono::steady_clock;
inline Clock::time_point deadline = Clock::time_point::max();
inline unsigned random_seed = 7;
inline double nms_distance = 0.75;
inline double nms_angle = 0.30;
inline bool expired() {return Clock::now() >= deadline;}
inline void checkpoint() {if (expired()) throw std::runtime_error("TIM search deadline exceeded; partial candidate set discarded");}
struct WallTime {
  Clock::time_point value;
  static WallTime now() {return {Clock::now()};}
  struct Duration {double seconds; double toSec() const {return seconds;}};
  Duration operator-(const WallTime & other) const {return {std::chrono::duration<double>(value-other.value).count()};}
};
inline geometry_msgs::msg::Quaternion quaternion(double roll, double pitch, double yaw) {
  tf2::Quaternion q; q.setRPY(roll,pitch,yaw); return tf2::toMsg(q);
}
inline geometry_msgs::msg::Quaternion quaternion_yaw(double yaw) {return quaternion(0,0,yaw);}
}

#ifndef AMR_NAV2_PLUGINS__WHEEL_SPEED_CRITIC_HPP_
#define AMR_NAV2_PLUGINS__WHEEL_SPEED_CRITIC_HPP_
#include <cmath>
#include <memory>
#include <stdexcept>
#include "dwb_core/trajectory_critic.hpp"

namespace amr_nav2_plugins
{
class WheelSpeedLimit
{
public:
  WheelSpeedLimit(double diameter, double track, double rev_s)
  {
    for (double value : {diameter, track, rev_s}) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Wheel geometry and rev/s must be positive and finite");
      }
    }
    maximum_ = std::acos(-1.0) * diameter * rev_s;
    half_track_ = track / 2.0;
  }
  bool allows(double v, double w) const
  {
    return std::isfinite(v) && std::isfinite(w) &&
           std::abs(v) + std::abs(w) * half_track_ <= maximum_ + 1e-9;
  }
private:
  double maximum_, half_track_;
};

class WheelSpeedCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override;
private:
  std::unique_ptr<WheelSpeedLimit> limit_;
};
}  // namespace amr_nav2_plugins
#endif

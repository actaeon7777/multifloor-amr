// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <amr_tim_localization/localizer.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <relocalization/utils/relocalization_2d/gicp_2d.h>
namespace amr_tim_localization {
struct FineDiagnostic {
 bool accepted=false;
 std::string reason;
 double coarse_score=0, fine_score=0, translation=0, yaw=0;
 gicp_2d::MatchDiagnostics icp;
};
struct FineResult {
 std::vector<geometry_msgs::msg::TransformStamped> poses;
 std::vector<float> scores;
 std::vector<FineDiagnostic> diagnostics;
};
std::string fine_rejection(const FineDiagnostic &d, const Config &c, bool deadline_expired);
FineResult refine_candidates(const nav_msgs::msg::OccupancyGrid &map,
 const sensor_msgs::msg::LaserScan &raw_scan,
 const std::vector<geometry_msgs::msg::TransformStamped> &coarse, const Config &config,
 const std::vector<float> &coarse_scores);
}

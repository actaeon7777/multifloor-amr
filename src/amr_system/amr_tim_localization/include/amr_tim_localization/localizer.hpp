// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <amr_localization_interfaces/srv/global_localize.hpp>
namespace amr_tim_localization {
struct Config {
 int max_beams = 1080;
 double max_range = 30.0;
 double max_timeout_sec = 30.0;
 int max_map_cells = 4000000;
 int threads = 2;
 int random_seed = 7;
 int map_sampling_ratio = 2;
 int sampling_duration_sec = 1;
 double anchor_spacing = 0.40;
 double min_laser_clearance = 0.20;
 double angle_search_step = 0.19634954084936207;
 int batch_size = 200;
 int candidates_per_batch = 20;
 double min_score = 0.85;
 double max_score = 0.98;
 double sigma_hit = 0.2;
 double max_tolerance_dist = 3.0;
 double max_correspondence_distance = 1.6;
 int icp_iterations = 60;
 double nms_distance = 0.75;
 double nms_angle = 0.30;
 bool fine_enabled = true;
 int fine_max_candidates = 5;
 int fine_max_beams = 1080;
 int fine_map_sampling_ratio = 1;
 int fine_icp_iterations = 60;
 double fine_correspondence_distance = 0.30;
 double fine_max_translation = 0.25;
 double fine_max_yaw = 0.10;
 double fine_min_overlap = 0.70;
 double fine_min_geometry_ratio = 0.002;
 double fine_budget_sec = 3.0;
 double fine_min_score_gain = 1e-6;
};
using Service = amr_localization_interfaces::srv::GlobalLocalize;
void localize(const Service::Request & request, Service::Response & response, const Config & config);
}

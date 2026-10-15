// SPDX-License-Identifier: GPL-3.0-only
// ROS 2 service adapter; algorithm source attribution is in UPSTREAM.md.
#include <amr_tim_localization/localizer.hpp>
#include <amr_tim_localization/fine_refinement.hpp>
#include <relocalization/utils/relocalization_2d/relocalization_2d.h>
#include <mutex>
#include <cmath>
#include <limits>
namespace amr_tim_localization {
namespace {
std::mutex core_mutex;
void require(bool condition, const char *message) {if (!condition) throw std::invalid_argument(message);}
bool finite(double value) {return std::isfinite(value);}
void validate_config(const Config &c) {
 require(c.max_beams>=12 && c.max_beams<=1440, "max_beams must be in [12,1440]");
 require(finite(c.max_range) && c.max_range>0 && c.max_range<=100, "invalid max_range");
 require(finite(c.max_timeout_sec) && c.max_timeout_sec>0 && c.max_timeout_sec<=120, "invalid max_timeout_sec");
 require(c.max_map_cells>0 && c.max_map_cells<=16000000 && c.threads>0 && c.threads<=16, "invalid resource limits");
 require(c.map_sampling_ratio>0 && c.map_sampling_ratio<=8 && c.sampling_duration_sec>=0 && c.sampling_duration_sec<=10, "invalid preprocessing parameters");
 require(finite(c.anchor_spacing) && c.anchor_spacing>=0.1 && c.anchor_spacing<=3.0 && finite(c.min_laser_clearance) && c.min_laser_clearance>=0, "invalid sampling distances");
 require(finite(c.angle_search_step) && c.angle_search_step>=0.02 && c.angle_search_step<=M_PI && c.batch_size>0 && c.batch_size<=1000 && c.candidates_per_batch>0 && c.candidates_per_batch<=c.batch_size, "invalid candidate search parameters");
 require(finite(c.min_score) && finite(c.max_score) && c.min_score>=0 && c.min_score<=c.max_score && c.max_score<=1, "invalid score thresholds");
 require(finite(c.sigma_hit) && c.sigma_hit>0 && finite(c.max_tolerance_dist) && c.max_tolerance_dist>0, "invalid TAM parameters");
 require(finite(c.max_correspondence_distance) && c.max_correspondence_distance>0 && c.icp_iterations>0 && c.icp_iterations<=200, "invalid ICP parameters");
 require(finite(c.nms_distance) && c.nms_distance>0 && finite(c.nms_angle) && c.nms_angle>0 && c.nms_angle<=M_PI, "invalid NMS parameters");
 require(finite(c.fine_min_score_gain)&&c.fine_min_score_gain>0&&c.fine_min_score_gain<=.1,"invalid fine minimum score gain");
 require(c.fine_max_candidates>=1&&c.fine_max_candidates<=5&&c.fine_max_beams>=12&&c.fine_max_beams<=1440&&c.fine_map_sampling_ratio>=1&&c.fine_map_sampling_ratio<=4,"invalid fine sampling parameters");
 require(c.fine_icp_iterations>=1&&c.fine_icp_iterations<=200&&finite(c.fine_correspondence_distance)&&c.fine_correspondence_distance>0&&finite(c.fine_max_translation)&&c.fine_max_translation>0&&finite(c.fine_max_yaw)&&c.fine_max_yaw>0&&c.fine_max_yaw<=M_PI,"invalid fine alignment parameters");
 require(finite(c.fine_min_overlap)&&c.fine_min_overlap>=0&&c.fine_min_overlap<=1&&finite(c.fine_min_geometry_ratio)&&c.fine_min_geometry_ratio>=0&&c.fine_min_geometry_ratio<=1&&finite(c.fine_budget_sec)&&c.fine_budget_sec>0&&c.fine_budget_sec<=30,"invalid fine quality/deadline parameters");
}
}
void localize(const Service::Request &request, Service::Response &response, const Config &c) {
 const auto started=tim_port::Clock::now();
 response=Service::Response{};
 std::unique_lock<std::mutex> lock(core_mutex, std::try_to_lock);
 try {
  require(lock.owns_lock(), "TIM worker is busy");
  validate_config(c);
  require(finite(request.timeout_sec) && request.timeout_sec>0, "timeout_sec must be finite and positive");
  require(request.max_candidates>=1 && request.max_candidates<=20, "max_candidates must be in [1,20]");
  tim_port::deadline=started+std::chrono::duration_cast<tim_port::Clock::duration>(std::chrono::duration<double>(std::min(request.timeout_sec,c.max_timeout_sec)));
  tim_port::random_seed=static_cast<unsigned>(c.random_seed);
  tim_port::nms_distance=c.nms_distance; tim_port::nms_angle=c.nms_angle;
  omp_set_dynamic(0); omp_set_num_threads(c.threads); omp_set_max_active_levels(1);
  const auto &m=request.map; const auto &scan=request.scan;
  const uint64_t cells=static_cast<uint64_t>(m.info.width)*m.info.height;
  require(m.info.width>1 && m.info.height>1 && m.info.width<=32768 && m.info.height<=32768 && cells<=static_cast<uint64_t>(c.max_map_cells) && cells==m.data.size(), "invalid map dimensions/data or map exceeds cell limit");
  require(finite(m.info.resolution) && m.info.resolution>=0.005 && m.info.resolution<=2.0, "invalid map resolution");
  require(c.max_tolerance_dist>m.info.resolution*0.5, "max_tolerance_dist must exceed half a map cell");
  require(!m.header.frame_id.empty() && !scan.header.frame_id.empty(), "map and scan frame_id are required");
  const auto &origin=m.info.origin;
  require(finite(origin.position.x) && finite(origin.position.y) && finite(origin.position.z), "non-finite map origin");
  const auto &q=origin.orientation;
  require(finite(q.x)&&finite(q.y)&&finite(q.z)&&finite(q.w), "non-finite map orientation");
  const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
  require(std::abs(norm-1.0)<0.01 && std::abs(q.x)<1e-6 && std::abs(q.y)<1e-6, "map orientation must be a normalized planar quaternion");
  bool free_found=false, obstacle_found=false;
  for (size_t i=0;i<m.data.size();++i) {
   if ((i&4095)==0) tim_port::checkpoint();
   const auto value=m.data[i]; require(value>=-1 && value<=100,"invalid map occupancy value");
   free_found |= value>=0 && value<=30; obstacle_found |= value>=55;
  }
  require(free_found && obstacle_found,"map needs known free and occupied cells");
  require(scan.ranges.size()>=12 && scan.ranges.size()<=100000 && finite(scan.angle_min) && finite(scan.angle_max) && finite(scan.angle_increment) && std::abs(scan.angle_increment)>1e-7, "invalid scan angular metadata or sample count");
  require(std::abs(scan.angle_max-(scan.angle_min+(scan.ranges.size()-1)*static_cast<double>(scan.angle_increment)))<=std::max(0.001,2.0*std::abs(static_cast<double>(scan.angle_increment))), "scan angles do not match range count");
  require(std::abs(scan.angle_increment)*(scan.ranges.size()-1)<=2*M_PI+0.05,"scan field of view exceeds 360 degrees");
  require(finite(scan.range_min)&&finite(scan.range_max)&&scan.range_min>=0&&scan.range_max>scan.range_min,"invalid scan range limits");
  sensor_msgs::msg::LaserScan reduced=scan;
  const size_t stride=(scan.ranges.size()+c.max_beams-1)/c.max_beams;
  reduced.ranges.clear(); reduced.intensities.clear();
  reduced.angle_increment=scan.angle_increment*stride;
  reduced.time_increment=scan.time_increment*stride;
  reduced.range_max=std::min<double>(scan.range_max,c.max_range);
  require(reduced.range_max>scan.range_min,"configured range excludes all scan returns");
  size_t valid=0;
  for (size_t i=0;i<scan.ranges.size();i+=stride) {
   const float range=scan.ranges[i];
   if (finite(range)&&range>=scan.range_min&&range<reduced.range_max) {reduced.ranges.push_back(range);++valid;}
   else reduced.ranges.push_back(std::numeric_limits<float>::quiet_NaN());
  }
  reduced.angle_max=reduced.angle_min+(reduced.ranges.size()-1)*reduced.angle_increment;
  require(valid>=12,"fewer than 12 valid downsampled LiDAR returns inside configured range");
  // All upstream grid operations are axis aligned. Normalize once, then compose
  // the original origin SE(2) onto every result. This also handles nonzero yaw.
  nav_msgs::msg::OccupancyGrid local_map=m;
  local_map.info.origin=geometry_msgs::msg::Pose{}; local_map.info.origin.orientation.w=1.0;
  const double origin_yaw=tf2::getYaw(q), cs=std::cos(origin_yaw), sn=std::sin(origin_yaw);
  relocalization_2d::relocalization_2d core(
   false,true,0.2f,2.0f,c.sampling_duration_sec,
   c.min_score,c.max_score,c.angle_search_step,c.batch_size,c.candidates_per_batch,c.min_laser_clearance,
   c.sigma_hit,1.0f,0.0f,c.max_tolerance_dist,0.8f,0.9f,0.7f,0.85f,true,30,55,
   reduced.range_max,c.anchor_spacing,c.max_correspondence_distance,1e-6f,1e-5f,c.icp_iterations);
  const auto candidates=core.get_candidates(reduced,false,1,c.map_sampling_ratio,local_map,request.max_candidates);
  tim_port::checkpoint();
  if (!std::get<0>(candidates)) throw std::runtime_error("TIM found no candidate above the configured TAM threshold");
  auto scores=std::get<1>(candidates); auto transforms=std::get<2>(candidates);
  if(c.fine_enabled) {
   auto refined=refine_candidates(local_map,scan,transforms,c,scores);
   transforms=std::move(refined.poses);scores=std::move(refined.scores);
  }
  // Optional refinement fallback cannot bypass the total request deadline.
  tim_port::checkpoint();
  for (size_t i=0;i<transforms.size();++i) {
   const auto &t=transforms[i]; const double x=t.transform.translation.x,y=t.transform.translation.y;
   const double yaw=tf2::getYaw(t.transform.rotation);
   if (!finite(x)||!finite(y)||!finite(yaw)||!finite(scores[i])) continue;
   geometry_msgs::msg::PoseStamped pose;
   pose.header=m.header; pose.header.stamp=scan.header.stamp;
   pose.pose.position.x=origin.position.x+cs*x-sn*y;
   pose.pose.position.y=origin.position.y+sn*x+cs*y;
   pose.pose.orientation=tim_port::quaternion_yaw(origin_yaw+yaw);
   response.poses.push_back(pose); response.scores.push_back(std::clamp<double>(scores[i],0.0,1.0));
  }
  response.success=!response.poses.empty();
  response.message=response.success ? "TIM complete: laser-frame candidates; independent validation required" : "TIM produced no finite candidate";
 } catch (const std::exception &e) {response.success=false; response.message=e.what(); response.poses.clear();response.scores.clear();}
 response.elapsed_sec=std::chrono::duration<double>(tim_port::Clock::now()-started).count();
}
}

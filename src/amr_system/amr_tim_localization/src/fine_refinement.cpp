// SPDX-License-Identifier: GPL-3.0-only
#include <amr_tim_localization/fine_refinement.hpp>
#include <relocalization/utils/relocalization_2d/likelihood_field_2d.h>
#include <algorithm>
#include <numeric>
namespace amr_tim_localization {
std::string fine_rejection(const FineDiagnostic &d,const Config &c,bool timeout) {
 if(timeout) return "fine_deadline";
 if(!d.icp.converged || d.icp.termination==0 || d.icp.termination==1 || d.icp.termination>=5) return "icp_not_converged";
 if(!std::isfinite(d.icp.rms) || !std::isfinite(d.translation) || !std::isfinite(d.yaw) || !std::isfinite(d.fine_score) || !std::isfinite(d.coarse_score) || !std::isfinite(d.icp.geometry_ratio) || !std::isfinite(d.icp.overlap)) return "nonfinite";
 if(d.translation>c.fine_max_translation || d.yaw>c.fine_max_yaw) return "excessive_correction";
 if(d.icp.overlap<c.fine_min_overlap) return "low_overlap";
 if(d.icp.geometry_ratio<c.fine_min_geometry_ratio || d.icp.normal_constraints<12) return "degenerate_geometry";
 if(d.fine_score<d.coarse_score+c.fine_min_score_gain) return "insufficient_dense_gain";
 return {};
}
FineResult refine_candidates(const nav_msgs::msg::OccupancyGrid &map,
 const sensor_msgs::msg::LaserScan &raw,const std::vector<geometry_msgs::msg::TransformStamped> &coarse,const Config &c,const std::vector<float> &coarse_scores) {
 tim_port::checkpoint();
 if(coarse_scores.size()!=coarse.size())throw std::invalid_argument("fine candidate score count mismatch");
 FineResult result;result.poses=coarse;result.scores=coarse_scores;result.diagnostics.resize(coarse.size());
 if(coarse.empty())return result;
 sensor_msgs::msg::LaserScan scan=raw;scan.ranges.clear();scan.intensities.clear();
 const size_t stride=(raw.ranges.size()+c.fine_max_beams-1)/c.fine_max_beams;
 scan.angle_increment=raw.angle_increment*stride;scan.time_increment=raw.time_increment*stride;
 scan.range_max=std::min<double>(raw.range_max,c.max_range);
 size_t valid=0;
 for(size_t i=0;i<raw.ranges.size();i+=stride) {
  const float r=raw.ranges[i];scan.ranges.push_back(std::isfinite(r)&&r>=raw.range_min&&r<scan.range_max?r:std::numeric_limits<float>::quiet_NaN());
  if(std::isfinite(scan.ranges.back()))++valid;
 }
 if(valid<12) {
  for(auto &d:result.diagnostics)d.reason="insufficient_dense_returns";
  RCLCPP_WARN(rclcpp::get_logger("tim_fine"),"stage=fallback_all reason=insufficient_dense_returns valid=%zu; original coarse poses and scores retained",valid);
  return result;
 }
 scan.angle_max=scan.angle_min+(scan.ranges.size()-1)*scan.angle_increment;
 auto distance=calc_min_dist_to_obs_table(map,55);tim_port::checkpoint();
 likelihood_field_2d::likelihood_field_2d metric(c.sigma_hit,1.f,0.f,.8f,.9f,.7f,.85f,55);
 // Every retained coarse candidate is measured on the SAME dense scan as a
 // refined candidate, including candidates not selected for refinement.
 std::vector<float> dense_scores(coarse.size());
 for(size_t i=0;i<coarse.size();++i) {
  tim_port::checkpoint();
  dense_scores[i]=metric.get_confidence(coarse[i],scan,map,distance,c.max_tolerance_dist,1,true);
  if(!std::isfinite(dense_scores[i])) {
   for(auto &d:result.diagnostics)d.reason="nonfinite_dense_metric";
   RCLCPP_WARN(rclcpp::get_logger("tim_fine"),"stage=fallback_all reason=nonfinite_dense_metric; original coarse poses and scores retained");
   return result;
  }
  result.diagnostics[i].coarse_score=dense_scores[i];result.diagnostics[i].fine_score=dense_scores[i];
  result.diagnostics[i].reason="not_selected";
 }
 result.scores=dense_scores;
 const auto request_deadline=tim_port::deadline;
 struct RestoreDeadline {tim_port::Clock::time_point value;~RestoreDeadline(){tim_port::deadline=value;}} restore{request_deadline};
 tim_port::deadline=std::min(request_deadline,tim_port::Clock::now()+std::chrono::duration_cast<tim_port::Clock::duration>(std::chrono::duration<double>(c.fine_budget_sec)));
 const size_t count=std::min<size_t>(coarse.size(),c.fine_max_candidates);
 try {
 gicp_2d::gicp_2d fine(c.fine_correspondence_distance,1e-8f,1e-7f,c.fine_icp_iterations,55,false);
 fine.set_map(map,c.fine_map_sampling_ratio);fine.set_scan(scan,1);
 for(size_t i=0;i<count;++i) {
  auto &d=result.diagnostics[i];
  if(tim_port::expired()){d.reason="fine_deadline";continue;}
  d.icp=fine.match_detailed(coarse[i]);
  d.translation=std::hypot(d.icp.pose.transform.translation.x-coarse[i].transform.translation.x,d.icp.pose.transform.translation.y-coarse[i].transform.translation.y);
  d.yaw=std::abs(angles::shortest_angular_distance(tf2::getYaw(coarse[i].transform.rotation),tf2::getYaw(d.icp.pose.transform.rotation)));
  if(!tim_port::expired())d.fine_score=metric.get_confidence(d.icp.pose,scan,map,distance,c.max_tolerance_dist,1,true);
  d.reason=fine_rejection(d,c,tim_port::expired());
  if(d.reason.empty()) {
   const auto point=create_point(d.icp.pose.transform.translation.x,d.icp.pose.transform.translation.y);
   if(!is_point_in_map(point,map)||is_point_unknow(point,map,-1,-1)||calc_min_dist_to_obs(map,distance,point)<c.min_laser_clearance)
    d.reason="infeasible_laser_position";
  }
  d.accepted=d.reason.empty();
  if(d.accepted){result.poses[i]=d.icp.pose;result.scores[i]=d.fine_score;d.reason="accepted";}
 }
 } catch(const std::exception &error) {
  // Optional refinement failures preserve every coarse candidate. A total
  // request timeout is still rejected by the checkpoint below.
  result.poses=coarse;result.scores=dense_scores;
  for(auto &d:result.diagnostics){d.accepted=false;d.reason="fine_exception";}
  RCLCPP_WARN(rclcpp::get_logger("tim_fine"),"stage=fallback_all reason=fine_exception error=%s",error.what());
 }
 for(size_t i=0;i<count;++i) {
  const auto &d=result.diagnostics[i];
  RCLCPP_INFO(rclcpp::get_logger("tim_fine"),
   "candidate=%zu status=%s coarse_dense=%.6f fine_dense=%.6f shift=%.4fm yaw=%.4frad termination=%d rms=%.5fm overlap=%.3f geometry=%.6f points=%zu correspondences=%zu normals=%zu",
   i,d.reason.c_str(),d.coarse_score,d.fine_score,d.translation,d.yaw,d.icp.termination,d.icp.rms,d.icp.overlap,d.icp.geometry_ratio,d.icp.source_points,d.icp.correspondences,d.icp.normal_constraints);
 }
 tim_port::deadline=request_deadline;tim_port::checkpoint();
 // Re-sort and suppress duplicates only after all scores use the dense metric.
 std::vector<size_t> order(coarse.size());std::iota(order.begin(),order.end(),0);
 std::stable_sort(order.begin(),order.end(),[&](size_t a,size_t b){return result.scores[a]>result.scores[b];});
 FineResult sorted;sorted.diagnostics=result.diagnostics;
 for(size_t i:order) {
  bool duplicate=false;
  for(const auto &p:sorted.poses) {
   if(std::hypot(p.transform.translation.x-result.poses[i].transform.translation.x,p.transform.translation.y-result.poses[i].transform.translation.y)<=c.nms_distance &&
      std::abs(angles::shortest_angular_distance(tf2::getYaw(p.transform.rotation),tf2::getYaw(result.poses[i].transform.rotation)))<=c.nms_angle){duplicate=true;break;}
  }
  if(!duplicate){sorted.poses.push_back(result.poses[i]);sorted.scores.push_back(result.scores[i]);}
 }
 return sorted;
}
}

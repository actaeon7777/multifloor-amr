// SPDX-License-Identifier: GPL-3.0-only
#include <amr_tim_localization/localizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
namespace amr_tim_localization {
class TimNode : public rclcpp::Node {
public:
 TimNode():Node("tim_global_localizer") {
  rcl_interfaces::msg::ParameterDescriptor startup_only;
  startup_only.read_only=true;
  startup_only.description="Loaded at startup; edit the YAML configuration and restart this node to change.";
#define PARAM(name) config_.name=declare_parameter(#name,config_.name,startup_only)
  PARAM(max_beams); PARAM(max_range); PARAM(max_timeout_sec); PARAM(max_map_cells);
  PARAM(threads); PARAM(random_seed); PARAM(map_sampling_ratio); PARAM(sampling_duration_sec);
  PARAM(anchor_spacing); PARAM(min_laser_clearance); PARAM(angle_search_step);
  PARAM(batch_size); PARAM(candidates_per_batch); PARAM(min_score); PARAM(max_score);
  PARAM(sigma_hit); PARAM(max_tolerance_dist); PARAM(max_correspondence_distance); PARAM(icp_iterations);
  PARAM(nms_distance); PARAM(nms_angle);
  PARAM(fine_enabled); PARAM(fine_max_candidates); PARAM(fine_max_beams);
  PARAM(fine_map_sampling_ratio); PARAM(fine_icp_iterations); PARAM(fine_correspondence_distance);
  PARAM(fine_max_translation); PARAM(fine_max_yaw); PARAM(fine_min_overlap);
  PARAM(fine_min_geometry_ratio); PARAM(fine_budget_sec); PARAM(fine_min_score_gain);
#undef PARAM
  const auto service_name=declare_parameter<std::string>("service_name","/localization/global_localize",startup_only);
  service_=create_service<Service>(service_name,[this](const Service::Request::SharedPtr request, Service::Response::SharedPtr response) {
   localize(*request,*response,config_);
   RCLCPP_INFO(get_logger(),"%s (%.3fs, %zu candidates)",response->message.c_str(),response->elapsed_sec,response->poses.size());
  });
  RCLCPP_INFO(get_logger(),"TIM global localization ready on %s; no motion, TF or initialpose publishers",service_name.c_str());
 }
private:
 Config config_;
 rclcpp::Service<Service>::SharedPtr service_;
};
}
int main(int argc,char **argv) {
 rclcpp::init(argc,argv);
 rclcpp::spin(std::make_shared<amr_tim_localization::TimNode>());
 rclcpp::shutdown();return 0;
}

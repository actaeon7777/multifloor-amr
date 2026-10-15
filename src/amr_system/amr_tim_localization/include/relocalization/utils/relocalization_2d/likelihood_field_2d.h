/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#ifndef LIKELIHOOD_FIELD_2D_H
#define LIKELIHOOD_FIELD_2D_H

#include <algorithm>
#include <random>
#include <map>
#include <unordered_map>
#include <relocalization/utils/math_utils.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

namespace likelihood_field_2d
{
    class likelihood_field_2d
    {
    public:
        likelihood_field_2d(const float &sigma_hit = 0.2f, const float &z_hit = 1.0f, const float &z_rand = 0.0f,
                            const float &keep_ratio_1_min = 0.8f, const float &keep_ratio_1_max = 0.9f, const float &keep_ratio_2_min = 0.7f, const float &keep_ratio_2_max = 0.85f,
                            const int &map_occupied_threshold = 50);
        ~likelihood_field_2d() {};

        tuple<float, float, float> get_likelihood_field_score(const geometry_msgs::msg::TransformStamped &tf_map_lidar,
                                                              const sensor_msgs::msg::LaserScan &scan,
                                                              const nav_msgs::msg::OccupancyGrid &map,
                                                              const vector<float> &obs_dist_table,
                                                              const int &lidar_sampling_step);
        tuple<float, float, float> get_likelihood_field_score(const geometry_msgs::msg::TransformStamped &tf_map_lidar,
                                                              const sensor_msgs::msg::LaserScan &scan,
                                                              const nav_msgs::msg::OccupancyGrid &map,
                                                              const int &lidar_sampling_step);
        float get_confidence(const geometry_msgs::msg::TransformStamped &tf_map_lidar,
                             const sensor_msgs::msg::LaserScan &scan,
                             const nav_msgs::msg::OccupancyGrid &map,
                             const vector<float> &obs_dist_table,
                             const float &max_tolerance_dist,
                             const int &lidar_sampling_step,
                             const bool &geometric_mean_fusion = true);
    private:
        float sigma_hit_ = 0.2f;
        float z_hit_ = 1.0f;
        float z_rand_ = 0.0f;
        int map_occupied_threshold_ = 50;
        float keep_ratio_1_min_ = 0.8f;
        float keep_ratio_1_max_ = 0.9f;
        float keep_ratio_2_min_ = 0.70f;
        float keep_ratio_2_max_ = 0.85f;

        float gaussian_prob(float dist);

    };
}

#endif
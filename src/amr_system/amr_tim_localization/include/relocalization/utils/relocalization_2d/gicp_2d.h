/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#ifndef GICP_2D_H
#define GICP_2D_H

#include <random>
#include <map>
#include <unordered_map>
#include <relocalization/utils/math_utils.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

namespace gicp_2d
{
    struct MatchDiagnostics {
        geometry_msgs::msg::TransformStamped pose;
        bool converged = false;
        int termination = 0;
        double rms = std::numeric_limits<double>::infinity();
        double overlap = 0.0;
        double geometry_ratio = 0.0;
        size_t source_points = 0;
        size_t correspondences = 0;
        size_t normal_constraints = 0;
    };
    class gicp_2d
    {
    public:
        gicp_2d(const float &max_correspondence_distance = 1.0f,
                const float &transformation_epsilon = 1e-6,
                const float &euclidean_fitness_epsilon = 1e-5,
                const int &max_iterations = 50,
                const int &map_occupied_threshold = 50,
                const bool &enable_visualization = false);
        ~gicp_2d() {};

        void set_scan(const sensor_msgs::msg::LaserScan &scan,
                      const int &lidar_sampling_step = 1);
        void set_map(const nav_msgs::msg::OccupancyGrid &map,
                     const int &map_sampling_ratio = 1);

        geometry_msgs::msg::TransformStamped match(const geometry_msgs::msg::TransformStamped &predict);
        MatchDiagnostics match_detailed(const geometry_msgs::msg::TransformStamped &predict);

    private:
        geometry_msgs::msg::TransformStamped match_impl(
            const geometry_msgs::msg::TransformStamped &predict, MatchDiagnostics *diagnostics);
        bool enable_visualization_;


        float max_correspondence_distance_;
        float transformation_epsilon_;
        float euclidean_fitness_epsilon_;
        int max_iterations_;
        float map_occupied_threshold_;
        float map_sampling_ratio_ = 1;
        float map_resolution_ = 0.05f;
        float scan_max_range_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr map_kdtree_;

        void build_local_map(const float &center_x,
                             const float &center_y,
                             const float &radius,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &local_map);
    };
}

#endif
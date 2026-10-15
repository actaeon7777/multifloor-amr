/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#ifndef SECTOR_QUERY_2D_H
#define SECTOR_QUERY_2D_H

#include <random>
#include <map>
#include <unordered_map>
#include <relocalization/utils/math_utils.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// if to use fenwick tree
#define USE_FENWICK_TREE 1

using namespace std;

namespace sector_query_2d
{
    class sector_query_2d
    {
    public:
        sector_query_2d(const int &sector_sample_num = 1200);
        ~sector_query_2d() {};

        void add_range(const float &angle, const float &range);

        float get_sum(const float &angle_start, const float &angle);
        float get_mean(const float &angle_start, const float &angle);

    private:
        int sector_sample_num_;
        vector<float> tree_;
        vector<float> data_;

        int angle_to_index(const float &angle);

        void update(int index, float value);
        float query(int index);
    };
}

#endif
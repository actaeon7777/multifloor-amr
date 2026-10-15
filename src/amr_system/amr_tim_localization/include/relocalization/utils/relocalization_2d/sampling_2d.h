/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#ifndef SAMPLING_2D_H
#define SAMPLING_2D_H

#include <random>
#include <map>
#include <unordered_map>
#include <relocalization/utils/math_utils.h>
#include <relocalization/utils/relocalization_2d/rrt_common.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// if to use R-Tree in RRT sampling
#define USE_R_TREE 1

#if USE_R_TREE
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#endif

using namespace std;

namespace sampling_2d
{
#if USE_R_TREE
    namespace bg = boost::geometry;
    namespace bgi = boost::geometry::index;
    typedef bg::model::point<float, 2, bg::cs::cartesian> BGPoint;
    typedef pair<BGPoint, size_t> RTreeValue;
#endif

    class sampling_2d
    {
    public:
        sampling_2d(const float &min_expand_dist, const float &max_expand_dist, const float &min_dist_to_obs,
               const float &points_min_dist, const bool &use_traversability_sampling = true,
               const int &free_threshold = 30, const int &occupied_threshold = 55);
        ~sampling_2d() {};

        void initialize(const nav_msgs::msg::OccupancyGrid &map_data, const vector<float> &min_dist_to_obs_table);
        vector<geometry_msgs::msg::Point32> get_points(const int &timeout_secs);

    private:
        float min_expand_dist_, max_expand_dist_;
        float min_dist_to_obs_;
        float points_min_dist_;
        bool use_traversability_sampling_;
        float start_x_, start_y_;
        float current_map_x_, current_map_y_;

        geometry_msgs::msg::Point32 origin_;

        int free_threshold_, occupied_threshold_;

        nav_msgs::msg::OccupancyGrid map_data_;
        vector<float> min_dist_to_obs_table_;

        unique_ptr<random_device> random_device_;
        unique_ptr<mt19937> random_generator_;
        unique_ptr<uniform_real_distribution<float>> gen_random_number_;

        int node_index_ = 0;
        rrt_nodes rrt_tree_;

#if USE_R_TREE
        bgi::rtree<RTreeValue, bgi::quadratic<16>> rtree_;
        vector<BGPoint> rtree_points_;
#endif

        float generate_random_number();
    };
}

#endif
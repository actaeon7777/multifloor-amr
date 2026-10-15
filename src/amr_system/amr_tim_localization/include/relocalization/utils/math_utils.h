/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#ifndef MATH_UTILS
#define MATH_UTILS
#include <amr_tim_localization/port.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <angles/angles.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <octomap/octomap.h>
#include <tuple>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <omp.h>

using namespace std;

geometry_msgs::msg::Polygon transform_footprint(geometry_msgs::msg::PoseWithCovarianceStamped &pose,
                                           geometry_msgs::msg::Polygon &footprint);

int orientation(const geometry_msgs::msg::Point32 &p, const geometry_msgs::msg::Point32 &q, const geometry_msgs::msg::Point32 &r);

bool on_segment(const geometry_msgs::msg::Point32 &p, const geometry_msgs::msg::Point32 &q, const geometry_msgs::msg::Point32 &r);

bool is_intersect(const geometry_msgs::msg::Point32 &p1,
                  const geometry_msgs::msg::Point32 &p2,
                  const geometry_msgs::msg::Point32 &p3,
                  const geometry_msgs::msg::Point32 &p4);

bool is_point_in_rect(const geometry_msgs::msg::Point32 &p, const geometry_msgs::msg::Polygon &rect);

float calc_path_point_orientation(geometry_msgs::msg::Pose &pose, vector<geometry_msgs::msg::Pose> &forward_points);

bool is_footprint_has_occupied_point(nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid,
                                     geometry_msgs::msg::Polygon &footprint,
                                     const geometry_msgs::msg::PoseStamped &pose,
                                     float stop_distance,
                                     bool reversed = false,
                                     int sampling_ratio = 1,
                                     float padding_ratio = 1.2);

geometry_msgs::msg::Point32 calculate_polygon_center(const geometry_msgs::msg::Polygon &polygon);

float calculate_distance(const geometry_msgs::msg::Point32 &p1, const geometry_msgs::msg::Point32 &p2);

float calculate_distance(const geometry_msgs::msg::Pose &p1, const geometry_msgs::msg::Pose &p2);

float calculate_distance(const octomap::point3d &p1, const octomap::point3d &p2);

float calculate_distance(const cv::Point2f &p1, const cv::Point2f &p2);

float calculate_distance(const pcl::PointXY &p1, const pcl::PointXY &p2);

float calculate_distance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

geometry_msgs::msg::Point32 create_point(const float &x, const float &y);

geometry_msgs::msg::Point32 create_point(const float &x, const float &y, const float &z);

geometry_msgs::msg::Point create_pointd(const float &x, const float &y);

geometry_msgs::msg::Point create_pointd(const float &x, const float &y, const float &z);

geometry_msgs::msg::Point point_ros_to_rosd(const geometry_msgs::msg::Point32 &p);

octomap::point3d point_ros_to_octomap(const geometry_msgs::msg::Point32 &p);

geometry_msgs::msg::Point32 point_octomap_to_ros(const octomap::point3d &p);

cv::Point2f point_ros_to_cv(const geometry_msgs::msg::Point32 &p);

geometry_msgs::msg::Point32 point_cv_to_ros(const cv::Point2f &p);

geometry_msgs::msg::Point point_cv_to_rosd(const cv::Point2f &p);

geometry_msgs::msg::Point32 point_pcl_xyz_to_ros(const pcl::PointXYZ &p);

pcl::PointXYZRGB point_ros_to_pcl_xyzrgb(const geometry_msgs::msg::Point32 &p);

geometry_msgs::msg::Point32 point_pcl_xyzrgb_to_ros(const pcl::PointXYZRGB &p);

geometry_msgs::msg::Point point_pcl_xyz_to_rosd(const pcl::PointXYZ &p);

geometry_msgs::msg::Point point_pcl_xyzrgb_to_rosd(const pcl::PointXYZRGB &p);

geometry_msgs::msg::Polygon points_to_polygon(const vector<geometry_msgs::msg::Point32> &points);

geometry_msgs::msg::Polygon points_to_polygon(const vector<octomap::point3d> &points);

geometry_msgs::msg::Polygon points_to_polygon(const vector<cv::Point2f> &points);

geometry_msgs::msg::Polygon points_to_polygon(const geometry_msgs::msg::Point32 &p0,
                                         const geometry_msgs::msg::Point32 &p1,
                                         const geometry_msgs::msg::Point32 &p2,
                                         const geometry_msgs::msg::Point32 &p3);

geometry_msgs::msg::Polygon points_to_polygon(const octomap::point3d &p0,
                                         const octomap::point3d &p1,
                                         const octomap::point3d &p2,
                                         const octomap::point3d &p3);

geometry_msgs::msg::Polygon points_to_polygon(const cv::Point2f &p0,
                                         const cv::Point2f &p1,
                                         const cv::Point2f &p2,
                                         const cv::Point2f &p3);

tuple<geometry_msgs::msg::Point32, geometry_msgs::msg::Point32> get_box_aabb(const geometry_msgs::msg::Polygon &polygon);

tuple<int, int, int, int> get_box_aabb_in_grid(const geometry_msgs::msg::Polygon &polygon,
                                               const nav_msgs::msg::OccupancyGrid &grid);

geometry_msgs::msg::Point32 grid_xy_to_position(const int &x, const int &y, const nav_msgs::msg::OccupancyGrid &grid);

geometry_msgs::msg::Point32 grid_index_to_position(const int &index, const nav_msgs::msg::OccupancyGrid &grid);

pcl::PointXYZ grid_xy_to_pointcloud_xyz(const int &x, const int &y, const nav_msgs::msg::OccupancyGrid &grid);

pcl::PointXYZ grid_index_to_pointcloud_xyz(const int &index, const nav_msgs::msg::OccupancyGrid &grid);

pcl::PointXYZRGB grid_xy_to_pointcloud_xyzrgb(const int &x, const int &y, const nav_msgs::msg::OccupancyGrid &grid);

pcl::PointXYZRGB grid_index_to_pointcloud_xyzrgb(const int &index, const nav_msgs::msg::OccupancyGrid &grid);

geometry_msgs::msg::Point32 grid_index_to_position(const int &index, const nav_msgs::msg::OccupancyGrid &grid);

int grid_xy_to_grid_index(const int &x, const int &y, const nav_msgs::msg::OccupancyGrid &grid);

tuple<int, int> grid_index_to_grid_xy(const int &index, const nav_msgs::msg::OccupancyGrid &grid);

tuple<int, int> position_to_grid_xy(const geometry_msgs::msg::Point32 &p, const nav_msgs::msg::OccupancyGrid &grid);

tuple<int, int> position_to_grid_xy(const octomap::point3d &p, const nav_msgs::msg::OccupancyGrid &grid);

tuple<int, int> position_to_grid_xy(const cv::Point2f &p, const nav_msgs::msg::OccupancyGrid &grid);

tuple<int, int> position_to_grid_xy(const pcl::PointXYZ &p, const nav_msgs::msg::OccupancyGrid &grid);

tuple<int, int> position_to_grid_xy(const pcl::PointXYZRGB &p, const nav_msgs::msg::OccupancyGrid &grid);

int position_to_grid_index(const geometry_msgs::msg::Point32 &p, const nav_msgs::msg::OccupancyGrid &grid);

int position_to_grid_index(const octomap::point3d &p, const nav_msgs::msg::OccupancyGrid &grid);

int position_to_grid_index(const cv::Point2f &p, const nav_msgs::msg::OccupancyGrid &grid);

bool is_point_in_circle(const geometry_msgs::msg::Point32 &center, const float &radius, const geometry_msgs::msg::Point32 &p);

tuple<geometry_msgs::msg::Point32, float> circle_from_two_points(const geometry_msgs::msg::Point32 &p1, const geometry_msgs::msg::Point32 &p2);

tuple<geometry_msgs::msg::Point32, float> circle_from_three_points(const geometry_msgs::msg::Point32 &p1, const geometry_msgs::msg::Point32 &p2, const geometry_msgs::msg::Point32 &p3);

tuple<geometry_msgs::msg::Point32, float> welzl(vector<geometry_msgs::msg::Point32> points, vector<geometry_msgs::msg::Point32> points_recurs, size_t n);

tuple<geometry_msgs::msg::Point32, float> find_min_bounding_circle(const vector<geometry_msgs::msg::Point32> &points);

tuple<geometry_msgs::msg::Point32, float> find_min_bounding_circle(const pcl::PointCloud<pcl::PointXYZ> &points);

tuple<geometry_msgs::msg::Point32, float> find_min_bounding_circle(const pcl::PointCloud<pcl::PointXYZRGB> &points);

vector<geometry_msgs::msg::PoseStamped> calc_path_orientation(const vector<geometry_msgs::msg::PoseStamped> &path);

vector<int> enumerate_line_indices(const float &x0, const float &y0, const float &x1, const float &y1,
                                   const int &width, const int &height, const float &resolu);

vector<int> enumerate_line_indices(const geometry_msgs::msg::Point32 &p0, const geometry_msgs::msg::Point32 &p1,
                                   const nav_msgs::msg::OccupancyGrid &grid);

tuple<bool, float, geometry_msgs::msg::Point32> raycast_to_first_obstacle(const geometry_msgs::msg::Point32 &p0,
                                                                     const float &angle_rad,
                                                                     const float &max_range,
                                                                     const nav_msgs::msg::OccupancyGrid &grid,
                                                                     const int &map_occupied_threshold);

float calc_laser_aabb_area(const sensor_msgs::msg::LaserScan &scan, const int &lidar_sampling_step);

float calc_laser_obb_area(const sensor_msgs::msg::LaserScan &scan, const int &lidar_sampling_step);

float calc_laser_mean_dist(const sensor_msgs::msg::LaserScan &scan, const float &sensing_radius, const int &lidar_sampling_step);

bool is_segment_no_collision(const float &x0, const float &y0, const float &x1, const float &y1, const int &step,
                             const int &width, const int &height, const float &resolu,
                             const float &min_obs_dist,
                             const vector<float> &obs_dist_table);

enum exploration_segment_type
{
    NO_COLLISION_WITH_UNKNOW = -1,
    NO_COLLISION = 0,
    WITH_COLLISION = 1
};

exploration_segment_type exploration_check_segment_type(const geometry_msgs::msg::Point32 &p0, const geometry_msgs::msg::Point32 &p1, const int &step,
                                                        const float &min_obs_dist,
                                                        const nav_msgs::msg::OccupancyGrid &grid,
                                                        const vector<float> &obs_dist_table,
                                                        const bool &check_obs,
                                                        const int &free_threshold = 30, const int &occupied_threshold = 60);

nav_msgs::msg::OccupancyGrid clear_footprint_on_grid(const nav_msgs::msg::OccupancyGrid &grid,
                                                const geometry_msgs::msg::Point32 &origin, const float &radius);

vector<float> calc_min_dist_to_obs_table(const nav_msgs::msg::OccupancyGrid &grid, const int &threshold);

vector<float> calc_min_dist_to_obs_table(const nav_msgs::msg::OccupancyGrid &grid,
                                         const float center_x, const float center_y, const float radius,
                                         const int &threshold);

float calc_min_dist_to_obs(const nav_msgs::msg::OccupancyGrid &grid, const vector<float> &min_dist_to_obs_table, const geometry_msgs::msg::Point32 &position);

float calc_min_dist_to_obs(const nav_msgs::msg::OccupancyGrid &grid, const vector<float> &min_dist_to_obs_table, const geometry_msgs::msg::PoseStamped &pose);

float calc_min_dist_to_obs(const nav_msgs::msg::OccupancyGrid &grid, const vector<float> &min_dist_to_obs_table, const geometry_msgs::msg::TransformStamped &tf);

vector<int> resample_points(const vector<geometry_msgs::msg::Point32> &points, const vector<int> &indices,
                            const float &min_dist);

vector<geometry_msgs::msg::Point32> resample_points(const vector<geometry_msgs::msg::Point32> &points, const float &min_dist);

geometry_msgs::msg::Point32 generate_point(const geometry_msgs::msg::Point32 &p0, const geometry_msgs::msg::Point32 &dir, const float &dist);

geometry_msgs::msg::Point32 generate_point_on_segment(const geometry_msgs::msg::Point32 &p0, const geometry_msgs::msg::Point32 &dir, const float &dist);

float calc_unknow_area(const geometry_msgs::msg::Point32 &p, const float &radius, const nav_msgs::msg::OccupancyGrid &grid,
                       const int &free_threshold = 30, const int &occupied_threshold = 60);

tuple<int, geometry_msgs::msg::Point32> get_nearest_point(const geometry_msgs::msg::Point32 &p,
                                                     const vector<geometry_msgs::msg::Point32> &points, const vector<int> &indices);

tuple<int, geometry_msgs::msg::Point32> get_nearest_point(const geometry_msgs::msg::Point32 &p,
                                                     const vector<geometry_msgs::msg::Point32> &points, const vector<int> &indices, const vector<bool> &del_states);

bool is_point_in_map(const geometry_msgs::msg::Point32 &p, const nav_msgs::msg::OccupancyGrid &grid);

bool is_point_unknow(const geometry_msgs::msg::Point32 &p, const nav_msgs::msg::OccupancyGrid &grid,
                     const int &free_threshold = 30, const int &occupied_threshold = 60);

float calc_decayed_distance(const geometry_msgs::msg::Point32 &p, const nav_msgs::msg::OccupancyGrid &grid, const vector<float> &obs_dist_table,
                            const float &min_dist_to_obs, const float &min_expand_dist, const float &max_expand_dist);

float calc_orientation_diff(const geometry_msgs::msg::Point32 &p0, const geometry_msgs::msg::Point32 &p1, const geometry_msgs::msg::Point32 &p2);

tuple<float, float> get_grid_max_min(const nav_msgs::msg::OccupancyGrid &grid);

vector<geometry_msgs::msg::Point32> get_round_points(const geometry_msgs::msg::Point32 &p,
                                                const float &radius, const int &num,
                                                const nav_msgs::msg::OccupancyGrid &grid,
                                                const float &min_obs_dist, const vector<float> &obs_dist_table);

vector<geometry_msgs::msg::PoseStamped> get_round_points(const geometry_msgs::msg::PoseStamped &p,
                                                    const float &radius, const int &num,
                                                    const nav_msgs::msg::OccupancyGrid &grid,
                                                    const float &min_obs_dist, const vector<float> &obs_dist_table);

float calc_seg_direction(const geometry_msgs::msg::PoseStamped &p0, const geometry_msgs::msg::Point32 &p1);

float calc_map_area(const nav_msgs::msg::OccupancyGrid &grid,
                    const int &free_threshold = 30, const int &occupied_threshold = 60);

#endif
/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <relocalization/utils/math_utils.h>

geometry_msgs::msg::Polygon transform_footprint(geometry_msgs::msg::PoseWithCovarianceStamped &pose,
                                           geometry_msgs::msg::Polygon &footprint)
{
    geometry_msgs::msg::Polygon ret;

    ret.points.clear();

    size_t size = footprint.points.size();

    for (int i = 0; i < size; i++)
    {
        geometry_msgs::msg::Point32 point;

        float x = footprint.points[i].x;
        float y = footprint.points[i].y;
        float yaw = tf2::getYaw(pose.pose.pose.orientation);

        point.x = pose.pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));

        ret.points.push_back(point);
    }

    return ret;
}

int orientation(const geometry_msgs::msg::Point32 &p, const geometry_msgs::msg::Point32 &q, const geometry_msgs::msg::Point32 &r)
{
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

    if (fabs(val) < 1e-10)
    {
        return 0;
    }

    return (val > 0) ? 1 : 2;
}

bool on_segment(const geometry_msgs::msg::Point32 &p, const geometry_msgs::msg::Point32 &q, const geometry_msgs::msg::Point32 &r)
{
    return (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
            q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y));
}

bool is_intersect(const geometry_msgs::msg::Point32 &p1,
                  const geometry_msgs::msg::Point32 &p2,
                  const geometry_msgs::msg::Point32 &p3,
                  const geometry_msgs::msg::Point32 &p4)
{
    int o1 = orientation(p3, p4, p1);
    int o2 = orientation(p3, p4, p2);
    int o3 = orientation(p1, p2, p3);
    int o4 = orientation(p1, p2, p4);

    if (o1 != o2 && o3 != o4)
    {
        return true;
    }

    if (o1 == 0 && on_segment(p3, p1, p4))
    {
        return true;
    }
    if (o2 == 0 && on_segment(p3, p2, p4))
    {
        return true;
    }
    if (o3 == 0 && on_segment(p1, p3, p2))
    {
        return true;
    }
    if (o4 == 0 && on_segment(p1, p4, p2))
    {
        return true;
    }

    return false;
}

bool is_point_in_rect(const geometry_msgs::msg::Point32 &p, const geometry_msgs::msg::Polygon &rect)
{
    int count = 0;

    size_t size = rect.points.size();

    for (unsigned int i = 0; i < size; i++)
    {
        geometry_msgs::msg::Point32 a = rect.points[i];
        geometry_msgs::msg::Point32 b = rect.points[(i + 1) % size];

        if ((a.y <= p.y && b.y > p.y) || (b.y <= p.y && a.y > p.y))
        {
            double t = (p.y - a.y) / (double)(b.y - a.y);

            if (a.x + t * (b.x - a.x) > p.x)
            {
                count++;
            }
        }
    }

    return count % 2 != 0;
}

float calc_path_point_orientation(geometry_msgs::msg::Pose &pose, vector<geometry_msgs::msg::Pose> &forward_points)
{
    float avg_dx = 0.0;
    float avg_dy = 0.0;
    int num_points = 0;

    for (const auto &point : forward_points)
    {
        float dx = point.position.x - pose.position.x;
        float dy = point.position.y - pose.position.y;

        float distance = sqrt(dx * dx + dy * dy);

        if (distance <= 0.05)
        {
            avg_dx += dx;
            avg_dy += dy;
            num_points++;
        }
        else
        {
            break;
        }
    }

    if (num_points > 0)
    {
        avg_dx /= (float)num_points;
        avg_dy /= (float)num_points;
    }

    float avg_orientation = atan2(avg_dy, avg_dx);

    return avg_orientation;
}

bool is_footprint_has_occupied_point(nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid,
                                     geometry_msgs::msg::Polygon &footprint,
                                     const geometry_msgs::msg::PoseStamped &pose,
                                     float stop_distance,
                                     bool reversed,
                                     int sampling_ratio,
                                     float padding_ratio)
{
    geometry_msgs::msg::Point32 min_corner, max_corner;
    min_corner.x = pose.pose.position.x - stop_distance * padding_ratio;
    min_corner.y = pose.pose.position.y - stop_distance * padding_ratio;
    max_corner.x = pose.pose.position.x + stop_distance * padding_ratio;
    max_corner.y = pose.pose.position.y + stop_distance * padding_ratio;

    int min_x_index = max(0, (int)((min_corner.x - grid->info.origin.position.x) / grid->info.resolution));
    int min_y_index = max(0, (int)((min_corner.y - grid->info.origin.position.y) / grid->info.resolution));
    int max_x_index = min((int)grid->info.width - 1, (int)((max_corner.x - grid->info.origin.position.x) / grid->info.resolution));
    int max_y_index = min((int)grid->info.height - 1, (int)((max_corner.y - grid->info.origin.position.y) / grid->info.resolution));

    geometry_msgs::msg::Polygon obstacle_detection_area;

    if (!reversed)
    {
        float yaw = tf2::getYaw(pose.pose.orientation);

        geometry_msgs::msg::Point32 point;
        float x = footprint.points[0].x;
        float y = footprint.points[0].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);

        x = footprint.points[1].x;
        y = footprint.points[1].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);

        x = footprint.points[1].x + stop_distance * padding_ratio;
        y = footprint.points[1].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);

        x = footprint.points[0].x + stop_distance * padding_ratio;
        y = footprint.points[0].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);
    }
    else
    {
        float yaw = angles::normalize_angle(
            tf2::getYaw(pose.pose.orientation) + M_PI);

        geometry_msgs::msg::Point32 point;
        float x = footprint.points[2].x;
        float y = footprint.points[2].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);

        x = footprint.points[3].x;
        y = footprint.points[3].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);

        x = footprint.points[3].x - stop_distance * padding_ratio;
        y = footprint.points[3].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);

        x = footprint.points[2].x - stop_distance * padding_ratio;
        y = footprint.points[2].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);
    }

    for (int i = min_x_index; i <= max_x_index; i += sampling_ratio)
    {
        for (int j = min_y_index; j <= max_y_index; j += sampling_ratio)
        {
            geometry_msgs::msg::Point32 point;
            point.x = grid->info.origin.position.x + i * grid->info.resolution;
            point.y = grid->info.origin.position.y + j * grid->info.resolution;

            if (is_point_in_rect(point, obstacle_detection_area))
            {
                int index = j * grid->info.width + i;

                if (grid->data[index] >= 66)
                {
                    return true;
                }
            }
        }
    }

    return false;
}

geometry_msgs::msg::Point32 calculate_polygon_center(const geometry_msgs::msg::Polygon &polygon)
{
    geometry_msgs::msg::Point32 center;

    for (const auto &point : polygon.points)
    {
        center.x += point.x;
        center.y += point.y;
    }

    center.x /= polygon.points.size();
    center.y /= polygon.points.size();

    return center;
}

float calculate_distance(const geometry_msgs::msg::Pose &p1, const geometry_msgs::msg::Pose &p2)
{
    return hypot(hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y), p1.position.z - p2.position.z);
}

float calculate_distance(const geometry_msgs::msg::Point32 &p1, const geometry_msgs::msg::Point32 &p2)
{
    return hypot(hypot(p1.x - p2.x, p1.y - p2.y), p1.z - p2.z);
}

float calculate_distance(const octomap::point3d &p1, const octomap::point3d &p2)
{
    return hypot(hypot(p1.x() - p2.x(), p1.y() - p2.y()), p1.z() - p2.z());
}

float calculate_distance(const cv::Point2f &p1, const cv::Point2f &p2)
{
    return hypot(p1.x - p2.x, p1.y - p2.y);
}

float calculate_distance(const pcl::PointXY &p1, const pcl::PointXY &p2)
{
    return hypot(p1.x - p2.x, p1.y - p2.y);
}

float calculate_distance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
{
    return hypot(hypot(p1.x - p2.x, p1.y - p2.y), p1.z - p2.z);
}

geometry_msgs::msg::Point32 create_point(const float &x, const float &y)
{
    geometry_msgs::msg::Point32 ret;

    ret.x = x;
    ret.y = y;

    return ret;
}

geometry_msgs::msg::Point32 create_point(const float &x, const float &y, const float &z)
{
    geometry_msgs::msg::Point32 ret = create_point(x, y);

    ret.z = z;

    return ret;
}

geometry_msgs::msg::Point create_pointd(const float &x, const float &y)
{
    geometry_msgs::msg::Point ret;

    ret.x = x;
    ret.y = y;

    return ret;
}

geometry_msgs::msg::Point create_pointd(const float &x, const float &y, const float &z)
{
    geometry_msgs::msg::Point ret = create_pointd(x, y);

    ret.z = z;

    return ret;
}

geometry_msgs::msg::Point point_ros_to_rosd(const geometry_msgs::msg::Point32 &p)
{
    geometry_msgs::msg::Point ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = p.z;

    return ret;
}

octomap::point3d point_ros_to_octomap(const geometry_msgs::msg::Point32 &p)
{
    return octomap::point3d(p.x, p.y, p.z);
}

geometry_msgs::msg::Point32 point_octomap_to_ros(const octomap::point3d &p)
{
    geometry_msgs::msg::Point32 ret;

    ret.x = p.x();
    ret.y = p.y();
    ret.z = p.z();

    return ret;
}

cv::Point2f point_ros_to_cv(const geometry_msgs::msg::Point32 &p)
{
    return cv::Point2f(p.x, p.y);
}

geometry_msgs::msg::Point32 point_cv_to_ros(const cv::Point2f &p)
{
    geometry_msgs::msg::Point32 ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = 0.0;

    return ret;
}

geometry_msgs::msg::Point point_cv_to_rosd(const cv::Point2f &p)
{
    geometry_msgs::msg::Point ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = 0.0;

    return ret;
}

geometry_msgs::msg::Point32 point_pcl_xyz_to_ros(const pcl::PointXYZ &p)
{
    geometry_msgs::msg::Point32 ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = p.z;

    return ret;
}

geometry_msgs::msg::Point32 point_pcl_xyzrgb_to_ros(const pcl::PointXYZRGB &p)
{
    geometry_msgs::msg::Point32 ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = p.z;

    return ret;
}

pcl::PointXYZRGB point_ros_to_pcl_xyzrgb(const geometry_msgs::msg::Point32 &p)
{
    pcl::PointXYZRGB ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = p.z;

    return ret;
}

geometry_msgs::msg::Point point_pcl_xyz_to_rosd(const pcl::PointXYZ &p)
{
    geometry_msgs::msg::Point ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = p.z;

    return ret;
}

geometry_msgs::msg::Point point_pcl_xyzrgb_to_rosd(const pcl::PointXYZRGB &p)
{
    geometry_msgs::msg::Point ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = p.z;

    return ret;
}

geometry_msgs::msg::Polygon points_to_polygon(const vector<geometry_msgs::msg::Point32> &points)
{
    geometry_msgs::msg::Polygon polygon;

    for (const auto &p : points)
    {
        polygon.points.push_back(p);
    }

    return polygon;
}

geometry_msgs::msg::Polygon points_to_polygon(const vector<octomap::point3d> &points)
{
    geometry_msgs::msg::Polygon polygon;

    for (const auto &p : points)
    {
        polygon.points.push_back(point_octomap_to_ros(p));
    }

    return polygon;
}

geometry_msgs::msg::Polygon points_to_polygon(const vector<cv::Point2f> &points)
{
    geometry_msgs::msg::Polygon polygon;

    for (const auto &p : points)
    {
        polygon.points.push_back(point_cv_to_ros(p));
    }

    return polygon;
}

geometry_msgs::msg::Polygon points_to_polygon(const geometry_msgs::msg::Point32 &p0,
                                         const geometry_msgs::msg::Point32 &p1,
                                         const geometry_msgs::msg::Point32 &p2,
                                         const geometry_msgs::msg::Point32 &p3)
{
    return points_to_polygon(vector<geometry_msgs::msg::Point32>{p0, p1, p2, p3});
}

geometry_msgs::msg::Polygon points_to_polygon(const octomap::point3d &p0,
                                         const octomap::point3d &p1,
                                         const octomap::point3d &p2,
                                         const octomap::point3d &p3)
{
    return points_to_polygon(vector<octomap::point3d>{p0, p1, p2, p3});
}

geometry_msgs::msg::Polygon points_to_polygon(const cv::Point2f &p0,
                                         const cv::Point2f &p1,
                                         const cv::Point2f &p2,
                                         const cv::Point2f &p3)
{
    return points_to_polygon(vector<cv::Point2f>{p0, p1, p2, p3});
}

tuple<geometry_msgs::msg::Point32, geometry_msgs::msg::Point32> get_box_aabb(const geometry_msgs::msg::Polygon &polygon)
{
    float min_x = numeric_limits<float>::max();
    float min_y = numeric_limits<float>::max();
    float max_x = -numeric_limits<float>::max();
    float max_y = -numeric_limits<float>::max();

    for (const auto &point : polygon.points)
    {
        min_x = min(min_x, point.x);
        min_y = min(min_y, point.y);
        max_x = max(max_x, point.x);
        max_y = max(max_y, point.y);
    }

    geometry_msgs::msg::Point32 min_point, max_point;

    min_point.x = min_x;
    min_point.y = min_y;
    min_point.z = 0;

    max_point.x = max_x;
    max_point.y = max_y;
    max_point.z = 0;

    return {min_point, max_point};
}

tuple<int, int, int, int> get_box_aabb_in_grid(const geometry_msgs::msg::Polygon &polygon, const nav_msgs::msg::OccupancyGrid &grid)
{
    auto [min_point, max_point] = get_box_aabb(polygon);

    int min_x = max(0, static_cast<int>((min_point.x - grid.info.origin.position.x) / grid.info.resolution));
    int min_y = max(0, static_cast<int>((min_point.y - grid.info.origin.position.y) / grid.info.resolution));
    int max_x = min(static_cast<int>(grid.info.width), static_cast<int>((max_point.x - grid.info.origin.position.x) / grid.info.resolution));
    int max_y = min(static_cast<int>(grid.info.height), static_cast<int>((max_point.y - grid.info.origin.position.y) / grid.info.resolution));

    return {min_x, max_x, min_y, max_y};
}

geometry_msgs::msg::Point32 grid_xy_to_position(const int &x, const int &y, const nav_msgs::msg::OccupancyGrid &grid)
{
    geometry_msgs::msg::Point32 position;

    position.x = grid.info.origin.position.x + (x + 0.5) * grid.info.resolution;
    position.y = grid.info.origin.position.y + (y + 0.5) * grid.info.resolution;
    position.z = 0;

    return position;
}

geometry_msgs::msg::Point32 grid_index_to_position(const int &index, const nav_msgs::msg::OccupancyGrid &grid)
{
    int x = index % grid.info.width;
    int y = index / grid.info.width;

    return grid_xy_to_position(x, y, grid);
}

pcl::PointXYZ grid_xy_to_pointcloud_xyz(const int &x, const int &y, const nav_msgs::msg::OccupancyGrid &grid)
{
    float real_x = grid.info.origin.position.x + (x + 0.5) * grid.info.resolution;
    float real_y = grid.info.origin.position.y + (y + 0.5) * grid.info.resolution;

    return pcl::PointXYZ(real_x, real_y, 0.0f);
}

pcl::PointXYZ grid_index_to_pointcloud_xyz(const int &index, const nav_msgs::msg::OccupancyGrid &grid)
{
    int x = index % grid.info.width;
    int y = index / grid.info.width;

    return grid_xy_to_pointcloud_xyz(x, y, grid);
}

pcl::PointXYZRGB grid_xy_to_pointcloud_xyzrgb(const int &x, const int &y, const nav_msgs::msg::OccupancyGrid &grid)
{
    pcl::PointXYZRGB ret;

    float real_x = grid.info.origin.position.x + (x + 0.5) * grid.info.resolution;
    float real_y = grid.info.origin.position.y + (y + 0.5) * grid.info.resolution;

    ret.x = real_x;
    ret.y = real_y;

    return ret;
}

pcl::PointXYZRGB grid_index_to_pointcloud_xyzrgb(const int &index, const nav_msgs::msg::OccupancyGrid &grid)
{
    int x = index % grid.info.width;
    int y = index / grid.info.width;

    return grid_xy_to_pointcloud_xyzrgb(x, y, grid);
}

int grid_xy_to_grid_index(const int &x, const int &y, const nav_msgs::msg::OccupancyGrid &grid)
{
    return y * grid.info.width + x;
}

tuple<int, int> grid_index_to_grid_xy(const int &index, const nav_msgs::msg::OccupancyGrid &grid)
{
    int x = index % grid.info.width;
    int y = index / grid.info.width;

    return {x, y};
}

tuple<int, int> position_to_grid_xy(const geometry_msgs::msg::Point32 &p, const nav_msgs::msg::OccupancyGrid &grid)
{
    int x = static_cast<int>(floor((p.x - grid.info.origin.position.x) / grid.info.resolution));
    int y = static_cast<int>(floor((p.y - grid.info.origin.position.y) / grid.info.resolution));

    return {x, y};
}

tuple<int, int> position_to_grid_xy(const octomap::point3d &p, const nav_msgs::msg::OccupancyGrid &grid)
{
    return position_to_grid_xy(point_octomap_to_ros(p), grid);
}

tuple<int, int> position_to_grid_xy(const cv::Point2f &p, const nav_msgs::msg::OccupancyGrid &grid)
{
    return position_to_grid_xy(point_cv_to_ros(p), grid);
}

tuple<int, int> position_to_grid_xy(const pcl::PointXYZ &p, const nav_msgs::msg::OccupancyGrid &grid)
{
    return position_to_grid_xy(point_pcl_xyz_to_ros(p), grid);
}

tuple<int, int> position_to_grid_xy(const pcl::PointXYZRGB &p, const nav_msgs::msg::OccupancyGrid &grid)
{
    return position_to_grid_xy(point_pcl_xyzrgb_to_ros(p), grid);
}

int position_to_grid_index(const geometry_msgs::msg::Point32 &p, const nav_msgs::msg::OccupancyGrid &grid)
{
    auto [x, y] = position_to_grid_xy(p, grid);

    auto index = y * grid.info.width + x;

    return max(0, static_cast<int>(min(static_cast<int>(y * grid.info.width + x), static_cast<int>(grid.data.size() - 1))));
}

int position_to_grid_index(const octomap::point3d &p, const nav_msgs::msg::OccupancyGrid &grid)
{
    return position_to_grid_index(point_octomap_to_ros(p), grid);
}

int position_to_grid_index(const cv::Point2f &p, const nav_msgs::msg::OccupancyGrid &grid)
{
    return position_to_grid_index(point_cv_to_ros(p), grid);
}

bool is_point_in_circle(const geometry_msgs::msg::Point32 &center, const float &radius, const geometry_msgs::msg::Point32 &p)
{
    return calculate_distance(p, center) <= radius;
}

tuple<geometry_msgs::msg::Point32, float> circle_from_two_points(const geometry_msgs::msg::Point32 &p1, const geometry_msgs::msg::Point32 &p2)
{
    geometry_msgs::msg::Point32 center;

    center.x = (p1.x + p2.x) / 2;
    center.y = (p1.y + p2.y) / 2;

    float radius = calculate_distance(p1, center);

    return {center, radius};
}

tuple<geometry_msgs::msg::Point32, float> circle_from_three_points(const geometry_msgs::msg::Point32 &p1, const geometry_msgs::msg::Point32 &p2, const geometry_msgs::msg::Point32 &p3)
{
    float d = 2 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));

    geometry_msgs::msg::Point32 center;

    center.x = ((p1.x * p1.x + p1.y * p1.y) * (p2.y - p3.y) + (p2.x * p2.x + p2.y * p2.y) * (p3.y - p1.y) + (p3.x * p3.x + p3.y * p3.y) * (p1.y - p2.y)) / d;
    center.y = ((p1.x * p1.x + p1.y * p1.y) * (p3.x - p2.x) + (p2.x * p2.x + p2.y * p2.y) * (p1.x - p3.x) + (p3.x * p3.x + p3.y * p3.y) * (p2.x - p1.x)) / d;

    float radius = calculate_distance(p1, center);

    return {center, radius};
}

tuple<geometry_msgs::msg::Point32, float> welzl(vector<geometry_msgs::msg::Point32> points, vector<geometry_msgs::msg::Point32> points_recurs, size_t n)
{
    if (n == 0 || points_recurs.size() == 3)
    {
        switch (points_recurs.size())
        {
        case 0:
            return {geometry_msgs::msg::Point32(), 0};
        case 1:
            return {points_recurs[0], 0};
        case 2:
            return circle_from_two_points(points_recurs[0], points_recurs[1]);
        case 3:
            return circle_from_three_points(points_recurs[0], points_recurs[1], points_recurs[2]);
        }
    }

    size_t idx = rand() % n;
    geometry_msgs::msg::Point32 p = points[idx];
    swap(points[idx], points[n - 1]);

    auto [circle_center, circle_radius] = welzl(points, points_recurs, n - 1);

    if (is_point_in_circle(circle_center, circle_radius, p))
    {
        return {circle_center, circle_radius};
    }

    points_recurs.push_back(p);

    return welzl(points, points_recurs, n - 1);
}

tuple<geometry_msgs::msg::Point32, float> find_min_bounding_circle(const vector<geometry_msgs::msg::Point32> &points)
{
    vector<geometry_msgs::msg::Point32> points_modifiable = points;
    vector<geometry_msgs::msg::Point32> points_recurs;

    return welzl(points_modifiable, points_recurs, points.size());
}

tuple<geometry_msgs::msg::Point32, float> find_min_bounding_circle(const pcl::PointCloud<pcl::PointXYZ> &points)
{
    vector<geometry_msgs::msg::Point32> ros_points;

    for (const auto &point : points)
    {
        ros_points.push_back(point_pcl_xyz_to_ros(point));
    }

    return find_min_bounding_circle(ros_points);
}

tuple<geometry_msgs::msg::Point32, float> find_min_bounding_circle(const pcl::PointCloud<pcl::PointXYZRGB> &points)
{
    vector<geometry_msgs::msg::Point32> ros_points;

    for (const auto &point : points)
    {
        ros_points.push_back(point_pcl_xyzrgb_to_ros(point));
    }

    return find_min_bounding_circle(ros_points);
}

vector<geometry_msgs::msg::PoseStamped> calc_path_orientation(const vector<geometry_msgs::msg::PoseStamped> &path)
{
    vector<geometry_msgs::msg::PoseStamped> ret;

    size_t size = path.size();

    ret.resize(size);

#pragma omp parallel for
    for (int i = 0; i < size; i++)
    {
        ret[i].header = path[i].header;

        ret[i].pose = path[i].pose;

        if (i < size - 2)
        {
            vector<geometry_msgs::msg::Pose> forward_points;

            forward_points.clear();

            for (int j = 1; j <= min(5, (int)(size - i - 2)); j += 1)
            {
                forward_points.push_back(path[i + j].pose);
            }

            ret[i].pose.orientation =
                tim_port::quaternion_yaw(
                    calc_path_point_orientation(ret[i].pose, forward_points));
        }
        else if (i != size - 1)
        {
            ret[i].pose.orientation = ret[i - 1].pose.orientation;
        }
    }

    return ret;
}

vector<int> enumerate_line_indices(const float &x0, const float &y0, const float &x1, const float &y1,
                                   const int &width, const int &height, const float &resolu)
{
    int i0 = floor(x0 / resolu);
    int j0 = floor(y0 / resolu);
    int i1 = floor(x1 / resolu);
    int j1 = floor(y1 / resolu);

    vector<int> indices;

    int dx = abs(i1 - i0), dy = abs(j1 - j0);
    int sx = (i0 < i1) ? 1 : -1;
    int sy = (j0 < j1) ? 1 : -1;
    int err = dx - dy;

    int x = i0, y = j0;

    while (true)
    {
        if (x >= 0 && x < width && y >= 0 && y < height)
        {
            int index = y * width + x;

            if (index >= 0)
            {
                indices.push_back(index);
            }
        }

        if (x == i1 && y == j1)
        {
            break;
        }

        int e2 = 2 * err;

        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }

        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }

    return indices;
}

vector<int> enumerate_line_indices(
    const geometry_msgs::msg::Point32 &p0,
    const geometry_msgs::msg::Point32 &p1,
    const nav_msgs::msg::OccupancyGrid &grid)
{
    const float resolution = grid.info.resolution;
    const float origin_x = grid.info.origin.position.x;
    const float origin_y = grid.info.origin.position.y;
    const int width = grid.info.width;
    const int height = grid.info.height;

    int x0 = static_cast<int>(floor((p0.x - origin_x) / resolution));
    int y0 = static_cast<int>(floor((p0.y - origin_y) / resolution));
    int x1 = static_cast<int>(floor((p1.x - origin_x) / resolution));
    int y1 = static_cast<int>(floor((p1.y - origin_y) / resolution));

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    vector<int> indices;
    indices.reserve(max(dx, dy) + 1);

    int x = x0;
    int y = y0;

    while (true)
    {
        if (x >= 0 && x < width && y >= 0 && y < height)
        {
            int index = y * width + x;

            indices.push_back(index);
        }

        if (x == x1 && y == y1)
            break;

        int e2 = 2 * err;

        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }

        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }

    return indices;
}

tuple<bool, float, geometry_msgs::msg::Point32> raycast_to_first_obstacle(const geometry_msgs::msg::Point32 &p0,
                                                                     const float &angle_rad,
                                                                     const float &max_range,
                                                                     const nav_msgs::msg::OccupancyGrid &grid,
                                                                     const int &map_occupied_threshold)
{
    tuple<bool, float, geometry_msgs::msg::Point32> ret;

    const float resolution = grid.info.resolution;
    const float origin_x = grid.info.origin.position.x;
    const float origin_y = grid.info.origin.position.y;
    const int width = grid.info.width;
    const int height = grid.info.height;
    const vector<int8_t> &data = grid.data;

    int x0 = static_cast<int>(floor((p0.x - origin_x) / resolution));
    int y0 = static_cast<int>(floor((p0.y - origin_y) / resolution));

    float x1f = p0.x + max_range * cos(angle_rad);
    float y1f = p0.y + max_range * sin(angle_rad);
    int x1 = static_cast<int>(floor((x1f - origin_x) / resolution));
    int y1 = static_cast<int>(floor((y1f - origin_y) / resolution));

    get<0>(ret) = false;
    get<1>(ret) = 0.0f;
    geometry_msgs::msg::Point32 pt;
    pt.x = x1f;
    pt.y = y1f;
    pt.z = 0.0f;
    get<2>(ret) = pt;

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x1 >= x0) ? 1 : -1;
    int sy = (y1 >= y0) ? 1 : -1;
    int err = dx - dy;

    int x = x0;
    int y = y0;

    while (x >= 0 && x < width && y >= 0 && y < height)
    {
        float wx = origin_x + (x + 0.5f) * resolution;
        float wy = origin_y + (y + 0.5f) * resolution;
        float distance = hypot(wx - p0.x, wy - p0.y);

        if (distance > max_range)
        {
            get<0>(ret) = false;
            get<1>(ret) = max_range;
            geometry_msgs::msg::Point32 end_pt;
            end_pt.x = p0.x + max_range * cos(angle_rad);
            end_pt.y = p0.y + max_range * sin(angle_rad);
            end_pt.z = 0.0f;
            get<2>(ret) = end_pt;

            return ret;
        }

        int index = y * width + x;

        if (data[index] > map_occupied_threshold)
        {
            get<0>(ret) = true;
            get<1>(ret) = distance;
            geometry_msgs::msg::Point32 hit_pt;
            hit_pt.x = wx;
            hit_pt.y = wy;
            hit_pt.z = 0.0f;
            get<2>(ret) = hit_pt;

            return ret;
        }

        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }

    return ret;
}

float calc_laser_aabb_area(const sensor_msgs::msg::LaserScan &scan, const int &lidar_sampling_step)
{
    float min_x = numeric_limits<float>::max();
    float max_x = numeric_limits<float>::lowest();
    float min_y = numeric_limits<float>::max();
    float max_y = numeric_limits<float>::lowest();

#pragma omp parallel
    {
        float local_min_x = numeric_limits<float>::max();
        float local_max_x = numeric_limits<float>::lowest();
        float local_min_y = numeric_limits<float>::max();
        float local_max_y = numeric_limits<float>::lowest();

#pragma omp for nowait
        for (int i = 0; i < (int)scan.ranges.size(); i += lidar_sampling_step)
        {
            float angle = scan.angle_min + i * scan.angle_increment;
            float r = scan.ranges[i];

            if (isfinite(r) && r >= scan.range_min && r <= scan.range_max)
            {
                float x = r * cos(angle);
                float y = r * sin(angle);

                local_min_x = min(local_min_x, x);
                local_max_x = max(local_max_x, x);
                local_min_y = min(local_min_y, y);
                local_max_y = max(local_max_y, y);
            }
        }

#pragma omp critical
        {
            min_x = min(min_x, local_min_x);
            max_x = max(max_x, local_max_x);
            min_y = min(min_y, local_min_y);
            max_y = max(max_y, local_max_y);
        }
    }

    if (min_x > max_x || min_y > max_y)
        return 0.0f;

    return (max_x - min_x) * (max_y - min_y);
}

float calc_laser_obb_area(const sensor_msgs::msg::LaserScan &scan, const int &lidar_sampling_step)
{
    vector<cv::Point2f> points;

#pragma omp parallel
    {
        vector<cv::Point2f> local_points;
#pragma omp for nowait
        for (int i = 0; i < (int)scan.ranges.size(); i += lidar_sampling_step)
        {
            float angle = scan.angle_min + i * scan.angle_increment;
            float r = scan.ranges[i];

            if (isfinite(r) && r >= scan.range_min && r <= scan.range_max)
            {
                float x = r * cos(angle);
                float y = r * sin(angle);
                local_points.emplace_back(x, y);
            }
        }

#pragma omp critical
        points.insert(points.end(), local_points.begin(), local_points.end());
    }

    if (points.size() < 3)
        return 0.0f;

    cv::RotatedRect obb = cv::minAreaRect(points);

    return obb.size.width * obb.size.height;
}

float calc_laser_mean_dist(const sensor_msgs::msg::LaserScan &scan, const float &sensing_radius, const int &lidar_sampling_step)
{
    if (lidar_sampling_step <= 0)
    {
        return 0.0f;
    }

    const size_t n = scan.ranges.size();
    double sum = 0.0;
    size_t cnt = 0;

    if (n >= 2000)
    {
#pragma omp parallel for reduction(+ : sum, cnt) schedule(static)
        for (size_t i = 0; i < n; i += lidar_sampling_step)
        {
            const float r = scan.ranges[i];

            if (isfinite(r) && r <= sensing_radius && r > 0.0f)
            {
                sum += r;
                cnt++;
            }
        }
    }
    else
    {
        for (size_t i = 0; i < n; i += lidar_sampling_step)
        {
            const float r = scan.ranges[i];

            if (isfinite(r) && r <= sensing_radius && r > 0.0f)
            {
                sum += r;
                cnt++;
            }
        }
    }

    return (cnt > 0) ? static_cast<float>(sum / cnt) : 0.0f;
}

bool is_segment_no_collision(const float &x0, const float &y0, const float &x1, const float &y1, const int &step,
                             const int &width, const int &height, const float &resolu,
                             const float &min_obs_dist,
                             const vector<float> &obs_dist_table)
{
    auto grid_indices = enumerate_line_indices(x0, y0, x1, y1, width, height, resolu);

    size_t grid_indices_size = grid_indices.size();

    for (int i = 0;; i += step)
    {
        if (i >= grid_indices_size - 1)
        {
            i = grid_indices_size - 1;
        }

        int grid_index = grid_indices[i];

        if (grid_index >= 0 && grid_index < obs_dist_table.size())
        {
            if (obs_dist_table[grid_index] < min_obs_dist)
            {
                return false;
            }
        }

        if (i >= grid_indices_size - 1)
        {
            break;
        }
    }

    return true;
}

exploration_segment_type exploration_check_segment_type(const geometry_msgs::msg::Point32 &p0, const geometry_msgs::msg::Point32 &p1, const int &step,
                                                        const float &min_obs_dist,
                                                        const nav_msgs::msg::OccupancyGrid &grid,
                                                        const vector<float> &obs_dist_table,
                                                        const bool &check_obs,
                                                        const int &free_threshold, const int &occupied_threshold)
{
    auto grid_indices = enumerate_line_indices(p0, p1, grid);

    size_t grid_indices_size = grid_indices.size();

    bool unknow_flag = false;

    for (int i = 0;; i += step)
    {
        if (i >= grid_indices_size - 1)
        {
            i = grid_indices_size - 1;
        }

        int grid_index = grid_indices[i];

        if (grid_index >= 0 && grid_index < obs_dist_table.size())
        {
            if (check_obs)
            {
                if (obs_dist_table[grid_index] <= min_obs_dist)
                {
                    return exploration_segment_type::WITH_COLLISION;
                }
            }

            if (grid.data[grid_index] < 0 ||
                (grid.data[grid_index] > free_threshold && grid.data[grid_index] < occupied_threshold))
            {
                unknow_flag = true;
            }
        }
        else
        {
            unknow_flag = true;
        }

        if (i >= grid_indices_size - 1)
        {
            break;
        }
    }

    return unknow_flag ? exploration_segment_type::NO_COLLISION_WITH_UNKNOW : exploration_segment_type::NO_COLLISION;
}

nav_msgs::msg::OccupancyGrid clear_footprint_on_grid(const nav_msgs::msg::OccupancyGrid &grid,
                                                const geometry_msgs::msg::Point32 &origin, const float &radius)
{
    nav_msgs::msg::OccupancyGrid ret = grid;

    size_t size = ret.data.size();

#pragma omp parallel for
    for (int i = 0; i < size; i++)
    {
        if (calculate_distance(grid_index_to_position(i, ret), origin) <= radius + 0.01)
        {
            ret.data[i] = 0;
        }
    }

    return ret;
}

vector<float> calc_min_dist_to_obs_table(const nav_msgs::msg::OccupancyGrid &grid, const int &threshold)
{
    const int width = (int)grid.info.width;
    const int height = (int)grid.info.height;
    const float resolution = grid.info.resolution;

    const float INF = 1e20f;

    vector<float> g(width * height, INF);
    vector<float> d2(width * height, INF);

    auto dt_1d = [&](const float *f, const int n, float *d,
                     vector<int> &v, vector<float> &z)
    {
        int first = -1;

        for (int i = 0; i < n; i++)
        {
            if (f[i] < INF * 0.5f)
            {
                first = i;

                break;
            }
        }

        if (first < 0)
        {
            for (int q = 0; q < n; q++)
            {
                d[q] = INF;
            }

            return;
        }

        int k = 0;
        v[0] = first;
        z[0] = -INF;
        z[1] = +INF;

        for (int q = first + 1; q < n; q++)
        {
            if (f[q] >= INF * 0.5f)
            {
                continue;
            }

            float s = 0.0f;

            while (true)
            {
                const int p = v[k];

                s = ((f[q] + (float)(q * q)) - (f[p] + (float)(p * p))) / (2.0f * (float)(q - p));

                if (s <= z[k])
                {
                    k--;

                    if (k < 0)
                    {
                        k = 0;
                        break;
                    }
                }
                else
                {
                    break;
                }
            }

            k++;
            v[k] = q;
            z[k] = s;
            z[k + 1] = +INF;
        }

        k = 0;

        for (int q = 0; q < n; q++)
        {
            while (z[k + 1] < (float)q)
            {
                k++;
            }

            const int p = v[k];

            d[q] = (float)((q - p) * (q - p)) + f[p];
        }
    };

#pragma omp parallel
    {
        vector<float> f(height);
        vector<float> d(height);
        vector<int> v(height);
        vector<float> z(height + 1);

#pragma omp for schedule(static)
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                const int idx = y * width + x;
                f[y] = (grid.data[idx] >= threshold) ? 0.0f : INF;
            }

            dt_1d(f.data(), height, d.data(), v, z);

            for (int y = 0; y < height; y++)
            {
                const int idx = y * width + x;

                g[idx] = d[y];
            }
        }
    }

#pragma omp parallel
    {
        vector<float> f(width);
        vector<float> d(width);
        vector<int> v(width);
        vector<float> z(width + 1);

#pragma omp for schedule(static)
        for (int y = 0; y < height; y++)
        {
            const int row_base = y * width;

            for (int x = 0; x < width; x++)
            {
                f[x] = g[row_base + x];
            }

            dt_1d(f.data(), width, d.data(), v, z);

            for (int x = 0; x < width; x++)
            {
                d2[row_base + x] = d[x];
            }
        }
    }

    vector<float> min_dist_table(width * height, numeric_limits<float>::max());

#pragma omp parallel for schedule(static)
    for (int idx = 0; idx < width * height; idx++)
    {
        if (grid.data[idx] >= threshold)
        {
            min_dist_table[idx] = 0.0f;
        }
        else
        {
            const float val = d2[idx];

            if (val >= INF * 0.5f)
            {
                min_dist_table[idx] = numeric_limits<float>::max();
            }
            else
            {
                min_dist_table[idx] = sqrt(val) * resolution;
            }
        }
    }

    return min_dist_table;
}

vector<float> calc_min_dist_to_obs_table(const nav_msgs::msg::OccupancyGrid &grid,
                                         const float center_x, const float center_y, const float radius,
                                         const int &threshold)
{
    const int width = (int)grid.info.width;
    const int height = (int)grid.info.height;
    const float resolution = grid.info.resolution;
    const float origin_x = grid.info.origin.position.x;
    const float origin_y = grid.info.origin.position.y;

    const float INF = 1e20f;
    const float r2 = radius * radius;

    auto clampi = [](int v, int lo, int hi)
    {
        return max(lo, min(hi, v));
    };

    int xmin = (int)floor((center_x - radius - origin_x) / resolution);
    int xmax = (int)ceil((center_x + radius - origin_x) / resolution);
    int ymin = (int)floor((center_y - radius - origin_y) / resolution);
    int ymax = (int)ceil((center_y + radius - origin_y) / resolution);

    xmin = clampi(xmin, 0, width - 1);
    xmax = clampi(xmax, 0, width - 1);
    ymin = clampi(ymin, 0, height - 1);
    ymax = clampi(ymax, 0, height - 1);

    vector<float> min_dist_table(width * height, numeric_limits<float>::max());

    if (xmin > xmax || ymin > ymax)
    {
        return min_dist_table;
    }

    const int roi_w = xmax - xmin + 1;
    const int roi_h = ymax - ymin + 1;

    vector<float> g_roi(roi_w * roi_h, INF);
    vector<float> d2_roi(roi_w * roi_h, INF);

    auto dt_1d = [&](const float *f, const int n, float *d,
                     vector<int> &v, vector<float> &z)
    {
        int first = -1;

        for (int i = 0; i < n; i++)
        {
            if (f[i] < INF * 0.5f)
            {
                first = i;
                break;
            }
        }

        if (first < 0)
        {
            for (int q = 0; q < n; q++)
            {
                d[q] = INF;
            }
            return;
        }

        int k = 0;

        v[0] = first;
        z[0] = -INF;
        z[1] = +INF;

        for (int q = first + 1; q < n; q++)
        {
            if (f[q] >= INF * 0.5f)
            {
                continue;
            }

            float s = 0.0f;

            while (true)
            {
                const int p = v[k];

                s = ((f[q] + (float)(q * q)) - (f[p] + (float)(p * p))) / (2.0f * (float)(q - p));

                if (s <= z[k])
                {
                    k--;

                    if (k < 0)
                    {
                        k = 0;
                        break;
                    }
                }
                else
                {
                    break;
                }
            }

            k++;
            v[k] = q;
            z[k] = s;
            z[k + 1] = +INF;
        }

        k = 0;
        for (int q = 0; q < n; q++)
        {
            while (z[k + 1] < (float)q)
            {
                k++;
            }
            const int p = v[k];
            d[q] = (float)((q - p) * (q - p)) + f[p];
        }
    };

    int found_occ = 0;

#pragma omp parallel
    {
        vector<float> f(roi_h);
        vector<float> d(roi_h);
        vector<int> v(roi_h);
        vector<float> z(roi_h + 1);

        int local_found = 0;

#pragma omp for schedule(static)
        for (int i = 0; i < roi_w; i++)
        {
            const int x = xmin + i;
            const float wx = origin_x + (float)x * resolution;

            for (int j = 0; j < roi_h; j++)
            {
                const int y = ymin + j;
                const float wy = origin_y + (float)y * resolution;

                const float dx = wx - center_x;
                const float dy = wy - center_y;

                if (dx * dx + dy * dy > r2)
                {
                    f[j] = INF;
                    continue;
                }

                const int idx = y * width + x;

                if (grid.data[idx] >= threshold)
                {
                    f[j] = 0.0f;

                    local_found = 1;
                }
                else
                {
                    f[j] = INF;
                }
            }

            dt_1d(f.data(), roi_h, d.data(), v, z);

            for (int j = 0; j < roi_h; j++)
            {
                g_roi[j * roi_w + i] = d[j];
            }
        }

#pragma omp atomic
        found_occ |= local_found;
    }

    if (!found_occ)
    {
        return min_dist_table;
    }

#pragma omp parallel
    {
        vector<float> f(roi_w);
        vector<float> d(roi_w);
        vector<int> v(roi_w);
        vector<float> z(roi_w + 1);

#pragma omp for schedule(static)
        for (int j = 0; j < roi_h; j++)
        {
            for (int i = 0; i < roi_w; i++)
            {
                f[i] = g_roi[j * roi_w + i];
            }

            dt_1d(f.data(), roi_w, d.data(), v, z);

            for (int i = 0; i < roi_w; i++)
            {
                d2_roi[j * roi_w + i] = d[i];
            }
        }
    }

#pragma omp parallel for schedule(static)
    for (int j = 0; j < roi_h; j++)
    {
        const int y = ymin + j;
        const float wy = origin_y + (float)y * resolution;

        for (int i = 0; i < roi_w; i++)
        {
            const int x = xmin + i;
            const float wx = origin_x + (float)x * resolution;

            const float dx = wx - center_x;
            const float dy = wy - center_y;

            if (dx * dx + dy * dy > r2)
            {
                continue;
            }

            const int idx = y * width + x;

            if (grid.data[idx] >= threshold)
            {
                min_dist_table[idx] = 0.0f;
                continue;
            }

            const float val = d2_roi[j * roi_w + i];
            
            if (val >= INF * 0.5f)
            {
                continue;
            }

            min_dist_table[idx] = sqrt(val) * resolution;
        }
    }

    return min_dist_table;
}

float calc_min_dist_to_obs(const nav_msgs::msg::OccupancyGrid &grid, const vector<float> &min_dist_to_obs_table, const geometry_msgs::msg::Point32 &position)
{
    if (!is_point_in_map(position, grid)) return 20.0f;
    const auto index = position_to_grid_index(position, grid);
    if (grid.data[index] < 0) return 20.0f;
    return std::min(20.0f, min_dist_to_obs_table[index]);
}

float calc_min_dist_to_obs(const nav_msgs::msg::OccupancyGrid &grid, const vector<float> &min_dist_to_obs_table, const geometry_msgs::msg::PoseStamped &pose)
{
    return calc_min_dist_to_obs(grid, min_dist_to_obs_table, create_point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
}

float calc_min_dist_to_obs(const nav_msgs::msg::OccupancyGrid &grid, const vector<float> &min_dist_to_obs_table, const geometry_msgs::msg::TransformStamped &tf)
{
    return calc_min_dist_to_obs(grid, min_dist_to_obs_table, create_point(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z));
}

vector<int> resample_points(const vector<geometry_msgs::msg::Point32> &points, const vector<int> &indices,
                            const float &min_dist)
{
    if (points.empty() || indices.empty())
    {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>());

    size_t size = points.size();

    for (int i = 0; i < size; i++)
    {
        pcl::PointXYZL pcl_point;
        pcl_point.x = points[i].x;
        pcl_point.y = points[i].y;
        pcl_point.label = indices[i];
        cloud->points.push_back(pcl_point);
    }

    pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZL>());
    pcl::UniformSampling<pcl::PointXYZL> downsample_filter;
    downsample_filter.setInputCloud(cloud);
    downsample_filter.setRadiusSearch(min_dist);
    downsample_filter.filter(*filtered_cloud);

    vector<int> resampled_point_indices;

    for (const auto &pcl_point : filtered_cloud->points)
    {
        resampled_point_indices.push_back(pcl_point.label);
    }

    if (resampled_point_indices.empty())
    {
        resampled_point_indices.push_back(indices[0]);
    }

    return resampled_point_indices;
}

vector<geometry_msgs::msg::Point32> resample_points(const vector<geometry_msgs::msg::Point32> &points, const float &min_dist)
{
    if (points.empty())
    {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    size_t size = points.size();

    for (int i = 0; i < size; i++)
    {
        pcl::PointXYZ pcl_point;
        pcl_point.x = points[i].x;
        pcl_point.y = points[i].y;
        cloud->points.push_back(pcl_point);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(min_dist, min_dist, 1.0f);
    voxel_filter.filter(*filtered_cloud);

    vector<geometry_msgs::msg::Point32> resampled_points;

    for (const auto &pcl_point : filtered_cloud->points)
    {
        geometry_msgs::msg::Point32 point;
        point.x = pcl_point.x;
        point.y = pcl_point.y;
        resampled_points.push_back(point);
    }

    if (resampled_points.empty())
    {
        resampled_points.push_back(points[0]);
    }

    return resampled_points;
}

geometry_msgs::msg::Point32 generate_point(const geometry_msgs::msg::Point32 &p0, const geometry_msgs::msg::Point32 &dir, const float &dist)
{
    geometry_msgs::msg::Point32 new_point;

    if (calculate_distance(p0, dir) <= dist)
    {
        new_point = dir;

        return new_point;
    }

    float magnitude = sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);

    if (magnitude <= 1e-8)
    {
        new_point.x = p0.x;
        new_point.y = p0.y + dist;
        new_point.z = p0.z;
    }
    else
    {
        geometry_msgs::msg::Point32 normalized_dir;
        normalized_dir.x = dir.x / magnitude;
        normalized_dir.y = dir.y / magnitude;
        normalized_dir.z = dir.z / magnitude;

        new_point.x = p0.x + normalized_dir.x * dist;
        new_point.y = p0.y + normalized_dir.y * dist;
        new_point.z = p0.z + normalized_dir.z * dist;
    }

    return new_point;
}

geometry_msgs::msg::Point32 generate_point_on_segment(const geometry_msgs::msg::Point32 &p0, const geometry_msgs::msg::Point32 &dir, const float &dist)
{
    geometry_msgs::msg::Point32 new_point;

    float magnitude = sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);

    if (magnitude <= 1e-8)
    {
        new_point = p0;
    }
    else
    {
        geometry_msgs::msg::Point32 normalized_dir;
        normalized_dir.x = dir.x / magnitude;
        normalized_dir.y = dir.y / magnitude;
        normalized_dir.z = dir.z / magnitude;

        new_point.x = p0.x + normalized_dir.x * dist;
        new_point.y = p0.y + normalized_dir.y * dist;
        new_point.z = p0.z + normalized_dir.z * dist;
    }

    return new_point;
}

float calc_unknow_area(const geometry_msgs::msg::Point32 &p, const float &radius, const nav_msgs::msg::OccupancyGrid &grid,
                       const int &free_threshold, const int &occupied_threshold)
{
    float resolution = grid.info.resolution;
    float origin_x = grid.info.origin.position.x;
    float origin_y = grid.info.origin.position.y;
    int width = grid.info.width;
    int height = grid.info.height;

    int center_x = static_cast<int>((p.x - origin_x) / resolution);
    int center_y = static_cast<int>((p.y - origin_y) / resolution);
    int radius_in_cells = static_cast<int>(radius / resolution);

    int unknown_count = 0;

    for (int y = -radius_in_cells; y <= radius_in_cells; y++)
    {
        for (int x = -radius_in_cells; x <= radius_in_cells; x++)
        {
            int grid_x = center_x + x;
            int grid_y = center_y + y;

            if (x * x + y * y <= radius_in_cells * radius_in_cells)
            {
                if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height)
                {
                    int index = grid_y * width + grid_x;

                    if (index >= 0 && index < grid.data.size())
                    {
                        if (grid.data[index] < 0 || (grid.data[index] > free_threshold && grid.data[index] < occupied_threshold))
                        {
                            unknown_count++;
                        }
                    }
                }
            }
        }
    }

    float unknown_area = unknown_count * resolution * resolution;

    return unknown_area;
}

tuple<int, geometry_msgs::msg::Point32> get_nearest_point(const geometry_msgs::msg::Point32 &p,
                                                     const vector<geometry_msgs::msg::Point32> &points, const vector<int> &indices)
{
    int index = -1;
    geometry_msgs::msg::Point32 nearest_point;
    float min_dist = INFINITY;

    size_t size = points.size();

    for (int i = 0; i < size; i++)
    {
        float dist = calculate_distance(p, points[i]);

        if (dist < min_dist && dist > 1e-8)
        {
            index = indices[i];
            min_dist = dist;
            nearest_point.x = points[i].x;
            nearest_point.y = points[i].y;
            nearest_point.z = points[i].z;
        }
    }

    return {index, nearest_point};
}

tuple<int, geometry_msgs::msg::Point32> get_nearest_point(const geometry_msgs::msg::Point32 &p,
                                                     const vector<geometry_msgs::msg::Point32> &points, const vector<int> &indices, const vector<bool> &del_states)
{
    int index = -1;
    geometry_msgs::msg::Point32 nearest_point;
    float min_dist = INFINITY;

    size_t size = points.size();

    for (int i = 0; i < size; i++)
    {
        if (del_states[i])
        {
            continue;
        }

        float dist = calculate_distance(p, points[i]);

        if (dist < min_dist && dist > 1e-8)
        {
            index = indices[i];
            min_dist = dist;
            nearest_point.x = points[i].x;
            nearest_point.y = points[i].y;
            nearest_point.z = points[i].z;
        }
    }

    return {index, nearest_point};
}

bool is_point_in_map(const geometry_msgs::msg::Point32 &p, const nav_msgs::msg::OccupancyGrid &grid)
{
    auto [x, y] = position_to_grid_xy(p, grid);

    if (x < 0 || x >= grid.info.width || y < 0 || y >= grid.info.height)
    {
        return false;
    }

    auto index = y * grid.info.width + x;

    return index >= 0 && index < static_cast<int>(grid.data.size());
}

bool is_point_unknow(const geometry_msgs::msg::Point32 &p, const nav_msgs::msg::OccupancyGrid &grid,
                     const int &free_threshold, const int &occupied_threshold)
{
    auto index = position_to_grid_index(p, grid);

    if (free_threshold < 0 || occupied_threshold < 0)
    {
        return grid.data[index] < 0;
    }

    return grid.data[index] < 0 || (grid.data[index] > free_threshold && grid.data[index] < occupied_threshold);
}

float calc_decayed_distance(const geometry_msgs::msg::Point32 &p, const nav_msgs::msg::OccupancyGrid &grid, const vector<float> &obs_dist_table,
                            const float &min_dist_to_obs, const float &min_expand_dist, const float &max_expand_dist)
{
    float ret = max_expand_dist;

    int index = position_to_grid_index(p, grid);

    if (index >= 0 && index < obs_dist_table.size() && grid.data.size() == obs_dist_table.size())
    {
        float dist = obs_dist_table[index];

        if (dist >= min_dist_to_obs && dist <= 2.0f * min_dist_to_obs)
        {
            float t = (dist - min_dist_to_obs) / (min_dist_to_obs);
            ret = min_expand_dist + t * (max_expand_dist - min_expand_dist);
        }
        else if (dist > 2.0f * min_dist_to_obs)
        {
            ret = max_expand_dist;
        }
    }

    return ret;
}

float calc_orientation_diff(const geometry_msgs::msg::Point32 &p0, const geometry_msgs::msg::Point32 &p1, const geometry_msgs::msg::Point32 &p2)
{
    float dx1 = p1.x - p0.x;
    float dy1 = p1.y - p0.y;
    float dx2 = p2.x - p1.x;
    float dy2 = p2.y - p1.y;

    float mag1 = sqrt(dx1 * dx1 + dy1 * dy1);
    float mag2 = sqrt(dx2 * dx2 + dy2 * dy2);

    if (mag1 < 1e-8 || mag2 < 1e-8)
    {
        return 0.0f;
    }

    dx1 /= mag1;
    dy1 /= mag1;
    dx2 /= mag2;
    dy2 /= mag2;

    float dot_product = dx1 * dx2 + dy1 * dy2;
    float cross_product = dx1 * dy2 - dy1 * dx2;

    float angle_diff = atan2(cross_product, dot_product);

    return abs(angle_diff);
}

tuple<float, float> get_grid_max_min(const nav_msgs::msg::OccupancyGrid &grid)
{
    float max_value = numeric_limits<float>::lowest();
    float min_value = numeric_limits<float>::max();

    for (const auto &value : grid.data)
    {
        if (value >= 254 || value <= 0)
        {
            continue;
        }

        if (value > max_value)
        {
            max_value = value;
        }
        if (value < min_value)
        {
            min_value = value;
        }
    }

    if (max_value == numeric_limits<float>::lowest() ||
        min_value == numeric_limits<float>::max())
    {
        return make_tuple(-1.0f, -1.0f);
    }

    return make_tuple(max_value, min_value);
}

vector<geometry_msgs::msg::Point32> get_round_points(const geometry_msgs::msg::Point32 &p,
                                                const float &radius, const int &num,
                                                const nav_msgs::msg::OccupancyGrid &grid,
                                                const float &min_obs_dist, const vector<float> &obs_dist_table)
{
    vector<geometry_msgs::msg::Point32> points;
    points.reserve(num + 1);

    points.push_back(p);

    for (int i = 0; i < num; i++)
    {
        double theta = 2.0 * M_PI * i / num;

        geometry_msgs::msg::Point32 pt;

        pt.x = p.x + radius * cos(theta);
        pt.y = p.y + radius * sin(theta);
        pt.z = p.z;

        int index = position_to_grid_index(pt, grid);

        if (index >= 0 && index < grid.data.size())
        {
            if (obs_dist_table[index] >= min_obs_dist + 0.03)
            {
                points.push_back(pt);
            }
        }
    }

    return points;
}

vector<geometry_msgs::msg::PoseStamped> get_round_points(const geometry_msgs::msg::PoseStamped &p,
                                                    const float &radius, const int &num,
                                                    const nav_msgs::msg::OccupancyGrid &grid,
                                                    const float &min_obs_dist, const vector<float> &obs_dist_table)
{
    vector<geometry_msgs::msg::PoseStamped> points;
    points.reserve(num);

    geometry_msgs::msg::Point32 p1;
    p1.x = p.pose.position.x;
    p1.y = p.pose.position.y;
    p1.z = p.pose.position.z;

    auto result = get_round_points(p1, radius, num, grid, min_obs_dist, obs_dist_table);

    size_t size = result.size();

    for (int i = 0; i < size; i++)
    {
        geometry_msgs::msg::PoseStamped pose = p;
        pose.pose.position.x = result[i].x;
        pose.pose.position.y = result[i].y;
        pose.pose.position.z = result[i].z;

        points.push_back(pose);
    }

    return points;
}

float calc_seg_direction(const geometry_msgs::msg::PoseStamped &p0, const geometry_msgs::msg::Point32 &p1)
{
    float dx = p1.x - p0.pose.position.x;
    float dy = p1.y - p0.pose.position.y;

    float direction = atan2(dy, dx);

    return direction;
}

float calc_map_area(const nav_msgs::msg::OccupancyGrid &grid,
                    const int &free_threshold, const int &occupied_threshold)
{
    const double cell_area = grid.info.resolution * grid.info.resolution;

    uint64_t valid_count = 0;
    const size_t n = grid.data.size();

#pragma omp parallel for reduction(+ : valid_count) schedule(static)
    for (size_t i = 0; i < n; i++)
    {
        int8_t v = grid.data[i];

        if (v >= 0 && (v <= free_threshold || v >= occupied_threshold))
        {
            valid_count += 1;
        }
    }

    return static_cast<float>(valid_count * cell_area);
}
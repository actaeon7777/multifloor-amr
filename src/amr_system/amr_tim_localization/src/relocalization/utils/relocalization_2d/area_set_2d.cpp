/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <relocalization/utils/relocalization_2d/area_set_2d.h>

namespace area_set_2d
{
    area_set_2d::area_set_2d(const bool &use_traversability_sampling,
                             const float &rrt_min_expand_dist, const float &rrt_max_expand_dist, const int &traversability_sampling_duration,
                             const int &map_free_threshold, const int &map_occupied_threshold,
                             const float &sensing_radius, const float &anchor_point_min_dist,
                             const float &angle_search_step)
        : use_traversability_sampling_(use_traversability_sampling),
          rrt_min_expand_dist_(rrt_min_expand_dist), rrt_max_expand_dist_(rrt_max_expand_dist), traversability_sampling_duration_(traversability_sampling_duration),
          map_free_threshold_(map_free_threshold), map_occupied_threshold_(map_occupied_threshold),
          sensing_radius_(sensing_radius), anchor_point_min_dist_(anchor_point_min_dist),
          angle_search_step_(angle_search_step)
    {

        return;
    };

    nav_msgs::msg::OccupancyGrid area_set_2d::get_area_submap(const nav_msgs::msg::OccupancyGrid &map,
                                                         const sensor_msgs::msg::LaserScan &scan,
                                                         const bool &lidar_reverted,
                                                         const int &lidar_sampling_step,
                                                         const geometry_msgs::msg::Point32 &anchor_point,
                                                         const float &max_radius)
    {
        const float resolution = map.info.resolution;
        const int width = map.info.width;
        const int height = map.info.height;
        const float origin_x = map.info.origin.position.x;
        const float origin_y = map.info.origin.position.y;
        const vector<int8_t> &data = map.data;

        const int beam_count = scan.ranges.size();

        vector<pair<int, int>> hit_cells;

#pragma omp parallel
        {
            vector<pair<int, int>> local_hits;

#pragma omp for nowait
            for (int i = 0; i < beam_count; i += lidar_sampling_step)
            {
                float angle = scan.angle_min + i * scan.angle_increment;

                if (lidar_reverted)
                {
                    angle = angles::normalize_angle(M_PI - angle);
                }

                auto result = raycast_to_first_obstacle(anchor_point, angle, max_radius, map, map_occupied_threshold_);
                bool hit = get<0>(result);
                geometry_msgs::msg::Point32 hit_point = get<2>(result);

                if (hit)
                {
                    int mx = static_cast<int>(floor((hit_point.x - origin_x) / resolution));
                    int my = static_cast<int>(floor((hit_point.y - origin_y) / resolution));

                    if (mx >= 0 && mx < width && my >= 0 && my < height)
                    {
                        local_hits.emplace_back(mx, my);
                    }
                }
            }

#pragma omp critical
            hit_cells.insert(hit_cells.end(), local_hits.begin(), local_hits.end());
        }

        if (hit_cells.empty())
        {
            return nav_msgs::msg::OccupancyGrid();
        }

        int min_x = width, max_x = 0, min_y = height, max_y = 0;

        for (const auto &cell : hit_cells)
        {
            min_x = min(min_x, cell.first);
            max_x = max(max_x, cell.first);
            min_y = min(min_y, cell.second);
            max_y = max(max_y, cell.second);
        }

        nav_msgs::msg::OccupancyGrid ret;
        ret.info.resolution = resolution;
        ret.info.width = max_x - min_x + 1;
        ret.info.height = max_y - min_y + 1;
        ret.info.origin.position.x = origin_x + min_x * resolution;
        ret.info.origin.position.y = origin_y + min_y * resolution;
        ret.info.origin.position.z = 0.0;
        ret.info.origin.orientation.w = 1.0;
        ret.data.resize(ret.info.width * ret.info.height, 0);

        for (const auto &cell : hit_cells)
        {
            int full_x = cell.first;
            int full_y = cell.second;
            int local_x = full_x - min_x;
            int local_y = full_y - min_y;

            int global_idx = full_y * width + full_x;
            int local_idx = local_y * ret.info.width + local_x;

            ret.data[local_idx] = data[global_idx];
        }

        return ret;
    }

    vector<float> area_set_2d::get_mean_dist_list(const nav_msgs::msg::OccupancyGrid &map,
                                                  const sensor_msgs::msg::LaserScan &scan,
                                                  const bool &lidar_reverted,
                                                  const int &lidar_sampling_step,
                                                  const geometry_msgs::msg::Point32 &anchor_point,
                                                  const float &max_radius)
    {
        vector<float> ret;

        float range_total = 0.0f;
        int counter = 0;
        float range_mean = 0.0f;
        float scan_fov = abs(scan.angle_max - scan.angle_min);
        bool scan_full_fov = scan_fov >= (350.0f / 360.0f) * 2.0f * M_PI;

        unique_ptr<sector_query_2d::sector_query_2d> sector_query_instance;
        unique_ptr<sector_query_2d::sector_query_2d> sector_valid_num_query_instance;

        // Bound the synthetic full-circle sweep independently from a partial
        // scan's angular increment (tiny FOV must not allocate millions of rays).
        const int ray_count = std::clamp(static_cast<int>(std::floor(
            (2.0 * M_PI) / (std::abs(scan.angle_increment) * lidar_sampling_step))), 12, 1440);
        const float synthetic_angle_step = 2.0f * M_PI / ray_count;
        if (!scan_full_fov)
        {
            // reset sector query instances
            int sector_num = ray_count;
            sector_query_instance = make_unique<sector_query_2d::sector_query_2d>(sector_num);
            sector_valid_num_query_instance = make_unique<sector_query_2d::sector_query_2d>(sector_num);
        }


        vector<tuple<float, tuple<bool, float, geometry_msgs::msg::Point32>>> raycast_results(ray_count);

#pragma omp parallel for
        for (int i = 0; i < ray_count; i++)
        {
            if (tim_port::expired()) continue;
            float angle = i * synthetic_angle_step;
            auto result = raycast_to_first_obstacle(anchor_point, angle, max_radius, map, map_occupied_threshold_);

            raycast_results[i] = make_tuple(angle, result);
        }

        for (const auto &[angle, result] : raycast_results)
        {
            if (get<0>(result) && get<1>(result) > 0.0f && get<1>(result) <= max_radius)
            {
                if (scan_full_fov)
                {
                    range_total += get<1>(result);
                    counter++;
                }
                else
                {
                    sector_query_instance->add_range(angle, get<1>(result));
                    sector_valid_num_query_instance->add_range(angle, 1);
                }
            }
        }

        ret.clear();

        // get results
        if (scan_full_fov)
        {
            range_mean = counter > 0 ? (range_total / (float)counter) : 0.0f;

            ret.emplace_back(range_mean);
        }
        else
        {
            int yaw_steps = static_cast<int>(std::ceil(2 * M_PI / angle_search_step_));

            ret.resize(yaw_steps, -1.0f);

#pragma omp parallel for
            for (int i = 0; i < yaw_steps; i++)
            {
                float yaw = i * (2.0 * M_PI / yaw_steps);

                int count = static_cast<int>(round(
                    sector_valid_num_query_instance->get_sum(yaw + std::min(scan.angle_min, scan.angle_max), scan_fov)));

                if (count > 0)
                {
                    float sum = sector_query_instance->get_sum(yaw + std::min(scan.angle_min, scan.angle_max), scan_fov);
                    ret[i] = sum / static_cast<float>(count);
                }
            }

            ret.erase(remove_if(ret.begin(), ret.end(),
                                [](float v)
                                {
                                    return v < 0.0f;
                                }),
                      ret.end());
        }

        return ret;
    }

    vector<tuple<geometry_msgs::msg::Point32, vector<float>>> area_set_2d::get_area_set(const nav_msgs::msg::OccupancyGrid &map,
                                                                                   const sensor_msgs::msg::LaserScan &scan,
                                                                                   const bool &lidar_reverted,
                                                                                   const int &lidar_sampling_step,
                                                                                   const vector<float> &min_dist_to_obs_table,
                                                                                   float &min_dist_to_obstacle,
                                                                                   const bool &enable_visualization)
    {
        vector<tuple<geometry_msgs::msg::Point32, vector<float>>> ret;

        vector<geometry_msgs::msg::Point32> valid_points;

        sampling_2d::sampling_2d sampling(rrt_min_expand_dist_, rrt_max_expand_dist_, min_dist_to_obstacle,
                                          anchor_point_min_dist_, use_traversability_sampling_,
                                          map_free_threshold_, map_occupied_threshold_);

        sampling.initialize(map, min_dist_to_obs_table);

        valid_points = sampling.get_points(traversability_sampling_duration_);

        // get sub-area data
        auto mean_dist_calc_start_time = tim_port::WallTime::now();

#pragma omp parallel
        {
            vector<tuple<geometry_msgs::msg::Point32, vector<float>>> local_ret;

            size_t valid_points_size = valid_points.size();

#pragma omp for nowait
            for (int i = 0; i < valid_points_size; i++)
            {
                if (tim_port::expired()) continue;
                const auto &point = valid_points[i];

                vector<float> mean_dist_list = get_mean_dist_list(map, scan, lidar_reverted, lidar_sampling_step, point, sensing_radius_);

                local_ret.emplace_back(point, mean_dist_list);
            }

#pragma omp critical
            {
                ret.insert(ret.end(), local_ret.begin(), local_ret.end());
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("tim_core"), "SMAD computation time: %.3lfs.", (tim_port::WallTime::now() - mean_dist_calc_start_time).toSec());

        tim_port::checkpoint();
        // visualization


        return ret;
    }
}
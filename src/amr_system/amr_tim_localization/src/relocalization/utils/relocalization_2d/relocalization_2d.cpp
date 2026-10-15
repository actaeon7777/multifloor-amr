/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <relocalization/utils/relocalization_2d/relocalization_2d.h>

namespace relocalization_2d
{
    relocalization_2d::relocalization_2d(
        const bool &enable_visualization, const bool &use_traversability_sampling,
        const float &rrt_min_expand_dist, const float &rrt_max_expand_dist, const int &traversability_sampling_duration,
        const float &fit_score_threshold_min, const float &fit_score_threshold_max,
        const float &angle_search_step, const int &area_batch_proc_number, const int &area_candidate_number,
        const float &min_dist_to_obstacle,
        const float &sigma_hit, const float &z_hit, const float &z_rand, const float &max_tolerance_dist,
        const float &keep_ratio_1_min, const float &keep_ratio_1_max, const float &keep_ratio_2_min, const float &keep_ratio_2_max,
        const bool &geometric_mean_fusion,
        const int &map_free_threshold, const int &map_occupied_threshold,
        const float &sensing_radius, const float anchor_point_min_dist,
        const float &max_correspondence_distance, const float &transformation_epsilon, const float &euclidean_fitness_epsilon, const int &max_iterations)
        : enable_visualization_(enable_visualization), use_traversability_sampling_(use_traversability_sampling),
          rrt_min_expand_dist_(rrt_min_expand_dist), rrt_max_expand_dist_(rrt_max_expand_dist), traversability_sampling_duration_(traversability_sampling_duration),
          fit_score_threshold_min_(fit_score_threshold_min), fit_score_threshold_max_(fit_score_threshold_max),
          angle_search_step_(angle_search_step), area_batch_proc_number_(area_batch_proc_number), area_candidate_number_(area_candidate_number),
          min_dist_to_obstacle_(min_dist_to_obstacle),
          sigma_hit_(sigma_hit), z_hit_(z_hit), z_rand_(z_rand), max_tolerance_dist_(max_tolerance_dist),
          keep_ratio_1_min_(keep_ratio_1_min), keep_ratio_1_max_(keep_ratio_1_max), keep_ratio_2_min_(keep_ratio_2_min), keep_ratio_2_max_(keep_ratio_2_max),
          geometric_mean_fusion_(geometric_mean_fusion),
          map_free_threshold_(map_free_threshold), map_occupied_threshold_(map_occupied_threshold),
          sensing_radius_(sensing_radius), anchor_point_min_dist_(anchor_point_min_dist),
          max_correspondence_distance_(max_correspondence_distance), transformation_epsilon_(transformation_epsilon), euclidean_fitness_epsilon_(euclidean_fitness_epsilon), max_iterations_(max_iterations)
    {
        sensing_radius_ += anchor_point_min_dist_;

        area_set_2d_instance_ = unique_ptr<area_set_2d::area_set_2d>(new area_set_2d::area_set_2d(
            use_traversability_sampling,
            rrt_min_expand_dist, rrt_max_expand_dist, traversability_sampling_duration,
            map_free_threshold_, map_occupied_threshold_,
            sensing_radius_, anchor_point_min_dist_, angle_search_step_));

        likelihood_field_2d_instance_ = unique_ptr<likelihood_field_2d::likelihood_field_2d>(new likelihood_field_2d::likelihood_field_2d(
            sigma_hit_, z_hit_, z_rand_,
            keep_ratio_1_min_, keep_ratio_1_max_, keep_ratio_2_min_, keep_ratio_2_max_,
            map_occupied_threshold_));

        gicp_2d_instance_ = unique_ptr<gicp_2d::gicp_2d>(new gicp_2d::gicp_2d(
            max_correspondence_distance_,
            transformation_epsilon_,
            euclidean_fitness_epsilon_,
            max_iterations_,
            map_occupied_threshold_,
            enable_visualization_));


        return;
    };

    void relocalization_2d::set_map(const nav_msgs::msg::OccupancyGrid &map,
                                    const int &map_sampling_ratio)
    {
        RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Updating relocalization map data...");

        map_data_ = map;
        map_sampling_ratio_ = map_sampling_ratio;

        obs_dist_table_global_ = calc_min_dist_to_obs_table(
            map_data_, map_occupied_threshold_);

        gicp_2d_instance_->set_map(map_data_, map_sampling_ratio);

        RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Relocalization map data loaded successfully.");
    }

    tuple<bool, vector<float>, vector<geometry_msgs::msg::TransformStamped>> relocalization_2d::get_candidates(
        const sensor_msgs::msg::LaserScan &scan,
        const bool &lidar_reverted,
        const int &lidar_sampling_step,
        const int &map_sampling_ratio,
        const nav_msgs::msg::OccupancyGrid &map,
        const int &max_num_of_candidates)
    {
        tuple<bool, vector<float>, vector<geometry_msgs::msg::TransformStamped>> ret;
        get<0>(ret) = false;
        get<1>(ret).clear();
        get<2>(ret).clear();

        bool map_changed = false;

        if (map_data_.header.frame_id != map.header.frame_id ||
            map_data_.info.resolution != map.info.resolution ||
            map_data_.info.origin != map.info.origin ||
            map_data_.info.height != map.info.height ||
            map_data_.info.width != map.info.width ||
            map_data_.data != map.data ||
            map_sampling_ratio_ != map_sampling_ratio)
        {
            set_map(map, map_sampling_ratio);

            map_changed = true;
        }

        if (!map_changed)
        {
            RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Using cached relocalization map data.");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Relocalization map data updated.");
        }

        tim_port::checkpoint();
        auto sampling_start_time = tim_port::WallTime::now();

        bool sampling_changed = false;

        if (map_changed ||
            scan_data_.header.frame_id != scan.header.frame_id ||
            abs(scan_data_.angle_min - scan.angle_min) > 1e-3 ||
            abs(scan_data_.angle_max - scan.angle_max) > 1e-3 ||
            scan_data_.ranges.size() != scan.ranges.size() ||
            lidar_reverted_ != lidar_reverted ||
            lidar_sampling_step_ != lidar_sampling_step)
        {
            RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Updating relocalization sampling data...");

            scan_data_ = scan;
            lidar_reverted_ = lidar_reverted;
            lidar_sampling_step_ = lidar_sampling_step;

            area_set_raw_ = area_set_2d_instance_->get_area_set(
                map_data_, scan_data_, lidar_reverted_, lidar_sampling_step_,
                obs_dist_table_global_, min_dist_to_obstacle_,
                enable_visualization_);

            sampling_changed = true;
        }
        else
        {
            // to update timestamp
            scan_data_ = scan;
            lidar_reverted_ = lidar_reverted;
            lidar_sampling_step_ = lidar_sampling_step;
        }

        if (!sampling_changed)
        {
            RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Using cached relocalization sampling data.");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Relocalization sampling data updated.");
        }

        tim_port::checkpoint();
        gicp_2d_instance_->set_scan(scan_data_, lidar_sampling_step_);

        float scan_mean_dist = calc_laser_mean_dist(scan_data_, sensing_radius_, lidar_sampling_step_);

        auto sampling_end_time = tim_port::WallTime::now();
        auto matching_end_time = tim_port::WallTime::now();

        RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Total %ld observation regions available.", area_set_raw_.size());

        if (area_set_raw_.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("tim_core"), "No region available.");

            return ret;
        }

        RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Starting optimal region matching...");

        area_set_.clear();

        vector<tuple<geometry_msgs::msg::Point32, float>> area_set_parallel;

#pragma omp parallel
        {
            vector<tuple<geometry_msgs::msg::Point32, float>> local_result;

#pragma omp for nowait
            for (int i = 0; i < static_cast<int>(area_set_raw_.size()); i++)
            {
                const auto &sub_area_raw = area_set_raw_[i];

                float min_diff = numeric_limits<float>::max();
                float best_value = 0.0f;

                for (const auto &val : get<1>(sub_area_raw))
                {
                    float diff = abs(val - scan_mean_dist);

                    if (diff < min_diff)
                    {
                        min_diff = diff;
                        best_value = val;
                    }
                }

                local_result.emplace_back(get<0>(sub_area_raw), best_value);
            }

#pragma omp critical
            area_set_parallel.insert(area_set_parallel.end(), local_result.begin(), local_result.end());
        }

        area_set_ = move(area_set_parallel);

        sort(area_set_.begin(), area_set_.end(),
             [scan_mean_dist](const auto &a, const auto &b)
             {
                 return abs(get<1>(a) - scan_mean_dist) < abs(get<1>(b) - scan_mean_dist);
             });

        size_t area_set_size = area_set_.size();
        size_t start_idx = 0;
        int batch_idx = 0;

        geometry_msgs::msg::TransformStamped trans_best;
        float loc_conf_best = 0.0f;
        bool found_valid = false;
        bool early_terminated = false;

        bool early_termination_mode = (max_num_of_candidates <= 1);
        vector<tuple<float, geometry_msgs::msg::TransformStamped>> all_candidates;

        while (start_idx < area_set_size)
        {
            tim_port::checkpoint();
            RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Starting batch round %d...", ++batch_idx);

            size_t end_idx = min(start_idx + area_batch_proc_number_, area_set_size);

            vector<tuple<geometry_msgs::msg::Point32, float>> sub_area_batch(area_set_.begin() + start_idx, area_set_.begin() + end_idx);

            size_t sub_area_batch_size = sub_area_batch.size();

            vector<tuple<float, geometry_msgs::msg::TransformStamped, float>> candidate_sub_areas;

#pragma omp parallel
            {
                vector<tuple<float, geometry_msgs::msg::TransformStamped, float>> local_sub_areas;

#pragma omp for nowait
                for (int i = 0; i < static_cast<int>(sub_area_batch_size); i++)
                {
                    if (tim_port::expired()) continue;
                    geometry_msgs::msg::TransformStamped tf_map_to_lidar;
                    tf_map_to_lidar.header.frame_id = map_data_.header.frame_id;
                    tf_map_to_lidar.header.stamp = scan_data_.header.stamp;
                    tf_map_to_lidar.child_frame_id = scan_data_.header.frame_id;
                    tf_map_to_lidar.transform.translation.x = get<0>(sub_area_batch[i]).x;
                    tf_map_to_lidar.transform.translation.y = get<0>(sub_area_batch[i]).y;
                    tf_map_to_lidar.transform.rotation = tim_port::quaternion(lidar_reverted_ ? M_PI : 0, 0, 0);

                    geometry_msgs::msg::TransformStamped tf_map_to_lidar_area_best = tf_map_to_lidar;
                    float confidence_score_area_best = -INFINITY;

                    int yaw_steps = static_cast<int>(std::ceil(2 * M_PI / angle_search_step_));

                    for (int j = 0; j < yaw_steps; j++)
                    {
                        if (tim_port::expired()) break;
                        float yaw = j * (2.0 * M_PI / yaw_steps);

                        geometry_msgs::msg::TransformStamped tf_map_to_lidar_search = tf_map_to_lidar;
                        tf_map_to_lidar_search.transform.rotation =
                            tim_port::quaternion(lidar_reverted_ ? M_PI : 0, 0, yaw);

                        float confidence_score = likelihood_field_2d_instance_->get_confidence(
                            tf_map_to_lidar_search,
                            scan_data_, map_data_, obs_dist_table_global_,
                            max_tolerance_dist_, lidar_sampling_step_,
                            geometric_mean_fusion_);

                        if (confidence_score > confidence_score_area_best)
                        {
                            tf_map_to_lidar_area_best = tf_map_to_lidar_search;
                            confidence_score_area_best = confidence_score;
                        }
                    }

                    tuple<float, geometry_msgs::msg::TransformStamped, float> candidate_sub_area;
                    get<0>(candidate_sub_area) = confidence_score_area_best;
                    get<1>(candidate_sub_area) = tf_map_to_lidar_area_best;
                    get<2>(candidate_sub_area) = get<1>(sub_area_batch[i]);

                    local_sub_areas.emplace_back(candidate_sub_area);
                }

#pragma omp critical
                candidate_sub_areas.insert(candidate_sub_areas.end(),
                                           local_sub_areas.begin(), local_sub_areas.end());
            }

            tim_port::checkpoint();
            // sort region set
            RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Batch %d: Sorting region candidates...", batch_idx);

            sort(candidate_sub_areas.begin(), candidate_sub_areas.end(),
                 [](const auto &a, const auto &b)
                 {
                     return get<0>(a) > get<0>(b);
                 });

            vector<tuple<float, geometry_msgs::msg::TransformStamped, float>> candidate_transformations;

            size_t candidate_sub_area_size = candidate_sub_areas.size();
            size_t try_num = min(candidate_sub_area_size, (size_t)area_candidate_number_);

#pragma omp parallel for schedule(dynamic)
            for (int i = 0; i < static_cast<int>(try_num); i++)
            {
                if (tim_port::expired()) continue;
                geometry_msgs::msg::TransformStamped tf_map_to_lidar_aligned =
                    gicp_2d_instance_->match(get<1>(candidate_sub_areas[i]));

                float confidence_score = likelihood_field_2d_instance_->get_confidence(
                    tf_map_to_lidar_aligned,
                    scan_data_, map_data_, obs_dist_table_global_,
                    max_tolerance_dist_, lidar_sampling_step_,
                    geometric_mean_fusion_);

                tuple<float, geometry_msgs::msg::TransformStamped, float> candidate_transformation;
                get<0>(candidate_transformation) = confidence_score;
                get<1>(candidate_transformation) = tf_map_to_lidar_aligned;
                get<2>(candidate_transformation) = get<2>(candidate_sub_areas[i]);
                geometry_msgs::msg::Point32 candidate_position = create_point(
                    get<1>(candidate_transformation).transform.translation.x,
                    get<1>(candidate_transformation).transform.translation.y,
                    get<1>(candidate_transformation).transform.translation.z);

                if (((early_termination_mode && !use_traversability_sampling_) || calc_min_dist_to_obs(map_data_, obs_dist_table_global_, candidate_position) >= min_dist_to_obstacle_) &&
                    is_point_in_map(candidate_position, map_data_) &&
                    !is_point_unknow(candidate_position, map_data_, -1, -1))
                {
#pragma omp critical
                    candidate_transformations.emplace_back(candidate_transformation);
                }
            }

            tim_port::checkpoint();
            if (candidate_transformations.empty())
            {
                RCLCPP_WARN(rclcpp::get_logger("tim_core"), "Batch %d: No candidate transformations.", batch_idx);

                start_idx = end_idx;

                continue;
            }

            RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Batch %d: Sorting result candidates...", batch_idx);

            sort(candidate_transformations.begin(), candidate_transformations.end(),
                 [](const auto &a, const auto &b)
                 {
                     return get<0>(a) > get<0>(b);
                 });

            RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Batch %d: Best estimate in this batch: (%.3f, %.3f, %.3f), score: %.4f.",
                     batch_idx,
                     get<1>(candidate_transformations[0]).transform.translation.x,
                     get<1>(candidate_transformations[0]).transform.translation.y,
                     tf2::getYaw(get<1>(candidate_transformations[0]).transform.rotation),
                     get<0>(candidate_transformations[0]));

            if (early_termination_mode)
            {
                if (get<0>(candidate_transformations[0]) >= fit_score_threshold_max_)
                {
                    RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Batch %d: Confidence reached threshold, ending all batch.", batch_idx);

                    loc_conf_best = get<0>(candidate_transformations[0]);
                    trans_best = get<1>(candidate_transformations[0]);

                    found_valid = true;
                    early_terminated = true;

                    matching_end_time = tim_port::WallTime::now();

                    break;
                }
                else if (get<0>(candidate_transformations[0]) >= fit_score_threshold_min_)
                {
                    if (get<0>(candidate_transformations[0]) >= loc_conf_best)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Batch %d: Result score %.4f exceeds previous best %.4f, updating final result.",
                                 batch_idx,
                                 get<0>(candidate_transformations[0]), loc_conf_best);

                        loc_conf_best = get<0>(candidate_transformations[0]);
                        trans_best = get<1>(candidate_transformations[0]);
                    }
                    else
                    {
                        RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Batch %d: Result score %.4f will not be used to update final result.",
                                 batch_idx,
                                 get<0>(candidate_transformations[0]));
                    }

                    found_valid = true;
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger("tim_core"), "Batch %d: Result score %.4f is below threshold %.4f, batch discarded.",
                             batch_idx,
                             get<0>(candidate_transformations[0]), fit_score_threshold_min_);

                    if (get<0>(candidate_transformations[0]) >= loc_conf_best)
                    {
                        loc_conf_best = get<0>(candidate_transformations[0]);
                        trans_best = get<1>(candidate_transformations[0]);
                    }
                }
            }
            else
            {
                for (const auto &ct : candidate_transformations)
                {
                    float s = get<0>(ct);

                    if (s < fit_score_threshold_min_)
                    {
                        break;
                    }

                    all_candidates.emplace_back(s, get<1>(ct));
                }
            }

            start_idx = end_idx;
        }

        if (!early_terminated)
        {
            matching_end_time = tim_port::WallTime::now();
        }

        RCLCPP_INFO(rclcpp::get_logger("tim_core"), "Sampling time: %.3lfs, matching time: %.3lfs.",
                 (sampling_end_time - sampling_start_time).toSec(),
                 (matching_end_time - sampling_end_time).toSec());

        if (!early_termination_mode)
        {
            if (all_candidates.empty())
            {
                RCLCPP_WARN(rclcpp::get_logger("tim_core"), "No candidates >= fit_score_threshold_min_=%.4f.", fit_score_threshold_min_);

                get<0>(ret) = false;

                return ret;
            }

            sort(all_candidates.begin(), all_candidates.end(),
                 [](const auto &a, const auto &b)
                 {
                     return get<0>(a) > get<0>(b);
                 });

            int num_of_candidates = 0;
            geometry_msgs::msg::PoseArray candidate_poses;
            candidate_poses.header.frame_id = map_data_.header.frame_id;
            candidate_poses.header.stamp = scan_data_.header.stamp;

            for (int i = 0; i < all_candidates.size(); i++)
            {
                bool near_found = false;

                for (int j = 0; j < get<2>(ret).size(); j++)
                {
                    geometry_msgs::msg::Point32 p0 = create_point(
                        (float)get<1>(all_candidates[i]).transform.translation.x,
                        (float)get<1>(all_candidates[i]).transform.translation.y,
                        (float)get<1>(all_candidates[i]).transform.translation.z);
                    geometry_msgs::msg::Point32 p1 = create_point(
                        (float)get<2>(ret)[j].transform.translation.x,
                        (float)get<2>(ret)[j].transform.translation.y,
                        (float)get<2>(ret)[j].transform.translation.z);
                    float yaw0 = tf2::getYaw(get<1>(all_candidates[i]).transform.rotation);
                    float yaw1 = tf2::getYaw(get<2>(ret)[j].transform.rotation);

                    if (calculate_distance(p0, p1) <= tim_port::nms_distance &&
                        abs(angles::shortest_angular_distance(yaw0, yaw1)) <= tim_port::nms_angle)
                    {
                        near_found = true;

                        break;
                    }
                }

                if (near_found)
                {
                    continue;
                }

                get<1>(ret).emplace_back(get<0>(all_candidates[i]));
                get<2>(ret).emplace_back(get<1>(all_candidates[i]));

                geometry_msgs::msg::Pose candidate_pose;
                candidate_pose.position.x = get<1>(all_candidates[i]).transform.translation.x;
                candidate_pose.position.y = get<1>(all_candidates[i]).transform.translation.y;
                candidate_pose.position.z = get<1>(all_candidates[i]).transform.translation.z;
                candidate_pose.orientation = get<1>(all_candidates[i]).transform.rotation;
                candidate_poses.poses.emplace_back(candidate_pose);

                num_of_candidates++;

                if (num_of_candidates >= max_num_of_candidates)
                {
                    break;
                }
            }

            get<0>(ret) = (get<1>(ret).size() > 0);

    

            RCLCPP_INFO(rclcpp::get_logger("tim_core"), "\033[32mReturning %ld candidates. Top-1 score: %.4f.\033[0m",
                     get<1>(ret).size(), get<1>(ret)[0]);

            return ret;
        }

        if (found_valid)
        {
            get<0>(ret) = true;
            get<1>(ret).emplace_back(loc_conf_best);
            get<2>(ret).emplace_back(trans_best);

            RCLCPP_INFO(rclcpp::get_logger("tim_core"), "\033[32mFinal best estimate: (%.3f, %.3f, %.3f), score: %.4f.\033[0m",
                     trans_best.transform.translation.x,
                     trans_best.transform.translation.y,
                     tf2::getYaw(trans_best.transform.rotation),
                     loc_conf_best);
        }
        else
        {
            get<0>(ret) = false;

            get<1>(ret).emplace_back(loc_conf_best);
            get<2>(ret).emplace_back(trans_best);

            RCLCPP_ERROR(rclcpp::get_logger("tim_core"), "All batch results are lower than score threshold. Final score: %.4f, minimum required: %.4f. Relocalization failed.",
                      loc_conf_best, fit_score_threshold_min_);
        }

        return ret;
    }
};
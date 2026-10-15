/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <relocalization/utils/relocalization_2d/sampling_2d.h>

namespace sampling_2d
{
    sampling_2d::sampling_2d(const float &min_expand_dist, const float &max_expand_dist, const float &min_dist_to_obs,
                             const float &points_min_dist, const bool &use_traversability_sampling,
                             const int &free_threshold, const int &occupied_threshold)
    {
        random_device_ = make_unique<random_device>();
        random_generator_ = make_unique<mt19937>(tim_port::random_seed);
        gen_random_number_ = make_unique<uniform_real_distribution<float>>(0.0f, 1.0f);

        free_threshold_ = free_threshold;
        occupied_threshold_ = occupied_threshold;

        min_expand_dist_ = min_expand_dist;
        max_expand_dist_ = max_expand_dist;
        min_dist_to_obs_ = min_dist_to_obs;
        points_min_dist_ = points_min_dist;

        use_traversability_sampling_ = use_traversability_sampling;
    }

    void sampling_2d::initialize(const nav_msgs::msg::OccupancyGrid &map_data, const vector<float> &min_dist_to_obs_table)
    {
        map_data_ = map_data;
        min_dist_to_obs_table_ = min_dist_to_obs_table;
        const int width = map_data_.info.width, height = map_data_.info.height;
        current_map_x_ = width * map_data_.info.resolution;
        current_map_y_ = height * map_data_.info.resolution;
        start_x_ = map_data_.info.origin.position.x + current_map_x_ / 2;
        start_y_ = map_data_.info.origin.position.y + current_map_y_ / 2;
        rrt_tree_ = rrt_nodes{};
        rtree_.clear();
        rtree_points_.clear();
        // ROS 2 adaptation: seed an RRT forest in every connected feasible region.
        // The upstream's fixed (0,0) root excludes disconnected/translated maps.
        std::vector<bool> visited(map_data_.data.size(), false);
        auto feasible = [&](int i) {
            return map_data_.data[i] >= 0 && map_data_.data[i] <= free_threshold_ &&
                min_dist_to_obs_table_[i] >= min_dist_to_obs_;
        };
        std::vector<int> queue;
        for (int seed=0; seed < width*height; ++seed) {
            if ((seed & 1023) == 0) tim_port::checkpoint();
            if (visited[seed] || !feasible(seed)) continue;
            const auto point = grid_index_to_position(seed, map_data_);
            const int index = static_cast<int>(rrt_tree_.points.size());
            rrt_tree_.indices.push_back(index);
            rrt_tree_.points.push_back(point);
            rrt_tree_.prev_indices.push_back(index);
            rrt_tree_.prev_points.push_back(point);
            rrt_tree_.next_indices.emplace_back();
            rrt_tree_.next_points.emplace_back();
            rtree_points_.emplace_back(point.x, point.y);
            rtree_.insert({rtree_points_.back(), static_cast<size_t>(index)});
            queue.clear(); queue.push_back(seed); visited[seed]=true;
            for (size_t k=0; k<queue.size(); ++k) {
                if ((k & 1023) == 0) tim_port::checkpoint();
                const int i=queue[k], x=i%width, y=i/width;
                const int next[4]={x>0?i-1:-1, x+1<width?i+1:-1, y>0?i-width:-1, y+1<height?i+width:-1};
                for (int j:next) if (j>=0 && !visited[j] && feasible(j)) {
                    visited[j]=true; queue.push_back(j);
                }
            }
        }
        node_index_ = static_cast<int>(rrt_tree_.points.size())-1;
        origin_ = rrt_tree_.points.empty() ? geometry_msgs::msg::Point32{} : rrt_tree_.points.front();
    }

    vector<geometry_msgs::msg::Point32> sampling_2d::get_points(const int &timeout_secs)
    {
        if (rrt_tree_.points.empty()) return {};
        RCLCPP_INFO_STREAM(rclcpp::get_logger("tim_core"), "Starting " << (use_traversability_sampling_ ? "RRT sampling" : "random sampling") << ".");

        auto start_time = tim_port::WallTime::now();

        vector<geometry_msgs::msg::Point32> current_downsampled_points;
        vector<geometry_msgs::msg::Point32> last_downsampled_points;
        size_t point_num_per_batch = static_cast<int>(((float)(map_data_.info.width * map_data_.info.height * pow(map_data_.info.resolution, 2)) /
                                                       (float)pow(points_min_dist_, 2)));
        point_num_per_batch = point_num_per_batch < 2000 ? 2000 : point_num_per_batch;

        if (use_traversability_sampling_)
        {
            while ((tim_port::WallTime::now() - start_time).toSec() <= timeout_secs)
            {
                for (int i = 0; i < point_num_per_batch; i++)
                {
                    if ((i & 127) == 0) tim_port::checkpoint();
                    geometry_msgs::msg::Point32 sample_point;
                    sample_point.x = (generate_random_number() * current_map_x_) - (current_map_x_ / 2.0) + start_x_;
                    sample_point.y = (generate_random_number() * current_map_y_) - (current_map_y_ / 2.0) + start_y_;
                    sample_point.z = 0;

                    tuple<int, geometry_msgs::msg::Point32> nearest_point;

#if USE_R_TREE
                    BGPoint query_pt(sample_point.x, sample_point.y);
                    vector<RTreeValue> result;
                    rtree_.query(bgi::nearest(query_pt, 1), back_inserter(result));
                    get<0>(nearest_point) = result.front().second;
                    get<1>(nearest_point) = rrt_tree_.points[get<0>(nearest_point)];
#else
                    nearest_point = get_nearest_point(sample_point, rrt_tree_.points, rrt_tree_.indices);
#endif

                    if (calculate_distance(get<1>(nearest_point), origin_) < 2 * min_dist_to_obs_ ||
                        !is_point_unknow(get<1>(nearest_point), map_data_, -1, -1))
                    {
                        float expand_dist = calc_decayed_distance(get<1>(nearest_point),
                                                                  map_data_, min_dist_to_obs_table_, min_dist_to_obs_,
                                                                  min_expand_dist_, max_expand_dist_);

                        geometry_msgs::msg::Point32 new_point = generate_point(get<1>(nearest_point), sample_point, expand_dist);

                        if (is_point_in_map(new_point, map_data_))
                        {
                            auto check_result = exploration_check_segment_type(get<1>(nearest_point), new_point, 3,
                                                                               min_dist_to_obs_, map_data_, min_dist_to_obs_table_, true,
                                                                               free_threshold_, occupied_threshold_);

                            if (check_result == exploration_segment_type::NO_COLLISION)
                            {
                                rrt_tree_.indices.push_back(++node_index_);
                                rrt_tree_.points.push_back(new_point);
                                rrt_tree_.prev_indices.push_back(get<0>(nearest_point));
                                rrt_tree_.prev_points.push_back(get<1>(nearest_point));
                                rrt_tree_.next_indices.emplace_back();
                                rrt_tree_.next_points.emplace_back();
                                rrt_tree_.next_indices[get<0>(nearest_point)].push_back(node_index_);
                                rrt_tree_.next_points[get<0>(nearest_point)].push_back(new_point);

#if USE_R_TREE
                                rtree_points_.emplace_back(new_point.x, new_point.y);
                                rtree_.insert(make_pair(rtree_points_.back(), node_index_));
#endif
                            }
                        }
                    }
                }

                current_downsampled_points = resample_points(rrt_tree_.points, points_min_dist_);

                if (current_downsampled_points.size() - last_downsampled_points.size() <= 2)
                {
                    break;
                }

                last_downsampled_points = current_downsampled_points;
            }

            // Keep the upstream RRT expansion, then fill coverage gaps deterministically.
            // This ensures every feasible cell (including tiny disconnected regions)
            // lies within points_min_dist_ of an anchor; it is not a pose prior.
            std::vector<geometry_msgs::msg::Point32> anchors;
            bgi::rtree<RTreeValue, bgi::quadratic<16>> anchor_tree;
            auto add_if_uncovered = [&](const geometry_msgs::msg::Point32 &point) {
                std::vector<RTreeValue> nearest;
                anchor_tree.query(bgi::nearest(BGPoint(point.x,point.y),1), std::back_inserter(nearest));
                if (!nearest.empty() && calculate_distance(point, anchors[nearest.front().second]) < points_min_dist_) return;
                anchor_tree.insert({BGPoint(point.x,point.y), anchors.size()});
                anchors.push_back(point);
            };
            for (const auto &point:rrt_tree_.points) add_if_uncovered(point);
            for (size_t i=0; i<map_data_.data.size(); ++i) {
                if ((i & 255) == 0) tim_port::checkpoint();
                if (map_data_.data[i]>=0 && map_data_.data[i]<=free_threshold_ && min_dist_to_obs_table_[i]>=min_dist_to_obs_)
                    add_if_uncovered(grid_index_to_position(static_cast<int>(i),map_data_));
            }
            return anchors;
        }
        else
        {
            vector<geometry_msgs::msg::Point32> points;

            while ((tim_port::WallTime::now() - start_time).toSec() <= timeout_secs)
            {
                for (int i = 0; i < point_num_per_batch; i++)
                {
                    if ((i & 127) == 0) tim_port::checkpoint();
                    geometry_msgs::msg::Point32 sample_point;
                    sample_point.x = (generate_random_number() * current_map_x_) - (current_map_x_ / 2.0) + start_x_;
                    sample_point.y = (generate_random_number() * current_map_y_) - (current_map_y_ / 2.0) + start_y_;
                    sample_point.z = 0;

                    if (is_point_in_map(sample_point, map_data_) &&
                        map_data_.data[position_to_grid_index(sample_point, map_data_)] >= 0 &&
                        map_data_.data[position_to_grid_index(sample_point, map_data_)] <= free_threshold_ &&
                        min_dist_to_obs_table_[position_to_grid_index(sample_point, map_data_)] >= min_dist_to_obs_)
                    {
                        points.push_back(sample_point);
                    }
                }

                current_downsampled_points = resample_points(points, points_min_dist_);

                if (current_downsampled_points.size() - last_downsampled_points.size() <= 2)
                {
                    break;
                }

                last_downsampled_points = current_downsampled_points;
            }

            return resample_points(points, points_min_dist_);
        }
    }

    float sampling_2d::generate_random_number()
    {
        return (*gen_random_number_)(*random_generator_);
    }
}
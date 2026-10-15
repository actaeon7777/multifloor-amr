/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <relocalization/utils/relocalization_2d/likelihood_field_2d.h>

namespace likelihood_field_2d
{
    likelihood_field_2d::likelihood_field_2d(const float &sigma_hit, const float &z_hit, const float &z_rand,
                                             const float &keep_ratio_1_min, const float &keep_ratio_1_max, const float &keep_ratio_2_min, const float &keep_ratio_2_max,
                                             const int &map_occupied_threshold)
        : sigma_hit_(sigma_hit), z_hit_(z_hit), z_rand_(z_rand),
          keep_ratio_1_min_(keep_ratio_1_min), keep_ratio_1_max_(keep_ratio_1_max), keep_ratio_2_min_(keep_ratio_2_min), keep_ratio_2_max_(keep_ratio_2_max),
          map_occupied_threshold_(map_occupied_threshold)
    {

        return;
    };

    float likelihood_field_2d::gaussian_prob(float dist)
    {
        return z_hit_ * exp(-(dist * dist) / (2.0f * sigma_hit_ * sigma_hit_)) + z_rand_;
    }

    tuple<float, float, float> likelihood_field_2d::get_likelihood_field_score(
        const geometry_msgs::msg::TransformStamped &tf_map_lidar,
        const sensor_msgs::msg::LaserScan &scan,
        const nav_msgs::msg::OccupancyGrid &map,
        const vector<float> &obs_dist_table,
        const int &lidar_sampling_step)
    {
        vector<pair<float, float>> loglik_dist_pairs;

        float angle = scan.angle_min;
        float total_dist = 0;
        int valid_beam_count = 0;

        float delta = scan.angle_increment * lidar_sampling_step;
        float c = cos(scan.angle_min);
        float s = sin(scan.angle_min);
        float cd = cos(delta);
        float sd = sin(delta);

        for (size_t i = 0; i < scan.ranges.size(); i += lidar_sampling_step, angle += scan.angle_increment * lidar_sampling_step)
        {
            if (std::isfinite(scan.ranges[i]) && scan.ranges[i] >= scan.range_min && scan.ranges[i] < scan.range_max)
            {
                geometry_msgs::msg::PointStamped p_lidar, p_map;
                p_lidar.header.frame_id = scan.header.frame_id;
                p_lidar.point.x = scan.ranges[i] * c;
                p_lidar.point.y = scan.ranges[i] * s;
                tf2::doTransform(p_lidar, p_map, tf_map_lidar);

                float dist = calc_min_dist_to_obs(map, obs_dist_table,
                                                  create_point(p_map.point.x, p_map.point.y, p_map.point.z));
                float log_p = log(gaussian_prob(dist) + 1e-6f);
                loglik_dist_pairs.emplace_back(log_p, dist);
                total_dist += dist;

                valid_beam_count++;
            }

            float c_new = c * cd - s * sd;
            float s_new = s * cd + c * sd;
            c = c_new;
            s = s_new;
        }

        if (valid_beam_count < 5)
        {
            return {-numeric_limits<float>::infinity(), numeric_limits<float>::infinity(), -numeric_limits<float>::infinity()};
        }

        sort(loglik_dist_pairs.begin(), loglik_dist_pairs.end(),
             [](const pair<float, float> &a, const pair<float, float> &b)
             {
                 return a.first > b.first;
             });

        int n = loglik_dist_pairs.size();

        float scan_fov = abs(scan.angle_max - scan.angle_min);
        float alpha = clamp((float)((scan_fov - M_PI) / M_PI), 0.0f, 1.0f);
        float keep_ratio_1 = (1.0f - alpha) * keep_ratio_1_max_ + alpha * keep_ratio_1_min_;
        float keep_ratio_2 = (1.0f - alpha) * keep_ratio_2_max_ + alpha * keep_ratio_2_min_;

        int k100 = n;
        int k1 = min(n, max(5, static_cast<int>(keep_ratio_1 * n)));
        int k2 = min(n, max(5, static_cast<int>(keep_ratio_2 * n)));

        auto avg_score = [&](int k)
        {
            float sum = 0.0f;

            for (int i = 0; i < k; i++)
            {
                sum += loglik_dist_pairs[i].first;
            }

            return sum / k;
        };

        vector<pair<float, int>> score_candidates = {
            {avg_score(k100), k100},
            {avg_score(k1), k1},
            {avg_score(k2), k2}};

        sort(score_candidates.begin(), score_candidates.end(), greater<>());
        float avg_log_likelihood = score_candidates[1].first;
        int selected_k = score_candidates[1].second;

        float sum_sq = 0.0f;
        float selected_sum = 0.0f;

        for (int i = 0; i < selected_k; i++)
        {
            float d = loglik_dist_pairs[i].second;
            sum_sq += d * d;
            selected_sum += d;
        }

        float mean_dist = total_dist / valid_beam_count;
        float selected_mean = selected_sum / selected_k;
        float variance = (sum_sq / selected_k) - (selected_mean * selected_mean);
        variance = max(0.0f, variance);
        float consistency_score = exp(-variance / 2.0f);

        return {avg_log_likelihood, mean_dist, consistency_score};
    }

    tuple<float, float, float> likelihood_field_2d::get_likelihood_field_score(const geometry_msgs::msg::TransformStamped &tf_map_lidar,
                                                                               const sensor_msgs::msg::LaserScan &scan,
                                                                               const nav_msgs::msg::OccupancyGrid &map,
                                                                               const int &lidar_sampling_step)
    {
        float max_range = scan.range_max;
        float radius = max_range + 2.0f;
        vector<float> distance_map = calc_min_dist_to_obs_table(map,
                                                                tf_map_lidar.transform.translation.x,
                                                                tf_map_lidar.transform.translation.y,
                                                                radius, map_occupied_threshold_);

        return get_likelihood_field_score(tf_map_lidar, scan, map, distance_map,
                                          lidar_sampling_step);
    }

    float likelihood_field_2d::get_confidence(const geometry_msgs::msg::TransformStamped &tf_map_lidar,
                                              const sensor_msgs::msg::LaserScan &scan,
                                              const nav_msgs::msg::OccupancyGrid &map,
                                              const vector<float> &obs_dist_table,
                                              const float &max_tolerance_dist,
                                              const int &lidar_sampling_step,
                                              const bool &geometric_mean_fusion)
    {
        auto lf_score = get_likelihood_field_score(
            tf_map_lidar, scan, map, obs_dist_table, lidar_sampling_step);
        const float likelihood_field_score = get<0>(lf_score);
        const float mean_min_dist = get<1>(lf_score);
        const float consistency_score = get<2>(lf_score);

        if (!std::isfinite(likelihood_field_score) || !std::isfinite(mean_min_dist) || !std::isfinite(consistency_score)) return 0.0f;
        float p = exp(likelihood_field_score);

        float resolution = map.info.resolution;
        float min_dist = resolution * 0.5f;
        float max_dist = max_tolerance_dist;

        float clamped_dist = min(max(mean_min_dist, min_dist), max_dist);
        float d_score = 1.0f - ((clamped_dist - min_dist) / (max_dist - min_dist));

        float confidence = geometric_mean_fusion ? (pow(p * d_score * consistency_score, 1.0f / 3.0f)) : ((abs(p) + abs(d_score) + abs(consistency_score)) / 3.0f);

        return std::isfinite(confidence) ? std::clamp(confidence, 0.0f, 1.0f) : 0.0f;
    }
};
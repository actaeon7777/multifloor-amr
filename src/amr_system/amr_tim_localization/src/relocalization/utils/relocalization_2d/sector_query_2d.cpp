/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <relocalization/utils/relocalization_2d/sector_query_2d.h>

namespace sector_query_2d
{
    sector_query_2d::sector_query_2d(const int &sector_sample_num)
        : sector_sample_num_(sector_sample_num)
    {
        tree_.resize(sector_sample_num_ + 1, 0.0f);
        data_.resize(sector_sample_num_, 0.0f);
    };

    int sector_query_2d::angle_to_index(const float &angle)
    {
        float normalized = angles::normalize_angle_positive(angle);

        return static_cast<int>(normalized / (2 * M_PI) * sector_sample_num_);
    }

    void sector_query_2d::update(int index, float value)
    {
#if USE_FENWICK_TREE
        while (index <= sector_sample_num_)
        {
            tree_[index] += value;
            index += index & (-index);
        }
#endif
    }

    float sector_query_2d::query(int index)
    {
#if USE_FENWICK_TREE
        float sum = 0.0f;

        while (index > 0)
        {
            sum += tree_[index];
            index -= index & (-index);
        }

        return sum;
#else
        float sum = 0.0f;

        for (int i = 0; i < index && i < sector_sample_num_; i++)
        {
            sum += data_[i];
        }

        return sum;
#endif
    }

    void sector_query_2d::add_range(const float &angle, const float &range)
    {
        int idx = angle_to_index(angle);

        data_[idx] += range;

#if USE_FENWICK_TREE
        update(idx + 1, range);
#endif
    }

    float sector_query_2d::get_sum(const float &angle_start, const float &angle)
    {
        float angle_end = angle_start + angle;

        int idx_start = angle_to_index(angle_start);
        int idx_end = angle_to_index(angle_end);

        if (idx_start <= idx_end)
        {
            return query(idx_end) - query(idx_start);
        }
        else
        {
            return (query(sector_sample_num_) - query(idx_start)) + query(idx_end);
        }
    }

    float sector_query_2d::get_mean(const float &angle_start, const float &angle)
    {
        float angle_end = angle_start + angle;

        int idx_start = angle_to_index(angle_start);
        int idx_end = angle_to_index(angle_end);
        int count = 0;

        if (idx_start <= idx_end)
        {
            count = idx_end - idx_start;
        }
        else
        {
            count = (sector_sample_num_ - idx_start) + idx_end;
        }

        if (count == 0)
        {
            return 0.0f;
        }

        return get_sum(angle_start, angle) / count;
    }
};
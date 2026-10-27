#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace amr_depth_filter
{
// Keeps an obstacle point only if its fixed-frame XY cell (or a neighbour)
// also held an obstacle point in each of the previous frames - 1 clouds.
// Glossy-floor outliers appear for a single frame; real obstacles stay put.
class PersistenceFilter
{
public:
  PersistenceFilter(double cell_size, int frames, int neighbor_cells)
  : cell_(cell_size), frames_(frames), neighbor_(neighbor_cells)
  {
    if (!(cell_size > 0) || frames < 1 || neighbor_cells < 0) {
      throw std::invalid_argument("cell_size > 0, frames >= 1, neighbor_cells >= 0 required");
    }
    history_.resize(static_cast<size_t>(frames_ - 1));
  }

  // xy: fixed-frame positions of this cloud's obstacle candidates.
  std::vector<bool> apply(const std::vector<std::pair<double, double>> & xy)
  {
    std::vector<bool> keep;
    apply_into(xy, keep);
    return keep;
  }

  // Caller-owned storage avoids allocating the result on every cloud.
  void apply(const std::vector<std::pair<double, double>> & xy, std::vector<uint8_t> & keep)
  {
    apply_into(xy, keep);
  }

  void reset()
  {
    history_size_ = next_history_ = 0;
    for (auto & past : history_) {past.clear();}
    current_.clear();
    decisions_.clear();
  }

private:
  template<typename Keep>
  void apply_into(const std::vector<std::pair<double, double>> & xy, Keep & keep)
  {
    keep.clear();
    keep.reserve(xy.size());
    keep.resize(xy.size(), frames_ == 1);
    if (frames_ == 1) {return;}

    current_.clear();
    decisions_.clear();
    // Only grow: reserve(smaller) is allowed to shrink a cleared hash table.
    if (xy.size() > current_.bucket_count() * current_.max_load_factor()) {
      current_.reserve(xy.size());
    }
    const bool ready = history_size_ == history_.size();
    if (ready && xy.size() > decisions_.bucket_count() * decisions_.max_load_factor()) {
      decisions_.reserve(xy.size());
    }
    for (size_t i = 0; i < xy.size(); ++i) {
      const auto [cx, cy] = cell(xy[i]);
      const auto packed = key(cx, cy);
      if (!ready) {current_.insert(packed); continue;}
      auto [decision, inserted] = decisions_.try_emplace(packed, true);
      if (inserted) {
        current_.insert(packed);
        for (const auto & past : history_) {
          if (!near(past, cx, cy)) {decision->second = false; break;}
        }
      }
      keep[i] = decision->second;
    }
    // The oldest set must remain available until all decisions are made.
    current_.swap(history_[next_history_]);
    next_history_ = (next_history_ + 1) % history_.size();
    if (!ready) {++history_size_;}
  }

  std::pair<int64_t, int64_t> cell(const std::pair<double, double> & p) const
  {
    return {static_cast<int64_t>(std::floor(p.first / cell_)),
      static_cast<int64_t>(std::floor(p.second / cell_))};
  }
  static int64_t key(int64_t x, int64_t y)
  {
    // Preserve the original low-32-bit packing, including huge/negative cells,
    // without left-shifting a negative signed integer.
    return static_cast<int64_t>((static_cast<uint64_t>(x) << 32) ^
           (static_cast<uint64_t>(y) & 0xffffffffULL));
  }
  bool near(const std::unordered_set<int64_t> & set, int64_t x, int64_t y) const
  {
    for (int64_t dx = -neighbor_; dx <= neighbor_; ++dx) {
      for (int64_t dy = -neighbor_; dy <= neighbor_; ++dy) {
        if (set.count(key(x + dx, y + dy))) {return true;}
      }
    }
    return false;
  }

  double cell_;
  int frames_, neighbor_;
  std::vector<std::unordered_set<int64_t>> history_;
  std::unordered_set<int64_t> current_;
  std::unordered_map<int64_t, bool> decisions_;
  size_t history_size_{0}, next_history_{0};
};
}  // namespace amr_depth_filter

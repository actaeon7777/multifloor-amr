#pragma once
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <unordered_set>
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
  }

  // xy: fixed-frame positions of this cloud's obstacle candidates.
  std::vector<bool> apply(const std::vector<std::pair<double, double>> & xy)
  {
    std::unordered_set<int64_t> current;
    std::vector<bool> keep(xy.size(), true);
    for (size_t i = 0; i < xy.size(); ++i) {
      const auto [cx, cy] = cell(xy[i]);
      current.insert(key(cx, cy));
      if (static_cast<int>(history_.size()) < frames_ - 1) {
        keep[i] = false;
        continue;
      }
      for (const auto & past : history_) {
        if (!near(past, cx, cy)) {keep[i] = false; break;}
      }
    }
    if (frames_ > 1) {
      history_.push_back(std::move(current));
      while (static_cast<int>(history_.size()) > frames_ - 1) {history_.pop_front();}
    }
    return keep;
  }

  void reset() {history_.clear();}

private:
  std::pair<int64_t, int64_t> cell(const std::pair<double, double> & p) const
  {
    return {static_cast<int64_t>(std::floor(p.first / cell_)),
      static_cast<int64_t>(std::floor(p.second / cell_))};
  }
  static int64_t key(int64_t x, int64_t y) {return (x << 32) ^ (y & 0xffffffff);}
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
  std::deque<std::unordered_set<int64_t>> history_;
};
}  // namespace amr_depth_filter

#include <algorithm>
#include <array>
#include <deque>
#include <random>

#include "gtest/gtest.h"
#include "amr_depth_filter/persistence.hpp"

using amr_depth_filter::PersistenceFilter;

TEST(Persistence, SingleFrameOutlierNeverPasses)
{
  PersistenceFilter filter(.05, 3, 1);
  EXPECT_FALSE(filter.apply({{2.1, .3}})[0]);
  EXPECT_TRUE(filter.apply({}).empty());
  EXPECT_FALSE(filter.apply({{5., 5.}})[0]);
  EXPECT_FALSE(filter.apply({{2.1, .3}})[0]);  // Gap in between: still rejected.
}

TEST(Persistence, StaticObstaclePassesFromThirdFrame)
{
  PersistenceFilter filter(.05, 3, 1);
  EXPECT_FALSE(filter.apply({{2.1, .3}})[0]);
  EXPECT_FALSE(filter.apply({{2.12, .31}})[0]);
  EXPECT_TRUE(filter.apply({{2.1, .33}})[0]);  // Within one neighbouring 5 cm cell.
  EXPECT_TRUE(filter.apply({{2.1, .3}})[0]);
}

TEST(Persistence, FarJumpRejectedAndNegativeCoordinatesWork)
{
  PersistenceFilter filter(.05, 2, 1);
  filter.apply({{-1.02, -0.51}});
  auto keep = filter.apply({{-1.04, -0.49}, {-1.2, -0.51}});
  EXPECT_TRUE(keep[0]);
  EXPECT_FALSE(keep[1]);
}

TEST(Persistence, OneFrameMeansPassThroughAndResetForgets)
{
  PersistenceFilter pass(.05, 1, 0);
  EXPECT_TRUE(pass.apply({{1., 1.}})[0]);
  PersistenceFilter filter(.05, 2, 0);
  filter.apply({{1., 1.}});
  filter.reset();
  EXPECT_FALSE(filter.apply({{1., 1.}})[0]);
  EXPECT_THROW(PersistenceFilter(0., 2, 1), std::invalid_argument);
}

namespace
{
// Unmodified pre-optimization algorithm (apart from the class name).
class OriginalPersistenceFilter
{
public:
  OriginalPersistenceFilter(double cell_size, int frames, int neighbor_cells)
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
}  // namespace


TEST(Persistence, RandomizedSequencesMatchOriginal)
{
  // Every parameter combination sees 256 frames, including reset and ring reuse.
  for (const double cell_size : {.05, .125, 1.}) {
    for (const int frames : {1, 2, 3, 5}) {
      for (const int neighbors : {0, 1, 2}) {
        SCOPED_TRACE(::testing::Message() << "cell=" << cell_size << " frames=" << frames
                                       << " neighbors=" << neighbors);
        PersistenceFilter filter(cell_size, frames, neighbors);
        PersistenceFilter compatibility_api(cell_size, frames, neighbors);
        OriginalPersistenceFilter oracle(cell_size, frames, neighbors);
        std::mt19937_64 rng(0x5eed + frames * 17 + neighbors);
        std::vector<uint8_t> actual(1024, 123);
        const std::array<size_t, 7> counts{0, 1, 2, 17, 63, 128, 700};
        const std::array<int64_t, 9> centers{
          0, -1, 1, -37, 89, (1LL << 32), -(1LL << 32),
          (1LL << 40) + 7, -(1LL << 40) - 7};
        size_t accepted = 0, rejected = 0;
        for (int frame = 0; frame < 256; ++frame) {
          SCOPED_TRACE(frame);
          if (frame == 97 || frame == 193 || frame == 194) {
            filter.reset();
            compatibility_api.reset();
            oracle.reset();
          }
          // Bursts of stable frames exercise acceptance; other frames vary size.
          const size_t count = frame % 32 < 8 ? 128 : counts[rng() % counts.size()];
          std::vector<std::pair<double, double>> xy;
          xy.reserve(count);
          for (size_t i = 0; i < count; ++i) {
            if (i > 0 && i % 3 != 0) {
              xy.push_back(xy.back());  // Many candidates in the same cell.
              continue;
            }
            const bool stable = i % 2 == 0;
            const int64_t cx = centers[(stable ? i : rng()) % centers.size()];
            const int64_t cy = centers[(stable ? i / 2 : rng()) % centers.size()];
            const double dx = stable ? .25 : (static_cast<int>(rng() % 7) - 3) + .75;
            const double dy = stable ? .25 : (static_cast<int>(rng() % 7) - 3) + .75;
            xy.emplace_back((static_cast<double>(cx) + dx) * cell_size,
              (static_cast<double>(cy) + dy) * cell_size);
          }
          std::shuffle(xy.begin(), xy.end(), rng);
          const auto expected = oracle.apply(xy);
          filter.apply(xy, actual);
          ASSERT_EQ(expected.size(), actual.size());
          EXPECT_EQ(expected, compatibility_api.apply(xy));
          for (size_t i = 0; i < actual.size(); ++i) {
            ASSERT_EQ(expected[i], static_cast<bool>(actual[i])) << "candidate=" << i;
            ASSERT_LE(actual[i], 1);
            actual[i] ? ++accepted : ++rejected;
          }
        }
        EXPECT_GT(accepted, 0u);
        if (frames > 1) {EXPECT_GT(rejected, 0u);}
      }
    }
  }
}

TEST(Persistence, PackedKeyWrapAndNeighborBoundariesMatchOriginal)
{
  constexpr double wrap = 4294967296.;
  const std::vector<std::vector<std::pair<double, double>>> sequence{
    {{-.25, -.25}, {wrap - .25, wrap - .25}, {wrap + .25, .25}},
    {{-.25, -.25}, {wrap - .25, -.25}, {-.25, wrap - .25}, {.25, .25}},
    {{.25, .25}, {wrap + .25, -wrap + .25}, {wrap - .25, wrap - .25}},
    {},
    {{-wrap - .25, -wrap - .25}, {-.25, -.25}, {.25, .25}}
  };
  for (const int frames : {1, 2, 3, 5}) {
    for (const int neighbors : {0, 1, 2}) {
      PersistenceFilter filter(1., frames, neighbors);
      OriginalPersistenceFilter oracle(1., frames, neighbors);
      for (int repetition = 0; repetition < 50; ++repetition) {
        if (repetition == 23) {filter.reset(); oracle.reset();}
        for (const auto & xy : sequence) {
          EXPECT_EQ(oracle.apply(xy), filter.apply(xy));
        }
      }
    }
  }
}

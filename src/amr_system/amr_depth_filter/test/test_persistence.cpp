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

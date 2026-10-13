#include <gtest/gtest.h>

#include "focus_region.hpp"

namespace {

using focus_region::fomRegion;
using focus_region::Region;

void expectRect(const focus_region::Rect& r, uint32_t x, uint32_t y,
                uint32_t w, uint32_t h) {
  EXPECT_EQ(r.x, x);
  EXPECT_EQ(r.y, y);
  EXPECT_EQ(r.width, w);
  EXPECT_EQ(r.height, h);
}

TEST(FocusRegionTest, PispGridAveragesTheMiddleHalfOnBothAxes) {
  // 8 x 8 grid: rows [8/3, 8 - 8/3) = [2, 6), cols [8/4, 8 - 8/4) = [2, 6).
  const Region r = fomRegion(4608, 2592, 4608, 2592);
  EXPECT_EQ(r.grid_size, 8u);
  EXPECT_EQ(r.first_col, 2u);
  EXPECT_EQ(r.num_cols, 4u);
  EXPECT_EQ(r.first_row, 2u);
  EXPECT_EQ(r.num_rows, 4u);
  // 4608 / 8 = 576, 2592 / 8 = 324: cells divide the frame exactly.
  expectRect(r.sensor, 1152, 648, 2304, 1296);
  expectRect(r.frame, 1152, 648, 2304, 1296);
  EXPECT_EQ(r.frame_width, 4608u);
  EXPECT_EQ(r.frame_height, 2592u);
}

TEST(FocusRegionTest, CellSizeTruncatesToEven) {
  // libpisp finalise_cdaf: (2328 / 8) & ~1 = 290, (1748 / 8) & ~1 = 218 —
  // the grid falls a few pixels short of the right/bottom edge, so the block
  // sits slightly left of and above the exact centre.
  const Region r = fomRegion(2328, 1748, 2328, 1748);
  expectRect(r.sensor, 580, 436, 1160, 872);
  expectRect(r.frame, 580, 436, 1160, 872);

  // (4656 / 8) & ~1 = 582, (3496 / 8) & ~1 = 436.
  const Region full = fomRegion(4656, 3496, 4656, 3496);
  expectRect(full.sensor, 1164, 872, 2328, 1744);
}

TEST(FocusRegionTest, CellSizeHasAFloorOfTwo) {
  const Region r = fomRegion(8, 8, 8, 8);
  expectRect(r.sensor, 4, 4, 8, 8);
}

TEST(FocusRegionTest, FrameRectScalesWhenOutputDiffersFromMode) {
  // Sensor-space block of 4656x3496 mapped onto a half-size output.
  const Region r = fomRegion(4656, 3496, 2328, 1748);
  expectRect(r.sensor, 1164, 872, 2328, 1744);
  expectRect(r.frame, 582, 436, 1164, 872);
  EXPECT_EQ(r.frame_width, 2328u);
  EXPECT_EQ(r.frame_height, 1748u);
}

TEST(FocusRegionTest, OtherGridSizesFollowTheIpaFormula) {
  // A 12 x 12 grid shows the general rule: the middle third of the rows
  // ([4, 8)) and the middle half of the columns ([3, 9)).
  const Region r = fomRegion(1200, 1200, 1200, 1200, 12);
  EXPECT_EQ(r.first_row, 4u);
  EXPECT_EQ(r.num_rows, 4u);
  EXPECT_EQ(r.first_col, 3u);
  EXPECT_EQ(r.num_cols, 6u);
  expectRect(r.sensor, 300, 400, 600, 400);
}

}  // namespace

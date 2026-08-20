// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <memory>

#include "grid_map_core/GridMap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "s57_layer.h"

// Test subclass exposing protected get_cost_from_grid, tide_offset_ and
// depth_costs_ for direct unit testing of the cost-mapping logic without
// going through the full updateBounds/updateCosts pipeline.
class S57LayerForTest : public s57_layer::S57Layer
{
public:
  using s57_layer::S57Layer::S57Layer;
  unsigned char testGetCost(grid_map::GridMap & grid,
                            const grid_map::Index & index)
  {
    return get_cost_from_grid(grid, index);
  }
  void setTideOffset(double v) { tide_offset_ = v; }
  void setDepthCosts(bool v) { depth_costs_ = v; }
};

class DepthCostsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

// Helper: build a 1x1 grid_map with all S57 channels (including "hazard")
// initialized to NaN.  Caller fills in only the channel(s) under test.
static grid_map::GridMap makeS57Grid()
{
  grid_map::GridMap grid({"elevation", "overhead", "restricted",
                          "unsurveyed", "caution", "hazard"});
  grid.setGeometry(grid_map::Length(1.0, 1.0), 1.0,
                   grid_map::Position(0.0, 0.0));
  for (const auto & name : grid.getLayers()) {
    grid[name].setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  return grid;
}

// Helper: same grid but WITHOUT the "hazard" layer, as produced by an
// s57_grids build predating the channel (mixed-version deployment).
static grid_map::GridMap makeLegacyS57Grid()
{
  grid_map::GridMap grid({"elevation", "overhead", "restricted",
                          "unsurveyed", "caution"});
  grid.setGeometry(grid_map::Length(1.0, 1.0), 1.0,
                   grid_map::Position(0.0, 0.0));
  for (const auto & name : grid.getLayers()) {
    grid[name].setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  return grid;
}

// (a) Default-mode regression: with depth_costs left at its default (true),
// a submerged cell continues to use the depth ramp exactly as before.
TEST_F(DepthCostsTest, DefaultModeDepthRampUnchanged)
{
  S57LayerForTest layer;
  auto grid = makeS57Grid();
  grid_map::Index idx(0, 0);

  // 5 m deep at chart datum; default minimum_depth_=0, maximum_caution_depth_=5.
  grid.at("elevation", idx) = -5.0;
  layer.setTideOffset(0.0);
  EXPECT_EQ(layer.testGetCost(grid, idx), nav2_costmap_2d::FREE_SPACE)
    << "Default mode: 5m-deep cell at zero tide must be FREE_SPACE.";

  // Shallow cell inside the caution band gets a scaled non-lethal cost.
  grid.at("elevation", idx) = -2.5;
  auto cost = layer.testGetCost(grid, idx);
  EXPECT_GT(cost, nav2_costmap_2d::FREE_SPACE);
  EXPECT_LT(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

// (b) Suppressed mode: a submerged cell asserts NO depth cost, but it MUST still
// claim the cell with a real value (FREE_SPACE) rather than NO_INFORMATION.
//
// This previously asserted NO_INFORMATION, which codified the multi-scale
// precedence bug fixed on 2026-08-05: generateTile composites overlapping
// charts finest-first and writes only into cells still NO_INFORMATION, so an
// unclaimed water cell was handed to the next-coarser chart. On an overview
// cell a harbour basin sits inside the LNDARE polygon (elevation > 0), so whole
// navigable basins were painted LETHAL at Lewes, DE — including the cell the
// vehicle was floating in. Depth authority still belongs to bathymetry_layer,
// which runs later and max-combines on top; FREE_SPACE asserts "this is water,
// not land", not "this is safe depth".
TEST_F(DepthCostsTest, SuppressedSubmergedCellClaimsCellAsFreeSpace)
{
  S57LayerForTest layer;
  layer.setDepthCosts(false);
  auto grid = makeS57Grid();
  grid_map::Index idx(0, 0);

  for (double elevation : {-0.5, -5.0, -50.0}) {
    grid.at("elevation", idx) = elevation;
    EXPECT_EQ(layer.testGetCost(grid, idx), nav2_costmap_2d::FREE_SPACE)
      << "Suppressed mode: submerged cell (elevation=" << elevation
      << ") must claim the cell as FREE_SPACE so a coarser chart cannot "
         "overwrite it with land.";
  }

  // elevation == 0.0 (DEPARE band with DRVAL1=0, e.g. the Broadkill 0-1.8 m
  // band) is submerged, not land — also claimed, depth deferred to
  // bathymetry_layer.
  grid.at("elevation", idx) = 0.0;
  EXPECT_EQ(layer.testGetCost(grid, idx), nav2_costmap_2d::FREE_SPACE);
}

// Suppressed mode: a cell with NO elevation data at all is genuinely
// uninformative and must stay NO_INFORMATION, so a coarser chart that *does*
// cover it can still contribute. This is the boundary that keeps the fix above
// from blinding the compositor to legitimate coarse-chart data.
TEST_F(DepthCostsTest, SuppressedAbsentElevationStaysNoInformation)
{
  S57LayerForTest layer;
  layer.setDepthCosts(false);
  auto grid = makeS57Grid();   // every channel NaN
  grid_map::Index idx(0, 0);

  EXPECT_EQ(layer.testGetCost(grid, idx), nav2_costmap_2d::NO_INFORMATION)
    << "Suppressed mode: a cell with no elevation data must remain unclaimed.";
}

// Regression guard for the Lewes 2026-08-05 failure, at the level the bug
// actually bit: fine-chart water must outrank coarse-chart land under
// generateTile's first-writer-wins compositing. Emulates that rule directly —
// the fine chart is consulted first, and only an unclaimed (NO_INFORMATION)
// cell falls through to the coarse chart.
TEST_F(DepthCostsTest, SuppressedFineWaterOutranksCoarseLand)
{
  S57LayerForTest layer;
  layer.setDepthCosts(false);
  grid_map::Index idx(0, 0);

  // Fine harbour chart: 6 m of charted water.
  auto fine = makeS57Grid();
  fine.at("elevation", idx) = -6.0;

  // Coarse overview chart covering the same spot: unresolved basin inside the
  // land polygon.
  auto coarse = makeS57Grid();
  coarse.at("elevation", idx) = 1.0;

  const unsigned char fine_cost = layer.testGetCost(fine, idx);
  const unsigned char coarse_cost = layer.testGetCost(coarse, idx);

  ASSERT_EQ(coarse_cost, nav2_costmap_2d::LETHAL_OBSTACLE)
    << "Precondition: the coarse overview chart does read as land here.";

  // First-writer-wins: the coarse chart may only write if the fine chart left
  // the cell unclaimed.
  const unsigned char composited =
    (fine_cost == nav2_costmap_2d::NO_INFORMATION) ? coarse_cost : fine_cost;

  EXPECT_NE(composited, nav2_costmap_2d::LETHAL_OBSTACLE)
    << "Charted navigable water must not be overwritten as LETHAL by a "
       "coarser-scale chart's land polygon.";
  EXPECT_EQ(composited, nav2_costmap_2d::FREE_SPACE);
}

// (c) Suppressed mode: land (elevation > 0) stays LETHAL, independent of tide.
TEST_F(DepthCostsTest, SuppressedLandStaysLethal)
{
  S57LayerForTest layer;
  layer.setDepthCosts(false);
  auto grid = makeS57Grid();
  grid_map::Index idx(0, 0);

  grid.at("elevation", idx) = 1.0;  // canonical land marker
  for (double tide : {-1.0, 0.0, 3.0, 7.0}) {
    layer.setTideOffset(tide);
    EXPECT_EQ(layer.testGetCost(grid, idx), nav2_costmap_2d::LETHAL_OBSTACLE)
      << "Suppressed mode: land cell must be LETHAL at tide_offset=" << tide;
  }
}

// (d) Suppressed mode: restricted areas stay LETHAL (checked before the
// depth branch, unchanged from default mode).
TEST_F(DepthCostsTest, SuppressedRestrictedStaysLethal)
{
  S57LayerForTest layer;
  layer.setDepthCosts(false);
  auto grid = makeS57Grid();
  grid_map::Index idx(0, 0);

  grid.at("restricted", idx) = 0.0;
  EXPECT_EQ(layer.testGetCost(grid, idx), nav2_costmap_2d::LETHAL_OBSTACLE);

  // Low overhead clearance likewise (default overhead_clearance_ = 10 m).
  auto grid2 = makeS57Grid();
  grid2.at("overhead", idx) = 5.0;
  EXPECT_EQ(layer.testGetCost(grid2, idx), nav2_costmap_2d::LETHAL_OBSTACLE);
}

// (e) Suppressed mode: a cell that is BOTH submerged (elevation <= 0) AND
// marked unsurveyed/caution returns unsurveyed_cost_ (default 100).  A
// caution mark with no elevation data stays NO_INFORMATION, matching the
// default-mode structure where caution only modifies an elevation-bearing
// cell.
TEST_F(DepthCostsTest, SuppressedCautionRequiresElevation)
{
  S57LayerForTest layer;
  layer.setDepthCosts(false);
  grid_map::Index idx(0, 0);

  auto grid = makeS57Grid();
  grid.at("elevation", idx) = -3.0;
  grid.at("caution", idx) = 0.0;
  EXPECT_EQ(layer.testGetCost(grid, idx), (unsigned char)100)
    << "Submerged + caution must cost unsurveyed_cost_.";

  auto grid2 = makeS57Grid();
  grid2.at("elevation", idx) = -3.0;
  grid2.at("unsurveyed", idx) = 0.0;
  EXPECT_EQ(layer.testGetCost(grid2, idx), (unsigned char)100)
    << "Submerged + unsurveyed must cost unsurveyed_cost_.";

  auto grid3 = makeS57Grid();
  grid3.at("caution", idx) = 0.0;  // no elevation data
  EXPECT_EQ(layer.testGetCost(grid3, idx), nav2_costmap_2d::NO_INFORMATION)
    << "Caution with no elevation data stays NO_INFORMATION.";
}

// (f) Suppressed mode: a charted point hazard (UWTROC/WRECKS/PIPSOL via the
// "hazard" channel) is LETHAL regardless of its charted depth — this is the
// plan-review must-fix: without the dedicated channel these cells would be
// indistinguishable from a DEPARE band and vanish from the costmap wherever
// bathymetry_layer has no survey coverage.
TEST_F(DepthCostsTest, SuppressedHazardStaysLethal)
{
  S57LayerForTest layer;
  layer.setDepthCosts(false);
  auto grid = makeS57Grid();
  grid_map::Index idx(0, 0);

  for (double sounding_elevation : {0.0, -1.0, -20.0}) {
    grid.at("elevation", idx) = sounding_elevation;
    grid.at("hazard", idx) = sounding_elevation;
    EXPECT_EQ(layer.testGetCost(grid, idx), nav2_costmap_2d::LETHAL_OBSTACLE)
      << "Suppressed mode: hazard cell (charted sounding elevation="
      << sounding_elevation << ") must be LETHAL.";
  }

  // Soundingless UWTROC/WRECKS rasterize hazard=0.0 with NO elevation write
  // (awash assumption; round-2 review must-fix). Hazard alone must be LETHAL.
  auto grid2 = makeS57Grid();
  grid2.at("hazard", idx) = 0.0;  // elevation stays NaN
  EXPECT_EQ(layer.testGetCost(grid2, idx), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "Suppressed mode: soundingless hazard (no elevation) must be LETHAL.";
}

// (g) Default mode: the hazard channel does not alter the depth-ramp result —
// existing behavior is byte-identical with the channel present.
TEST_F(DepthCostsTest, DefaultModeIgnoresHazardChannel)
{
  S57LayerForTest layer;
  auto grid = makeS57Grid();
  grid_map::Index idx(0, 0);

  grid.at("elevation", idx) = -5.0;
  grid.at("hazard", idx) = -5.0;
  layer.setTideOffset(0.0);
  EXPECT_EQ(layer.testGetCost(grid, idx), nav2_costmap_2d::FREE_SPACE)
    << "Default mode: deep hazard cell must still follow the depth ramp.";
}

// (h) Suppressed mode: a grid WITHOUT a hazard layer (older s57_grids build)
// must not throw — the exists() guard tolerates mixed-version deployments.
TEST_F(DepthCostsTest, SuppressedToleratesGridWithoutHazardLayer)
{
  S57LayerForTest layer;
  layer.setDepthCosts(false);
  auto grid = makeLegacyS57Grid();
  grid_map::Index idx(0, 0);

  grid.at("elevation", idx) = -5.0;
  unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE;
  EXPECT_NO_THROW(cost = layer.testGetCost(grid, idx));
  // The point of this case is the exists() guard not throwing on a legacy grid.
  // The value is submerged water, so it claims the cell as FREE_SPACE — see
  // SuppressedSubmergedCellClaimsCellAsFreeSpace for why NO_INFORMATION here
  // would re-open the coarse-chart land overwrite.
  EXPECT_EQ(cost, nav2_costmap_2d::FREE_SPACE);
}

// (i) Regression for the PIPSOL DRVAL1 null-guard (marine_charts/src/
// s57_dataset.cpp): an unset/null DRVAL1 must NOT rasterize a hazard value.
// OGR returns 0.0 for an unset field, so an unguarded read writes hazard = -0.0
// (non-NaN) — and in suppressed mode ANY non-NaN hazard cell is LETHAL (see
// s57_layer.cpp), turning a depth-less charted pipeline into an unconditional
// lethal band that bathymetry_layer cannot clear.  The dataset-level
// rasterization path (S57Dataset::getGrid) opens a GDAL dataset by file path
// and has no in-memory OGR test seam, so the guard's effect is verified here at
// the layer contract it protects: with no hazard write (hazard = NaN) the
// submerged pipeline cell defers to bathymetry_layer (NO_INFORMATION), whereas
// the buggy hazard = -0.0 would instead be a LETHAL band.
TEST_F(DepthCostsTest, SuppressedUnsetPipsolDepthIsNotLethalBand)
{
  S57LayerForTest layer;
  layer.setDepthCosts(false);
  grid_map::Index idx(0, 0);

  // Guard working: DRVAL1 unset -> no elevation/hazard write -> both NaN.  The
  // pipeline footprint is left for bathymetry_layer, not painted lethal.
  auto guarded = makeS57Grid();
  EXPECT_EQ(layer.testGetCost(guarded, idx), nav2_costmap_2d::NO_INFORMATION)
    << "Unset-DRVAL1 PIPSOL must leave no hazard write (defer to bathymetry).";

  // Bug symptom (guard removed): OGR's 0.0 default -> hazard = -0.0 (non-NaN)
  // -> an unconditional lethal band.  Asserted to document why the guard
  // matters, not to endorse the behavior.
  auto unguarded = makeS57Grid();
  unguarded.at("elevation", idx) = -0.0;
  unguarded.at("hazard", idx) = -0.0;
  EXPECT_EQ(layer.testGetCost(unguarded, idx), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "A spurious hazard=-0.0 (the unguarded bug) would be a lethal band.";
}

// Integration smoke test: full updateBounds/updateCosts cycle in suppressed
// mode with tide frames configured but NO transforms published.  The layer
// must reach isCurrent() without throwing — in suppressed mode the tide TF
// is never looked up, so its absence can neither warn nor invalidate
// (echoboats#408 operational pain).
TEST_F(DepthCostsTest, SuppressedModeNeedsNoTideTransform)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_no_tf_needed");

  node->declare_parameter("chart_layer.depth_costs", false);
  node->declare_parameter("chart_layer.chart_datum_frame", std::string("chart_datum"));
  node->declare_parameter("chart_layer.sea_surface_frame", std::string("map_tide"));

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  // Deliberately publish NO transforms.

  nav2_costmap_2d::LayeredCostmap layered_costmap("map", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto * master = layered_costmap.getCostmap();

  auto layer = std::make_shared<s57_layer::S57Layer>();
  layer->initialize(&layered_costmap, "chart_layer", tf_buffer.get(), node, nullptr);

  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  EXPECT_NO_THROW({
    layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);
    layer->updateCosts(*master, 0, 0, 10, 10);
  });
  EXPECT_TRUE(layer->isCurrent());

  // A second cycle must not invalidate either (no tide state to change).
  minx = 1e30; miny = 1e30; maxx = -1e30; maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);
  EXPECT_TRUE(layer->isCurrent());
}

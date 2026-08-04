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

// (b) Suppressed mode: a submerged cell with no caution/unsurveyed marking
// returns NO_INFORMATION — the cell is left for bathymetry_layer.
TEST_F(DepthCostsTest, SuppressedSubmergedCellIsNoInformation)
{
  S57LayerForTest layer;
  layer.setDepthCosts(false);
  auto grid = makeS57Grid();
  grid_map::Index idx(0, 0);

  for (double elevation : {-0.5, -5.0, -50.0}) {
    grid.at("elevation", idx) = elevation;
    EXPECT_EQ(layer.testGetCost(grid, idx), nav2_costmap_2d::NO_INFORMATION)
      << "Suppressed mode: submerged cell (elevation=" << elevation
      << ") must be NO_INFORMATION.";
  }

  // elevation == 0.0 (DEPARE band with DRVAL1=0, e.g. the Broadkill 0-1.8 m
  // band) is submerged, not land — must also defer to bathymetry_layer.
  grid.at("elevation", idx) = 0.0;
  EXPECT_EQ(layer.testGetCost(grid, idx), nav2_costmap_2d::NO_INFORMATION);
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
  unsigned char cost = 0;
  EXPECT_NO_THROW(cost = layer.testGetCost(grid, idx));
  EXPECT_EQ(cost, nav2_costmap_2d::NO_INFORMATION);
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

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
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "s57_layer.h"

// Test subclass exposing protected get_cost_from_grid and tide_offset_
// for direct unit testing of the elevation-cost-mapping logic without
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
};

class TideOffsetTest : public ::testing::Test
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

// Helper: publish map → chart_datum and map → map_tide transforms.
// The layer calls lookupTransform(chart_datum, map_tide) to get the
// sea surface position expressed in the chart datum frame — i.e., the
// water height above MLLW.
// chart_datum_z: Z of chart datum (MLLW) in map frame
// map_tide_z: Z of sea surface in map frame
// tide offset = map_tide_z - chart_datum_z
static void publishTideTransforms(
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  double chart_datum_z,
  double map_tide_z)
{
  static int64_t stamp_ns = 1;
  auto stamp = rclcpp::Time(stamp_ns++);

  geometry_msgs::msg::TransformStamped cd;
  cd.header.stamp = stamp;
  cd.header.frame_id = "map";
  cd.child_frame_id = "chart_datum";
  cd.transform.translation.z = chart_datum_z;
  cd.transform.rotation.w = 1.0;
  tf_buffer->setTransform(cd, "test_authority", false);

  geometry_msgs::msg::TransformStamped mt;
  mt.header.stamp = stamp;
  mt.header.frame_id = "map";
  mt.child_frame_id = "map_tide";
  mt.transform.translation.z = map_tide_z;
  mt.transform.rotation.w = 1.0;
  tf_buffer->setTransform(mt, "test_authority", false);
}

// When chart_datum_frame and sea_surface_frame are set and a TF
// transform changes, the layer should invalidate cached tiles.
TEST_F(TideOffsetTest, TideChangeInvalidatesTiles)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_tide_offset");

  node->declare_parameter("chart_layer.chart_datum_frame", std::string("chart_datum"));
  node->declare_parameter("chart_layer.sea_surface_frame", std::string("map_tide"));

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Chart datum (MLLW) at -30m in map frame, sea surface at -29m
  // Tide offset = -29 - (-30) = 1.0 m above MLLW
  publishTideTransforms(tf_buffer, -30.0, -29.0);

  nav2_costmap_2d::LayeredCostmap layered_costmap("map", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);

  auto * master = layered_costmap.getCostmap();

  auto layer = std::make_shared<s57_layer::S57Layer>();
  layer->initialize(&layered_costmap, "chart_layer", tf_buffer.get(), node, nullptr);

  // First cycle: tiles become complete (no charts, uncharted allowed)
  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);
  layer->updateCosts(*master, 0, 0, 10, 10);

  EXPECT_TRUE(layer->isCurrent());

  // Change tide: sea surface rises to -27.5m, offset becomes 2.5m
  publishTideTransforms(tf_buffer, -30.0, -27.5);

  // Second cycle: tide change should invalidate tiles
  minx = 1e30; miny = 1e30; maxx = -1e30; maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);

  // current_ should be false after tide invalidation
  EXPECT_FALSE(layer->isCurrent());

  // After updateCosts, tiles should be regenerated and current again
  layer->updateCosts(*master, 0, 0, 10, 10);
  EXPECT_TRUE(layer->isCurrent());
}

// When chart_datum_frame is empty (default), no TF lookup occurs
// and tide_offset_ stays at zero.
TEST_F(TideOffsetTest, NoChartDatumFrameSkipsTideLookup)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_no_tide");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  nav2_costmap_2d::LayeredCostmap layered_costmap("map", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);

  auto * master = layered_costmap.getCostmap();

  auto layer = std::make_shared<s57_layer::S57Layer>();
  layer->initialize(&layered_costmap, "chart_layer", tf_buffer.get(), node, nullptr);

  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);
  layer->updateCosts(*master, 0, 0, 10, 10);

  // Should work fine with no chart_datum_frame — no TF errors
  EXPECT_TRUE(layer->isCurrent());
}

// Small tide changes (below threshold) should not invalidate tiles.
TEST_F(TideOffsetTest, SmallTideChangeIgnored)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_small_tide");

  node->declare_parameter("chart_layer.chart_datum_frame", std::string("chart_datum"));
  node->declare_parameter("chart_layer.sea_surface_frame", std::string("map_tide"));

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  publishTideTransforms(tf_buffer, -30.0, -29.0);

  nav2_costmap_2d::LayeredCostmap layered_costmap("map", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);

  auto * master = layered_costmap.getCostmap();

  auto layer = std::make_shared<s57_layer::S57Layer>();
  layer->initialize(&layered_costmap, "chart_layer", tf_buffer.get(), node, nullptr);

  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);
  layer->updateCosts(*master, 0, 0, 10, 10);
  EXPECT_TRUE(layer->isCurrent());

  // Change by only 0.005m — below the 0.01m threshold
  publishTideTransforms(tf_buffer, -30.0, -28.995);

  minx = 1e30; miny = 1e30; maxx = -1e30; maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);

  // Should still be current — change too small
  EXPECT_TRUE(layer->isCurrent());
}

// A configured tide_invalidate_threshold should override the default,
// allowing larger tide changes (between default and override) to pass
// without invalidating cached tiles. Used by sim configs where the
// accelerated tide_speed_factor would otherwise fire invalidations
// every few seconds.
TEST_F(TideOffsetTest, CustomThresholdAcceptsLargerChanges)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_custom_threshold");

  node->declare_parameter("chart_layer.chart_datum_frame", std::string("chart_datum"));
  node->declare_parameter("chart_layer.sea_surface_frame", std::string("map_tide"));
  node->declare_parameter("chart_layer.tide_invalidate_threshold", 0.05);

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  publishTideTransforms(tf_buffer, -30.0, -29.0);

  nav2_costmap_2d::LayeredCostmap layered_costmap("map", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);

  auto * master = layered_costmap.getCostmap();

  auto layer = std::make_shared<s57_layer::S57Layer>();
  layer->initialize(&layered_costmap, "chart_layer", tf_buffer.get(), node, nullptr);

  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);
  layer->updateCosts(*master, 0, 0, 10, 10);
  EXPECT_TRUE(layer->isCurrent());

  // 3 cm change — above the default 0.01 m but below the configured
  // 0.05 m — must NOT invalidate.
  publishTideTransforms(tf_buffer, -30.0, -28.97);
  minx = 1e30; miny = 1e30; maxx = -1e30; maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);
  EXPECT_TRUE(layer->isCurrent());

  // 6 cm change — above the configured 0.05 m — must invalidate.
  publishTideTransforms(tf_buffer, -30.0, -28.94);
  minx = 1e30; miny = 1e30; maxx = -1e30; maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);
  EXPECT_FALSE(layer->isCurrent());

  // After updateCosts, regen completes and tiles are current again.
  layer->updateCosts(*master, 0, 0, 10, 10);
  EXPECT_TRUE(layer->isCurrent());
}

// Invalid threshold values (negative, NaN, inf) should be rejected at
// onInitialize and the default restored.
TEST_F(TideOffsetTest, InvalidThresholdFallsBackToDefault)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_invalid_threshold");

  node->declare_parameter("chart_layer.chart_datum_frame", std::string("chart_datum"));
  node->declare_parameter("chart_layer.sea_surface_frame", std::string("map_tide"));
  // Negative threshold: would otherwise make every tide change pass the
  // (std::abs(...) > threshold) test and invalidate continuously.
  node->declare_parameter("chart_layer.tide_invalidate_threshold", -1.0);

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  publishTideTransforms(tf_buffer, -30.0, -29.0);

  nav2_costmap_2d::LayeredCostmap layered_costmap("map", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);

  auto * master = layered_costmap.getCostmap();

  auto layer = std::make_shared<s57_layer::S57Layer>();
  layer->initialize(&layered_costmap, "chart_layer", tf_buffer.get(), node, nullptr);

  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);
  layer->updateCosts(*master, 0, 0, 10, 10);
  EXPECT_TRUE(layer->isCurrent());

  // 0.005 m change — below the restored default 0.01 m — must NOT
  // invalidate. If validation was skipped, the negative threshold would
  // cause invalidation here.
  publishTideTransforms(tf_buffer, -30.0, -28.995);
  minx = 1e30; miny = 1e30; maxx = -1e30; maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);
  EXPECT_TRUE(layer->isCurrent());
}

// Helper: build a 1x1 grid_map with all S57 channels initialized to NaN.
// Caller fills in only the channel(s) they want to test.
static grid_map::GridMap makeEmptyS57Grid()
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

// Land features (LNDARE, COALNE, CAUSWY, HULKES, PONTON, SLCONS, …) are
// rasterized as elevation = 1.0 with the intent "no min depth values, so
// lethal" (marine_charts/s57_dataset.cpp).  Before the elevation>0
// short-circuit, any tide_offset_ above ~minimum_depth_ + 1.0 would
// convert these cells into "navigable depth" via -elevation + tide_offset_,
// silently turning every coastline navigable at any tide above ~1 m above
// MLLW.  Verify land cells stay LETHAL across realistic tide values.
TEST_F(TideOffsetTest, LandFeatureLethalAcrossAllTides)
{
  S57LayerForTest layer;

  auto grid = makeEmptyS57Grid();
  grid_map::Index idx(0, 0);
  grid.at("elevation", idx) = 1.0;  // canonical land marker

  for (double tide : {-1.0, 0.0, 0.5, 1.0, 2.0, 3.0, 5.0, 7.0}) {
    layer.setTideOffset(tide);
    EXPECT_EQ(
      layer.testGetCost(grid, idx),
      nav2_costmap_2d::LETHAL_OBSTACLE)
      << "Land cell (elevation=1.0) must be LETHAL at tide_offset="
      << tide << " m, but was not.";
  }
}

// Genuinely-submerged cells (negative elevation) must continue to use
// the depth = -elevation + tide_offset math.  Verify the existing
// behavior is preserved by the short-circuit fix: a 5 m-deep cell at
// MLLW (elevation = -5) stays navigable through the full tidal cycle,
// and goes lethal only when minimum_depth is not satisfied.
TEST_F(TideOffsetTest, SubmergedCellUsesTideAdjustedDepth)
{
  S57LayerForTest layer;

  auto grid = makeEmptyS57Grid();
  grid_map::Index idx(0, 0);

  // 5 m deep at chart datum (DRVAL1=5 → elevation=-5)
  grid.at("elevation", idx) = -5.0;

  // Default minimum_depth_ = 0.0, maximum_caution_depth_ = 5.0.
  // depth = 5 + tide_offset.  At tide_offset = -1 m (below MLLW),
  // depth = 4 m (still inside caution band, not lethal).  At
  // tide_offset = 2 m, depth = 7 m (above caution → FREE_SPACE).
  layer.setTideOffset(-1.0);
  EXPECT_NE(layer.testGetCost(grid, idx),
            nav2_costmap_2d::LETHAL_OBSTACLE)
    << "5m-deep cell at -1m tide must not be LETHAL.";
  layer.setTideOffset(2.0);
  EXPECT_EQ(layer.testGetCost(grid, idx),
            nav2_costmap_2d::FREE_SPACE)
    << "5m-deep cell at +2m tide should be FREE_SPACE.";

  // A 1 m-deep cell drained dry at low tide must go lethal.
  grid.at("elevation", idx) = -1.0;
  layer.setTideOffset(-1.5);  // 1.5 m below MLLW
  EXPECT_EQ(layer.testGetCost(grid, idx),
            nav2_costmap_2d::LETHAL_OBSTACLE)
    << "Cell with depth = -0.5m (drained) must be LETHAL.";
}

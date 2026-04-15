// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

#include <cmath>

#include <gtest/gtest.h>
#include <memory>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "s57_layer.h"

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

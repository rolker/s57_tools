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
#include "tf2_ros/static_transform_broadcaster.h"
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

// Helper: publish a dynamic transform for chart_datum -> map with given Z.
// Uses monotonically increasing timestamps since the transform is updated
// during tests to simulate tide changes.
static void publishChartDatumTransform(
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  double z_value)
{
  static int64_t stamp_ns = 1;

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = rclcpp::Time(stamp_ns++);
  t.header.frame_id = "map";
  t.child_frame_id = "chart_datum";
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = z_value;
  t.transform.rotation.w = 1.0;
  tf_buffer->setTransform(t, "test_authority", false);
}

// When chart_datum_frame is set and a TF transform changes,
// the layer should invalidate cached tiles (current_ becomes false).
TEST_F(TideOffsetTest, TideChangeInvalidatesTiles)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_tide_offset");

  // Configure chart_datum_frame parameter
  node->declare_parameter("chart_layer.chart_datum_frame", std::string("chart_datum"));

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Publish initial transform: chart datum at Z = -1.0 (1m below global frame)
  // This means tide_offset_ = -(-1.0) = 1.0 m water above datum
  publishChartDatumTransform(tf_buffer, -1.0);

  nav2_costmap_2d::LayeredCostmap layered_costmap("map", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);

  auto * master = layered_costmap.getCostmap();

  auto layer = std::make_shared<s57_layer::S57Layer>();
  layer->initialize(&layered_costmap, "chart_layer", tf_buffer.get(), node, nullptr);

  // First cycle: tiles become complete (no charts, uncharted allowed)
  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);
  layer->updateCosts(*master, 0, 0, 10, 10);

  // Should be current after processing
  EXPECT_TRUE(layer->isCurrent());

  // Change tide: chart datum at Z = -2.5 (offset changes from 1.0 to 2.5)
  publishChartDatumTransform(tf_buffer, -2.5);

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

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  publishChartDatumTransform(tf_buffer, -1.0);

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
  publishChartDatumTransform(tf_buffer, -1.005);

  minx = 1e30; miny = 1e30; maxx = -1e30; maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);

  // Should still be current — change too small
  EXPECT_TRUE(layer->isCurrent());
}

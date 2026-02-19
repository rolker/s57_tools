// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

#include <gtest/gtest.h>
#include <memory>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "s57_layer.h"

class AllowUnchartedTest : public ::testing::Test
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

static int countCellsWithCost(
  nav2_costmap_2d::Costmap2D * costmap, unsigned char cost)
{
  int count = 0;
  auto * charmap = costmap->getCharMap();
  unsigned int total = costmap->getSizeInCellsX() * costmap->getSizeInCellsY();
  for (unsigned int i = 0; i < total; i++) {
    if (charmap[i] == cost) {
      count++;
    }
  }
  return count;
}

// When allow_uncharted=true (default) and no chart service is available,
// updateCosts should leave the master costmap untouched.
TEST_F(AllowUnchartedTest, TrueKeepsMasterCostmapClean)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_uncharted_true");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // 10x10 cells, 1m resolution, origin at (0,0), track_unknown=false
  // so default cell value is FREE_SPACE
  nav2_costmap_2d::LayeredCostmap layered_costmap("map", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);

  auto * master = layered_costmap.getCostmap();
  ASSERT_EQ(countCellsWithCost(master, nav2_costmap_2d::FREE_SPACE), 100);

  // Initialize layer — allow_uncharted defaults to true
  auto layer = std::make_shared<s57_layer::S57Layer>();
  layer->initialize(&layered_costmap, "chart_layer", tf_buffer.get(), node, nullptr);

  // updateBounds triggers tile generation. With no charts available,
  // tiles are marked complete immediately.
  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);

  // updateCosts with the full costmap range
  layer->updateCosts(*master, 0, 0, 10, 10);

  // Master costmap should be untouched — still all FREE_SPACE
  EXPECT_EQ(countCellsWithCost(master, nav2_costmap_2d::FREE_SPACE), 100);
  EXPECT_EQ(countCellsWithCost(master, nav2_costmap_2d::NO_INFORMATION), 0);
}

// When allow_uncharted=false and no chart service is available,
// updateCosts should write NO_INFORMATION to all cells.
TEST_F(AllowUnchartedTest, FalseWritesNoInformation)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_uncharted_false");

  // Pre-declare parameter before layer initialization
  node->declare_parameter("chart_layer.allow_uncharted", false);

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  nav2_costmap_2d::LayeredCostmap layered_costmap("map", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);

  auto * master = layered_costmap.getCostmap();
  ASSERT_EQ(countCellsWithCost(master, nav2_costmap_2d::FREE_SPACE), 100);

  auto layer = std::make_shared<s57_layer::S57Layer>();
  layer->initialize(&layered_costmap, "chart_layer", tf_buffer.get(), node, nullptr);

  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);

  layer->updateCosts(*master, 0, 0, 10, 10);

  // All cells should be NO_INFORMATION — uncharted navigation blocked
  EXPECT_EQ(countCellsWithCost(master, nav2_costmap_2d::NO_INFORMATION), 100);
  EXPECT_EQ(countCellsWithCost(master, nav2_costmap_2d::FREE_SPACE), 0);
}

// Verify that current_ is set correctly after updateCosts.
// With no charts and allow_uncharted=true, current_ should be true
// (tiles are complete even though they have no chart data).
TEST_F(AllowUnchartedTest, CurrentFlagReflectsCompleteness)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_current_flag");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  nav2_costmap_2d::LayeredCostmap layered_costmap("map", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);

  auto * master = layered_costmap.getCostmap();

  auto layer = std::make_shared<s57_layer::S57Layer>();
  layer->initialize(&layered_costmap, "chart_layer", tf_buffer.get(), node, nullptr);

  // Before updateBounds/updateCosts, current_ should be false
  EXPECT_FALSE(layer->isCurrent());

  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  layer->updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);
  layer->updateCosts(*master, 0, 0, 10, 10);

  // After processing uncharted tiles (all complete), current_ should be true
  EXPECT_TRUE(layer->isCurrent());
}

// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

// The layer on a rolling window, driven by Nav2's own cycle:
// LayeredCostmap::updateMap moves the window to the robot, asks each layer
// for its bounds, resets the window and has each layer paint it. The layer's
// origin is captured once, by matchSize at the first resizeMap, as on a
// costmap node; a window that then rolls below it on either axis must still
// be painted to its west and south edge (#56).

#include <gtest/gtest.h>

#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "s57_msgs/msg/dataset_info.hpp"
#include "tf2_ros/buffer.h"

#include "s57_layer.h"

// Nav2's lifecycle node moved from nav2_util (Jazzy) to nav2_ros_common
// (Rolling); Layer::initialize takes a weak pointer to whichever one exists.
#if __has_include("nav2_ros_common/lifecycle_node.hpp")
#include "nav2_ros_common/lifecycle_node.hpp"
using TestLifecycleNode = nav2::LifecycleNode;
#else
#include "nav2_util/lifecycle_node.hpp"
using TestLifecycleNode = nav2_util::LifecycleNode;
#endif

namespace
{

// Gives the layer a chart directly, as getDatasetsCallback and
// mapGridCallback would, so no service, topic, TF or spinning is needed.
class S57LayerForTest : public s57_layer::S57Layer
{
public:
  void addChart(const std::string & label, std::shared_ptr<grid_map::GridMap> grid)
  {
    s57_msgs::msg::DatasetInfo info;
    info.label = label;
    current_charts_.push_back(info);

    const auto & position = grid->getPosition();
    const auto & length = grid->getLength();
    geometry_msgs::msg::Point min_point, max_point;
    min_point.x = position.x() - length.x() / 2.0;
    min_point.y = position.y() - length.y() / 2.0;
    max_point.x = position.x() + length.x() / 2.0;
    max_point.y = position.y() + length.y() / 2.0;
    chart_bounds_[label] = std::make_pair(min_point, max_point);

    grids_[label] = grid;
  }
};

// A chart grid with every channel the layer reads, all NaN but elevation,
// which is land (1.0) where is_land says so and 10 m of water elsewhere.
template<typename IsLand>
std::shared_ptr<grid_map::GridMap> makeChartGrid(
  double center_x, double center_y, double length, double resolution, IsLand is_land)
{
  auto grid = std::make_shared<grid_map::GridMap>(
    std::vector<std::string>{"elevation", "overhead", "restricted", "unsurveyed", "caution"});
  grid->setGeometry(
    grid_map::Length(length, length), resolution, grid_map::Position(center_x, center_y));
  for (const auto & name : grid->getLayers()) {
    (*grid)[name].setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  for (grid_map::GridMapIterator it(*grid); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    grid->getPosition(*it, position);
    grid->at("elevation", *it) = is_land(position.x(), position.y()) ? 1.0f : -10.0f;
  }
  return grid;
}

std::shared_ptr<grid_map::GridMap> makeLandGrid(double center_x, double center_y)
{
  return makeChartGrid(center_x, center_y, 100.0, 1.0, [](double, double) {return true;});
}

// Every cell of the window equals expected; on failure, how many do not,
// where the window is, and the first one that does not.
::testing::AssertionResult everyCellIs(
  nav2_costmap_2d::Costmap2D & costmap, unsigned char expected)
{
  unsigned int wrong = 0;
  std::ostringstream first;
  for (unsigned int j = 0; j < costmap.getSizeInCellsY(); j++) {
    for (unsigned int i = 0; i < costmap.getSizeInCellsX(); i++) {
      unsigned char cost = costmap.getCost(i, j);
      if (cost != expected) {
        if (wrong == 0) {
          double wx, wy;
          costmap.mapToWorld(i, j, wx, wy);
          first << "first wrong cell (" << i << ", " << j << ") centred at (" << wx << ", " <<
            wy << ") has cost " << static_cast<int>(cost);
        }
        wrong++;
      }
    }
  }
  if (wrong == 0) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() <<
         wrong << " of " << costmap.getSizeInCellsX() * costmap.getSizeInCellsY() <<
         " cells are not " << static_cast<int>(expected) << "; window origin (" <<
         costmap.getOriginX() << ", " << costmap.getOriginY() << "); " << first.str();
}

class RollingWindowTest : public ::testing::Test
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

  // A rolling window of 10 x 10 cells at 1 m whose layer origin is (0, 0),
  // with tiles of 4 cells so the window has a partial tile at every edge.
  // In a costmap node's order: resizeMap, addPlugin, initialize, so the
  // layer's matchSize takes origin (0, 0).
  void makeWindow(bool allow_uncharted = true)
  {
    node_ = std::make_shared<TestLifecycleNode>("test_rolling_window");
    node_->declare_parameter("chart_layer.tile_size", 4);
    node_->declare_parameter("chart_layer.allow_uncharted", allow_uncharted);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", true, false);
    layered_costmap_->resizeMap(10, 10, 1.0, 0.0, 0.0);
    layer_ = std::make_shared<S57LayerForTest>();
    layered_costmap_->addPlugin(layer_);
    layer_->initialize(layered_costmap_.get(), "chart_layer", tf_buffer_.get(), node_, nullptr);
  }

  void TearDown() override
  {
    layered_costmap_.reset();
    inflation_.reset();
    layer_.reset();
    node_.reset();
  }

  // One cycle with the robot at (x, y) over an all-land chart around it.
  ::testing::AssertionResult allLandAt(
    double robot_x, double robot_y, double origin_x, double origin_y)
  {
    makeWindow();
    layer_->addChart("land", makeLandGrid(robot_x, robot_y));
    layered_costmap_->updateMap(robot_x, robot_y, 0.0);
    auto * master = layered_costmap_->getCostmap();
    if (master->getOriginX() != origin_x || master->getOriginY() != origin_y) {
      return ::testing::AssertionFailure() << "window origin is (" << master->getOriginX() <<
             ", " << master->getOriginY() << "), not (" << origin_x << ", " << origin_y << ")";
    }
    return everyCellIs(*master, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  std::shared_ptr<TestLifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layered_costmap_;
  std::shared_ptr<S57LayerForTest> layer_;
  std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation_;
};

// The window's lower-left corner is west and south of the layer origin.
TEST_F(RollingWindowTest, BelowOriginOnBothAxes)
{
  EXPECT_TRUE(allLandAt(0.0, 0.0, -5.0, -5.0));
}

TEST_F(RollingWindowTest, BelowOriginOnXOnly)
{
  EXPECT_TRUE(allLandAt(0.0, 20.0, -5.0, 15.0));
}

TEST_F(RollingWindowTest, BelowOriginOnYOnly)
{
  EXPECT_TRUE(allLandAt(20.0, 0.0, 15.0, -5.0));
}

// Not a control: updateCosts finds its first tile from the centre of cell 0,
// which is never on a tile edge, so a corner on an exact tile multiple loses
// a whole tile column and row.
TEST_F(RollingWindowTest, BelowOriginOnAnExactTileMultiple)
{
  EXPECT_TRUE(allLandAt(-3.0, -3.0, -8.0, -8.0));
}

// Far from the origin, with a robot position that is not on a cell; the
// window snaps to the grid as Costmap2D::updateOrigin keeps it aligned.
TEST_F(RollingWindowTest, FarBelowOriginFromAFractionalPosition)
{
  EXPECT_TRUE(allLandAt(-1000.25, -2000.75, -1005.0, -2005.0));
}

// Control: the window is wholly above the origin, where the fix changes
// nothing.
TEST_F(RollingWindowTest, AboveOriginControl)
{
  EXPECT_TRUE(allLandAt(20.0, 20.0, 15.0, 15.0));
}

// allow_uncharted false and no chart: the window is marked unknown to its
// edge. Driven as a costmap node's first cycle is: the node sets the
// footprint after its plugins are initialised, so the inflation layer after
// this one asks for the whole window and this layer paints all of it. The
// inflation layer is needed: with no chart this layer reports no bounds of
// its own (a separate defect), so without it nothing would be painted, with
// or without the fix for #56.
TEST_F(RollingWindowTest, UnchartedBelowOriginMarkedUnknown)
{
  makeWindow(false);
  inflation_ = std::make_shared<nav2_costmap_2d::InflationLayer>();
  layered_costmap_->addPlugin(inflation_);
  inflation_->initialize(
    layered_costmap_.get(), "inflation_layer", tf_buffer_.get(), node_, nullptr);

  std::vector<geometry_msgs::msg::Point> footprint(4);
  for (std::size_t k = 0; k < footprint.size(); k++) {
    footprint[k].x = (k == 0 || k == 3) ? -0.25 : 0.25;
    footprint[k].y = (k < 2) ? -0.25 : 0.25;
  }
  layered_costmap_->setFootprint(footprint);

  layered_costmap_->updateMap(0.0, 0.0, 0.0);

  auto * master = layered_costmap_->getCostmap();
  ASSERT_EQ(master->getOriginX(), -5.0);
  ASSERT_EQ(master->getOriginY(), -5.0);
  EXPECT_TRUE(everyCellIs(*master, nav2_costmap_2d::NO_INFORMATION));
}

// The master-to-layer cell offset at a resolution that is not binary-exact.
// At 0.1 m, moving the window from 0 towards 0.35 puts its origin at
// 0 + 3*0.1 = 0.30000000000000004, so the offset of the tile above it is
// -0.9999999999999996 cells, which truncates to 0 and copies that tile one
// cell west. The oracle is a window that never moved, over the same chart
// with the same layer origin, compared cell by cell at the same world point.
// This must stay one cycle, painting in the cycle that moves the window:
// painted at origin 0 first, the cells a later move exposes are not repainted
// at all (a separate defect), so a two-cycle version passes without the fix.
TEST_F(RollingWindowTest, NonBinaryResolutionOffsetRoundsToNearestCell)
{
  // Land west of 0.6 m and in a band from 0.9 to 1.0 m, 10 m of water
  // elsewhere, so a shift of one cell shows at a coast in two tiles.
  auto coast = makeChartGrid(
    1.0, 1.0, 4.0, 0.1, [](double x, double) {return x < 0.6 || (x > 0.9 && x < 1.0);});

  auto node = std::make_shared<TestLifecycleNode>("test_rolling_window_offset");
  node->declare_parameter("chart_layer.tile_size", 4);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  nav2_costmap_2d::LayeredCostmap oracle_costmap("map", false, false);
  oracle_costmap.resizeMap(20, 20, 0.1, 0.0, 0.0);
  auto oracle_layer = std::make_shared<S57LayerForTest>();
  oracle_costmap.addPlugin(oracle_layer);
  oracle_layer->initialize(&oracle_costmap, "chart_layer", tf_buffer.get(), node, nullptr);
  oracle_layer->addChart("coast", coast);
  oracle_costmap.updateMap(1.0, 1.0, 0.0);
  auto * oracle = oracle_costmap.getCostmap();

  nav2_costmap_2d::LayeredCostmap rolling_costmap("map", true, false);
  rolling_costmap.resizeMap(10, 10, 0.1, 0.0, 0.0);
  auto rolling_layer = std::make_shared<S57LayerForTest>();
  rolling_costmap.addPlugin(rolling_layer);
  rolling_layer->initialize(&rolling_costmap, "chart_layer", tf_buffer.get(), node, nullptr);
  rolling_layer->addChart("coast", coast);
  rolling_costmap.updateMap(0.85, 0.5, 0.0);
  auto * master = rolling_costmap.getCostmap();

  // The origin this case relies on; without it the test would pass vacuously.
  ASSERT_EQ(master->getOriginX(), 0.30000000000000004);
  ASSERT_EQ(master->getOriginY(), 0.0);

  unsigned int wrong = 0;
  std::ostringstream first;
  for (unsigned int j = 0; j < master->getSizeInCellsY(); j++) {
    for (unsigned int i = 0; i < master->getSizeInCellsX(); i++) {
      double wx, wy;
      master->mapToWorld(i, j, wx, wy);
      unsigned int oi, oj;
      ASSERT_TRUE(oracle->worldToMap(wx, wy, oi, oj));
      unsigned char cost = master->getCost(i, j);
      unsigned char expected = oracle->getCost(oi, oj);
      if (cost != expected) {
        if (wrong == 0) {
          first << "first wrong cell (" << i << ", " << j << ") centred at (" << wx << ", " <<
            wy << ") has cost " << static_cast<int>(cost) << " where the unmoved window has " <<
            static_cast<int>(expected);
        }
        wrong++;
      }
    }
  }
  EXPECT_EQ(wrong, 0u) << wrong << " of 100 cells differ from the unmoved window; window origin (" <<
    std::setprecision(17) << master->getOriginX() << ", " << master->getOriginY() << "); " <<
    first.str();
}

}  // namespace

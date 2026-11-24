// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

// The layer on a rolling window, driven by Nav2's own cycle:
// LayeredCostmap::updateMap moves the window to the robot, asks each layer
// for its bounds, resets the window and has each layer paint it. The layer's
// origin is captured once, by matchSize at the first resizeMap, as on a
// costmap node; a window that then rolls below it on either axis must still
// be painted to its west and south edge (#56), and the cells each later move
// uncovers must be painted too (#57, MovingWindowTest below).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
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
  // A chart whose grid has not arrived yet (loaded false) is listed, with its
  // bounds, as getDatasetsCallback lists it; setGrid then delivers the grid
  // as mapGridCallback would.
  void addChart(
    const std::string & label, std::shared_ptr<grid_map::GridMap> grid, bool loaded = true)
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

    grids_[label] = loaded ? grid : nullptr;
  }

  void setGrid(const std::string & label, std::shared_ptr<grid_map::GridMap> grid)
  {
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
  void makeWindow(bool allow_uncharted = true, bool rolling = true)
  {
    node_ = std::make_shared<TestLifecycleNode>("test_rolling_window");
    node_->declare_parameter("chart_layer.tile_size", 4);
    node_->declare_parameter("chart_layer.allow_uncharted", allow_uncharted);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", rolling, false);
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

// The robot positions are chosen so that Jazzy and Rolling put the window at
// the same origin. LayeredCostmap::updateMap centres the window with
// robot - getSizeInMetersX() / 2, and Costmap2D::updateOrigin then truncates
// the offset from the old origin, in cells, toward zero. Rolling's
// getSizeInMetersX is size * resolution (10 m here, since Nav2 #6028);
// Jazzy's is (size - 1 + 0.5) * resolution (9.5 m), which asks for an origin
// a quarter metre further east and north. Every position below truncates to
// the same cell under both: at -0.5, Rolling asks for -5.5 and Jazzy for
// -5.25, both -5; at 20, Rolling asks for 15 and Jazzy for 15.25, both 15.
// Each case asserts the origin, so a wrong assumption fails on the origin,
// not on a cell count.

// The window's lower-left corner is west and south of the layer origin.
TEST_F(RollingWindowTest, BelowOriginOnBothAxes)
{
  EXPECT_TRUE(allLandAt(-0.5, -0.5, -5.0, -5.0));
}

TEST_F(RollingWindowTest, BelowOriginOnXOnly)
{
  EXPECT_TRUE(allLandAt(-0.5, 19.5, -5.0, 14.0));
}

TEST_F(RollingWindowTest, BelowOriginOnYOnly)
{
  EXPECT_TRUE(allLandAt(19.5, -0.5, 14.0, -5.0));
}

// Not a control: updateCosts finds its first tile from the centre of cell 0,
// which is never on a tile edge, so a corner on an exact tile multiple loses
// a whole tile column and row.
TEST_F(RollingWindowTest, BelowOriginOnAnExactTileMultiple)
{
  EXPECT_TRUE(allLandAt(-3.5, -3.5, -8.0, -8.0));
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
// edge. Driven as a costmap node's first cycle is, with the inflation layer
// a costmap node configures after this one: the node sets the footprint
// after its plugins are initialised, so the inflation layer asks for the
// whole window and this layer paints all of it. This layer also reports its
// own tiles with no chart loaded (#57); UnchartedFixedWindowMarkedUnknown
// tests that without an inflation layer.
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

  // Robot at -0.5: the same origin on Jazzy and Rolling (see above).
  layered_costmap_->updateMap(-0.5, -0.5, 0.0);

  auto * master = layered_costmap_->getCostmap();
  ASSERT_EQ(master->getOriginX(), -5.0);
  ASSERT_EQ(master->getOriginY(), -5.0);
  EXPECT_TRUE(everyCellIs(*master, nav2_costmap_2d::NO_INFORMATION));
}

// allow_uncharted false and no chart, on a window that does not move and
// with no other layer: the tiles this layer creates while no chart is loaded
// are reported in its own bounds, so the first cycle marks every cell
// unknown. Before #57 they were not, and the window stayed free.
TEST_F(RollingWindowTest, UnchartedFixedWindowMarkedUnknown)
{
  makeWindow(false, false);
  layered_costmap_->updateMap(5.0, 5.0, 0.0);

  auto * master = layered_costmap_->getCostmap();
  ASSERT_EQ(master->getOriginX(), 0.0);
  ASSERT_EQ(master->getOriginY(), 0.0);
  EXPECT_TRUE(everyCellIs(*master, nav2_costmap_2d::NO_INFORMATION));
}

// A layer that widens the bounds to a margin around the window (no margin:
// it reports nothing) and records the bounds it was given.
class BoundsLayer : public nav2_costmap_2d::Layer
{
public:
  explicit BoundsLayer(double margin)
  : margin_(margin) {}
  void onInitialize() override {current_ = true;}
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(
    double, double, double, double * min_x, double * min_y, double * max_x,
    double * max_y) override
  {
    given = {*min_x, *min_y, *max_x, *max_y};
    if (margin_ > 0.0) {
      auto * costmap = layered_costmap_->getCostmap();
      *min_x = std::min(*min_x, costmap->getOriginX() - margin_);
      *min_y = std::min(*min_y, costmap->getOriginY() - margin_);
      *max_x = std::max(*max_x, costmap->getOriginX() + costmap->getSizeInMetersX() + margin_);
      *max_y = std::max(*max_y, costmap->getOriginY() + costmap->getSizeInMetersY() + margin_);
    }
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override {}

  std::vector<double> given;

private:
  double margin_;
};

// The layer widens the bounds the layers before it reported and never
// narrows them (Nav2's layer contract). Before #57 it clamped them to the
// window whenever it had tiles to report, which Nav2 logs as an illegal
// bounds change; Nav2 clips to the window itself.
TEST_F(RollingWindowTest, BoundsFromEarlierLayersAreNotNarrowed)
{
  node_ = std::make_shared<TestLifecycleNode>("test_rolling_window");
  node_->declare_parameter("chart_layer.tile_size", 4);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", true, false);
  layered_costmap_->resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto wide = std::make_shared<BoundsLayer>(50.0);
  layered_costmap_->addPlugin(wide);
  wide->initialize(layered_costmap_.get(), "wide", tf_buffer_.get(), node_, nullptr);
  layer_ = std::make_shared<S57LayerForTest>();
  layered_costmap_->addPlugin(layer_);
  layer_->initialize(layered_costmap_.get(), "chart_layer", tf_buffer_.get(), node_, nullptr);
  auto spy = std::make_shared<BoundsLayer>(0.0);
  layered_costmap_->addPlugin(spy);
  spy->initialize(layered_costmap_.get(), "spy", tf_buffer_.get(), node_, nullptr);
  layer_->addChart("land", makeLandGrid(20.25, 20.25));

  // The first cycle, when every tile in the window is new and reported.
  layered_costmap_->updateMap(20.5, 20.5, 0.0);
  auto * master = layered_costmap_->getCostmap();
  ASSERT_EQ(master->getOriginX(), 15.0);
  ASSERT_EQ(master->getOriginY(), 15.0);
  ASSERT_EQ(spy->given.size(), 4u);
  EXPECT_EQ(spy->given[0], 15.0 - 50.0);
  EXPECT_EQ(spy->given[1], 15.0 - 50.0);
  EXPECT_EQ(spy->given[2], 15.0 + master->getSizeInMetersX() + 50.0);
  EXPECT_EQ(spy->given[3], 15.0 + master->getSizeInMetersY() + 50.0);
  EXPECT_TRUE(everyCellIs(*master, nav2_costmap_2d::LETHAL_OBSTACLE));
}

// One cycle of a 10 x 10 rolling window at 0.1 m whose layer origin is
// (0, 0), with the robot at (robot_x, robot_y), compared cell by cell at the
// same world point with an oracle over the same chart: a 20 x 20 window with
// the same layer origin, so the same tiles, moved by one cycle with the robot
// at (oracle_robot, oracle_robot) to an origin a whole number of cells from
// the layer origin in floating point, so its own offset is exact and any
// rounding of it is right.
::testing::AssertionResult matchesOracle(
  std::shared_ptr<grid_map::GridMap> chart, double oracle_robot, double oracle_origin,
  double robot_x, double robot_y, double origin_x, double origin_y)
{
  auto node = std::make_shared<TestLifecycleNode>("test_rolling_window_offset");
  node->declare_parameter("chart_layer.tile_size", 4);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  nav2_costmap_2d::LayeredCostmap oracle_costmap("map", true, false);
  oracle_costmap.resizeMap(20, 20, 0.1, 0.0, 0.0);
  auto oracle_layer = std::make_shared<S57LayerForTest>();
  oracle_costmap.addPlugin(oracle_layer);
  oracle_layer->initialize(&oracle_costmap, "chart_layer", tf_buffer.get(), node, nullptr);
  oracle_layer->addChart("coast", chart);
  oracle_costmap.updateMap(oracle_robot, oracle_robot, 0.0);
  auto * oracle = oracle_costmap.getCostmap();
  if (oracle->getOriginX() != oracle_origin || oracle->getOriginY() != oracle_origin ||
    oracle_origin / 0.1 != std::round(oracle_origin / 0.1))
  {
    return ::testing::AssertionFailure() << std::setprecision(17) <<
           "oracle window origin is (" << oracle->getOriginX() << ", " << oracle->getOriginY() <<
           "), not (" << oracle_origin << ", " << oracle_origin <<
           ") or not a whole number of cells from the layer origin";
  }

  nav2_costmap_2d::LayeredCostmap rolling_costmap("map", true, false);
  rolling_costmap.resizeMap(10, 10, 0.1, 0.0, 0.0);
  auto rolling_layer = std::make_shared<S57LayerForTest>();
  rolling_costmap.addPlugin(rolling_layer);
  rolling_layer->initialize(&rolling_costmap, "chart_layer", tf_buffer.get(), node, nullptr);
  rolling_layer->addChart("coast", chart);
  rolling_costmap.updateMap(robot_x, robot_y, 0.0);
  auto * master = rolling_costmap.getCostmap();

  // The origin the case relies on; without it the test would pass vacuously.
  if (master->getOriginX() != origin_x || master->getOriginY() != origin_y) {
    return ::testing::AssertionFailure() << std::setprecision(17) << "window origin is (" <<
           master->getOriginX() << ", " << master->getOriginY() << "), not (" << origin_x <<
           ", " << origin_y << ")";
  }

  unsigned int wrong = 0, oracle_land = 0, oracle_water = 0;
  std::ostringstream first;
  for (unsigned int j = 0; j < master->getSizeInCellsY(); j++) {
    for (unsigned int i = 0; i < master->getSizeInCellsX(); i++) {
      double wx, wy;
      master->mapToWorld(i, j, wx, wy);
      unsigned int oi, oj;
      if (!oracle->worldToMap(wx, wy, oi, oj)) {
        return ::testing::AssertionFailure() << "cell (" << i << ", " << j << ") centred at (" <<
               wx << ", " << wy << ") is outside the oracle";
      }
      unsigned char cost = master->getCost(i, j);
      unsigned char expected = oracle->getCost(oi, oj);
      oracle_land += expected == nav2_costmap_2d::LETHAL_OBSTACLE;
      oracle_water += expected == nav2_costmap_2d::FREE_SPACE;
      if (cost != expected) {
        if (wrong == 0) {
          first << "first wrong cell (" << i << ", " << j << ") centred at (" << wx << ", " <<
            wy << ") has cost " << static_cast<int>(cost) << " where the oracle has " <<
            static_cast<int>(expected);
        }
        wrong++;
      }
    }
  }
  // The oracle runs the same layer code: it must show the coast, or a
  // comparison of two unpainted windows would pass.
  if (oracle_land == 0 || oracle_water == 0) {
    return ::testing::AssertionFailure() << "the oracle shows " << oracle_land <<
           " land and " << oracle_water << " water cells under the window, not a coast";
  }
  if (wrong == 0) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << wrong <<
         " of 100 cells differ from the oracle; window origin (" <<
         std::setprecision(17) << master->getOriginX() << ", " << master->getOriginY() << "); " <<
         first.str();
}

// The master-to-layer cell offset at a resolution that is not binary-exact.
// At 0.1 m, moving the window from 0 towards 0.35 (0.375 on Jazzy, see the
// size formulas above; both truncate to 3 cells) puts its origin at
// 0 + 3*0.1 = 0.30000000000000004, so the offset of the tile above it is
// -0.9999999999999996 cells, which truncates to 0 and copies that tile one
// cell west. The oracle's origin is 0, where it starts.
// One cycle, painting in the cycle that moves the window: the rounding is in
// updateCosts, which every repaint goes through, including the strips a later
// move uncovers (#57), so a second cycle would test nothing more.
TEST_F(RollingWindowTest, NonBinaryResolutionOffsetRoundsToNearestCell)
{
  // Land west of 0.6 m and in a band from 0.9 to 1.0 m, 10 m of water
  // elsewhere, so a shift of one cell shows at a coast in two tiles.
  auto coast = makeChartGrid(
    1.0, 1.0, 4.0, 0.1, [](double x, double) {return x < 0.6 || (x > 0.9 && x < 1.0);});
  EXPECT_TRUE(matchesOracle(coast, 1.0, 0.0, 0.85, 0.5, 0.30000000000000004, 0.0));
}

// The same on both axes below the layer origin, so the y offset and a
// negative one are rounded too. With the robot at (0.14, 0.14) the window
// asks for -0.36 (Rolling) or -0.335 (Jazzy), both 3 cells below 0, and its
// origin is 0 + -3*0.1 = -0.30000000000000004 on both axes. The offset of
// the tile below 0 (tile -1) is then 4 - 3.0000000000000004 =
// 0.9999999999999996 cells, which truncates to 0 and copies that tile one
// cell east and north. The other tiles in the window have offsets that
// truncate correctly. One cycle, for the reason above. The oracle's robot at
// -0.06 puts its origin at -1.0 on both distros (-1.06 and -1.035 both
// truncate to -10 cells), and -1.0 / 0.1 is exactly -10.
TEST_F(RollingWindowTest, NegativeNonBinaryOffsetRoundsToNearestCellOnBothAxes)
{
  // Land west of -0.1 m and south of -0.1 m, 10 m of water elsewhere, so a
  // shift of one cell shows at the coast inside tile -1 on either axis.
  auto coast = makeChartGrid(
    0.0, 0.0, 4.0, 0.1, [](double x, double y) {return x < -0.1 || y < -0.1;});
  EXPECT_TRUE(
    matchesOracle(coast, -0.06, -1.0, 0.14, 0.14, -0.30000000000000004, -0.30000000000000004));
}

// ---------------------------------------------------------------------------
// Several cycles of a moving window (#57).
//
// When a rolling window moves, Nav2 resets the cells it uncovers to the
// default value, and the layer reports them as strips (s57_layer.h): on
// travel along one axis in the same cycle (one cycle late when the travel
// reverses), otherwise within 3 cycles. Each
// case drives LayeredCostmap::updateMap along a path and compares every cell
// after every cycle with an oracle: a fresh costmap with the same layers and
// charts, moved to the same window in its first cycle, which paints all of
// it. With no inflation layer every cell must equal the oracle's; with one,
// whether a cell is lethal and whether it is unknown (Jazzy's queue
// inflation over a part of the window can differ from a whole pass by a few
// cost units, and the Jazzy CI job runs these tests).
//
// Each case runs with no inflation layer, and with Nav2's InflationLayer at
// a radius under a tile (2 m; tiles are 4 m) and at or above one (6 m), its
// footprint set as a costmap node sets it. The inflation layer widens the
// bounds it is given around this and last cycle's, which repaints some
// uncovered cells by itself: a case may pass without the fix under one set
// up and fail under another. Which fail without it is in the pull request
// for #57.
//
// Window origins. The window is 20 x 20 cells at 1 m. Nav2 centres it with
// robot - getSizeInMetersX() / 2 and truncates the offset from the old
// origin, in cells, toward zero; getSizeInMetersX is 20 m on Rolling and
// 19.5 m on Jazzy (see above). Every path below is checked, step by step, to
// give the same origin under both formulas, and the window's origin is
// asserted every cycle. That is why the slow diagonals move 0.5 or 1.5 m per
// cycle and not, say, 0.6 m: a step whose fractional part is not 0 or 0.5
// brings the offset's fraction into [0.75, 1) at some cycle, where the two
// formulas truncate to different cells.
//
// Charts. Their cell edges lie a quarter cell off the tile edges, and they
// cover every tile a path reaches, so rolker/s57_tools#63 (a chart cell
// edge on a tile edge) and #59 (a tile whose centre is outside a chart)
// cannot affect these cases.

constexpr unsigned int kCells = 20;

enum class Inflation { none, under_tile, over_tile };

const char * inflationName(Inflation inflation)
{
  switch (inflation) {
    case Inflation::none: return "no inflation layer";
    case Inflation::under_tile: return "inflation radius 2 m (under a tile)";
    default: return "inflation radius 6 m (a tile and more)";
  }
}

// One costmap as a costmap node builds it: a 20 x 20 window at 1 m, resized
// at (0, 0) so the layer's tiles start there, chart_layer, and optionally
// Nav2's InflationLayer, then the footprint.
struct Costmap
{
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> costmap;
  std::shared_ptr<S57LayerForTest> layer;
  std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation;
};

struct ChartSpec
{
  std::string label;
  std::shared_ptr<grid_map::GridMap> grid;
  bool loaded;
};

struct Step
{
  double x;
  double y;
  enum Event { none, reset, grid_arrives } event = none;
};

// The origin updateMap gives the window, from origin, for the robot at
// robot, when getSizeInMetersX() is size.
double nextOrigin(double origin, double robot, double size)
{
  return origin + static_cast<int>(robot - size / 2.0 - origin);
}

class MovingWindowTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void TearDown() override
  {
    window_ = Costmap();
    node_.reset();
  }

  Costmap build(
    Inflation inflation, bool rolling, bool track_unknown, const std::vector<ChartSpec> & charts)
  {
    Costmap c;
    c.costmap = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", rolling, track_unknown);
    c.costmap->resizeMap(kCells, kCells, 1.0, 0.0, 0.0);
    c.layer = std::make_shared<S57LayerForTest>();
    c.costmap->addPlugin(c.layer);
    c.layer->initialize(c.costmap.get(), "chart_layer", tf_buffer_.get(), node_, nullptr);
    for (const auto & chart : charts) {
      c.layer->addChart(chart.label, chart.grid, chart.loaded);
    }
    if (inflation != Inflation::none) {
      c.inflation = std::make_shared<nav2_costmap_2d::InflationLayer>();
      c.costmap->addPlugin(c.inflation);
      c.inflation->initialize(
        c.costmap.get(), "inflation_layer", tf_buffer_.get(), node_, nullptr);
      std::vector<geometry_msgs::msg::Point> footprint(4);
      for (std::size_t k = 0; k < footprint.size(); k++) {
        footprint[k].x = (k == 0 || k == 3) ? -0.25 : 0.25;
        footprint[k].y = (k < 2) ? -0.25 : 0.25;
      }
      c.costmap->setFootprint(footprint);
    }
    return c;
  }

  void makeNode(Inflation inflation)
  {
    node_ = std::make_shared<TestLifecycleNode>("test_moving_window");
    node_->declare_parameter("chart_layer.tile_size", 4);
    node_->declare_parameter(
      "inflation_layer.inflation_radius", inflation == Inflation::over_tile ? 6.0 : 2.0);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  }

  // The oracle for the window at (origin_x, origin_y): a fresh costmap moved
  // there in its first cycle. Its robot asks for 0.4 m past the origin, away
  // from zero, which truncates to the origin under either size formula.
  ::testing::AssertionResult oracleAt(
    Inflation inflation, bool rolling, bool track_unknown, const std::vector<ChartSpec> & charts,
    double origin_x, double origin_y, double robot_x, double robot_y, Costmap & oracle)
  {
    oracle = build(inflation, rolling, track_unknown, charts);
    if (rolling) {
      robot_x = origin_x + (origin_x < 0.0 ? -0.4 : 0.4) + kCells / 2.0;
      robot_y = origin_y + (origin_y < 0.0 ? -0.4 : 0.4) + kCells / 2.0;
    }
    oracle.costmap->updateMap(robot_x, robot_y, 0.0);
    auto * o = oracle.costmap->getCostmap();
    if (o->getOriginX() != origin_x || o->getOriginY() != origin_y) {
      return ::testing::AssertionFailure() << "the oracle's window origin is (" <<
             o->getOriginX() << ", " << o->getOriginY() << "), not (" << origin_x << ", " <<
             origin_y << ")";
    }
    return ::testing::AssertionSuccess();
  }

  // Drives the window along path under one set-up. allowance is the number of
  // cycles a cell may wait after entering the window: a cell that has been in
  // it for allowance or more cycles must match the oracle (0: every cell,
  // every cycle). Stops at the first cycle that fails, and says why.
  ::testing::AssertionResult drive(
    Inflation inflation, const std::vector<Step> & path, int allowance,
    std::vector<ChartSpec> charts, bool rolling = true, bool track_unknown = false)
  {
    makeNode(inflation);
    window_ = build(inflation, rolling, track_unknown, charts);
    auto * master = window_.costmap->getCostmap();

    // age[j][i]: cycles the cell at (i, j) of the window has been in it.
    std::vector<std::vector<int>> age(kCells, std::vector<int>(kCells, 0));
    double rolling_origin_x = 0.0, rolling_origin_y = 0.0;
    double jazzy_origin_x = 0.0, jazzy_origin_y = 0.0;
    for (std::size_t step = 0; step < path.size(); step++) {
      const Step & s = path[step];
      std::ostringstream where;
      where << "cycle " << step << ", robot at (" << s.x << ", " << s.y << "): ";
      if (s.event == Step::reset) {
        // As Costmap2DROS::resetLayers does.
        master->resetMap(0, 0, master->getSizeInCellsX(), master->getSizeInCellsY());
        window_.layer->reset();
        if (window_.inflation) {
          window_.inflation->reset();
        }
      }
      if (s.event == Step::grid_arrives) {
        for (auto & chart : charts) {
          if (!chart.loaded) {
            window_.layer->setGrid(chart.label, chart.grid);
            chart.loaded = true;
          }
        }
      }

      const double old_x = master->getOriginX(), old_y = master->getOriginY();
      window_.costmap->updateMap(s.x, s.y, 0.0);

      if (rolling) {
        rolling_origin_x = nextOrigin(rolling_origin_x, s.x, kCells);
        rolling_origin_y = nextOrigin(rolling_origin_y, s.y, kCells);
        jazzy_origin_x = nextOrigin(jazzy_origin_x, s.x, kCells - 0.5);
        jazzy_origin_y = nextOrigin(jazzy_origin_y, s.y, kCells - 0.5);
        if (rolling_origin_x != jazzy_origin_x || rolling_origin_y != jazzy_origin_y) {
          return ::testing::AssertionFailure() << where.str() <<
                 "the path gives the window a different origin on Jazzy (" << jazzy_origin_x <<
                 ", " << jazzy_origin_y << ") than on Rolling (" << rolling_origin_x << ", " <<
                 rolling_origin_y << ")";
        }
      }
      if (master->getOriginX() != rolling_origin_x || master->getOriginY() != rolling_origin_y) {
        return ::testing::AssertionFailure() << where.str() << "window origin is (" <<
               master->getOriginX() << ", " << master->getOriginY() << "), not (" <<
               rolling_origin_x << ", " << rolling_origin_y << ")";
      }

      // Each cell's age: one more than the cell at the same world point had
      // last cycle, or 0 if it was outside the window.
      const long dx = std::lround(master->getOriginX() - old_x);
      const long dy = std::lround(master->getOriginY() - old_y);
      std::vector<std::vector<int>> next(kCells, std::vector<int>(kCells, 0));
      for (long j = 0; j < static_cast<long>(kCells); j++) {
        for (long i = 0; i < static_cast<long>(kCells); i++) {
          const long pi = i + dx, pj = j + dy;
          if (step > 0 && pi >= 0 && pj >= 0 && pi < static_cast<long>(kCells) &&
            pj < static_cast<long>(kCells))
          {
            next[j][i] = age[pj][pi] + 1;
          }
        }
      }
      age = next;

      Costmap oracle;
      auto made = oracleAt(
        inflation, rolling, track_unknown, charts, master->getOriginX(), master->getOriginY(),
        s.x, s.y, oracle);
      if (!made) {
        return made << " (" << where.str() << ")";
      }
      auto * o = oracle.costmap->getCostmap();
      // Every chart here has land in the window: an oracle without any would
      // let two unpainted windows compare equal.
      bool oracle_land = false;
      for (unsigned int j = 0; j < kCells && !oracle_land; j++) {
        for (unsigned int i = 0; i < kCells && !oracle_land; i++) {
          oracle_land = o->getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE;
        }
      }
      if (!oracle_land) {
        return ::testing::AssertionFailure() << where.str() << "the oracle shows no land";
      }

      unsigned int wrong = 0;
      int oldest = -1;
      std::ostringstream first;
      for (unsigned int j = 0; j < kCells; j++) {
        for (unsigned int i = 0; i < kCells; i++) {
          if (age[j][i] < allowance) {
            continue;
          }
          const unsigned char cost = master->getCost(i, j), expected = o->getCost(i, j);
          const bool same = inflation == Inflation::none ? cost == expected :
            (cost == nav2_costmap_2d::LETHAL_OBSTACLE) ==
            (expected == nav2_costmap_2d::LETHAL_OBSTACLE) &&
            (cost == nav2_costmap_2d::NO_INFORMATION) ==
            (expected == nav2_costmap_2d::NO_INFORMATION);
          if (!same) {
            if (wrong == 0) {
              double wx, wy;
              master->mapToWorld(i, j, wx, wy);
              first << "first (" << wx << ", " << wy << ") has cost " <<
                static_cast<int>(cost) << " where the oracle has " <<
                static_cast<int>(expected);
            }
            wrong++;
            oldest = std::max(oldest, age[j][i]);
          }
        }
      }
      if (wrong > 0) {
        return ::testing::AssertionFailure() << where.str() << wrong << " of " <<
               kCells * kCells << " cells differ from the oracle (window origin (" <<
               master->getOriginX() << ", " << master->getOriginY() <<
               "), allowance " << allowance << " cycles, the oldest wrong cell has been in "
               "the window " << oldest << " cycles); " << first.str();
      }
    }
    return ::testing::AssertionSuccess();
  }

  // Each set-up in turn, failures labelled with it.
  template<typename Case>
  void forEachInflation(Case run_case)
  {
    for (auto inflation : {Inflation::none, Inflation::under_tile, Inflation::over_tile}) {
      SCOPED_TRACE(inflationName(inflation));
      run_case(inflation);
      TearDown();
    }
  }

  std::shared_ptr<TestLifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  Costmap window_;
};

// All land, 200 m square around the paths, cell edges at x.25.
std::vector<ChartSpec> landChart()
{
  return {{"land", makeChartGrid(40.25, 40.25, 200.0, 1.0, [](double, double) {return true;}),
      true}};
}

// Land in about one cell in five, scattered, elsewhere 10 m of water.
std::vector<ChartSpec> scatteredChart()
{
  return {{"scattered", makeChartGrid(
        40.25, 40.25, 200.0, 1.0, [](double x, double y) {
          const long h = std::lround(std::floor(x)) * 73856093L ^
          std::lround(std::floor(y)) * 19349663L;
          return ((h % 5) + 5) % 5 == 0;
        }), true}};
}

std::vector<Step> line(double x, double y, double dx, double dy, int cycles)
{
  std::vector<Step> path;
  for (int k = 0; k < cycles; k++) {
    path.push_back({x + k * dx, y + k * dy});
  }
  return path;
}

// Then 3 cycles at the last position.
std::vector<Step> withStop(std::vector<Step> path)
{
  const Step last{path.back().x, path.back().y};
  path.insert(path.end(), 3, last);
  return path;
}

std::vector<Step> join(std::vector<Step> a, const std::vector<Step> & b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

// Travel along one axis: every cell after every cycle.
TEST_F(MovingWindowTest, EastEveryCellEveryCycle)
{
  forEachInflation([this](Inflation inflation) {
      EXPECT_TRUE(drive(inflation, withStop(line(20.5, 20.5, 1.0, 0.0, 16)), 0, landChart()));
    });
}

TEST_F(MovingWindowTest, NorthEveryCellEveryCycle)
{
  forEachInflation([this](Inflation inflation) {
      EXPECT_TRUE(drive(inflation, withStop(line(20.5, 20.5, 0.0, 1.0, 16)), 0, landChart()));
    });
}

// East 8 m, then west past the start, so the window uncovers cells on the
// side it came from: every cell within one cycle (the column uncovered in
// the cycle the travel reverses waits one cycle, see chooseStrip), and every
// cell after every cycle stopped.
TEST_F(MovingWindowTest, ReversalPastTheStartEveryCellWithinOneCycle)
{
  forEachInflation([this](Inflation inflation) {
      EXPECT_TRUE(
        drive(
          inflation, withStop(join(line(20.5, 20.5, 1.0, 0.0, 9), line(27.5, 20.5, -1.0, 0.0, 16))),
          1, landChart()));
    });
}

// Fast zig-zags, reversing every cycle or every other cycle, east-west and
// then on both axes. Each reversal is followed by a gap cycle, and the cells
// that scrolled out and back in across it must be painted: this guards
// remembering the window in every cycle, gap cycles included (remembered
// only when a strip is reported, the window comes back to cells that were
// reset while it was away and nothing reports them). East-west, every cell
// within one cycle; on both axes, within 3 cycles.
TEST_F(MovingWindowTest, FastZigZagsEveryCellWithinTheBound)
{
  // Steps of +-2 m from (20.5, 20.5), the sign flipping every period cycles.
  auto zigzag = [](int period, double ky, int cycles) {
      std::vector<Step> path{{20.5, 20.5}};
      for (int k = 1; k < cycles; k++) {
        const double d = ((k - 1) / period) % 2 ? -2.0 : 2.0;
        path.push_back({path.back().x + d, path.back().y + ky * d});
      }
      return withStop(path);
    };
  forEachInflation([this, &zigzag](Inflation inflation) {
      for (int period : {1, 2}) {
        SCOPED_TRACE("reversing every " + std::to_string(period) + " cycles");
        EXPECT_TRUE(drive(inflation, zigzag(period, 0.0, 16), 1, landChart()));
        TearDown();
        EXPECT_TRUE(drive(inflation, zigzag(period, 0.5, 16), 3, landChart()));
        TearDown();
      }
    });
}

// A jump further than the window is wide, a step, and a jump back over cells
// painted before, then travel east: every cell after every cycle.
TEST_F(MovingWindowTest, JumpPastTheWindowAndBackEveryCellEveryCycle)
{
  forEachInflation([this](Inflation inflation) {
      const std::vector<Step> jumps{{20.5, 20.5}, {21.5, 20.5}, {51.5, 44.5}, {52.5, 44.5},
        {22.5, 21.5}};
      EXPECT_TRUE(
        drive(inflation, withStop(join(jumps, line(23.5, 21.5, 1.0, 0.0, 6))), 0, landChart()));
    });
}

// A jump shorter than the window, as when localisation jumps: 10 m (more
// than a 4 m tile), so the uncovered cells reach the robot's own cell.
// Every cell after every cycle, the jump's cycle included: a move of more
// than a tile on either axis reports the whole window (recordUncovered).
// Without that, the strips of a jump would wait their turn like any other.
// After travel east the jump goes north-east; after travel north it goes
// east only, a jump on one axis, whose strip would wait a cycle behind the
// y strip reported the cycle before.
TEST_F(MovingWindowTest, JumpShorterThanTheWindowEveryCellEveryCycle)
{
  forEachInflation([this](Inflation inflation) {
      {
        SCOPED_TRACE("east, then a jump north-east");
        auto path = line(20.5, 20.5, 1.0, 0.0, 4);
        path.push_back({33.5, 30.5});
        EXPECT_TRUE(drive(inflation, withStop(path), 0, landChart()));
        TearDown();
      }
      {
        SCOPED_TRACE("north, then a jump east");
        auto path = line(20.5, 20.5, 0.0, 1.0, 4);
        path.push_back({30.5, 23.5});
        EXPECT_TRUE(drive(inflation, withStop(path), 0, landChart()));
        TearDown();
      }
    });
}

// The layers reset mid-run, as Costmap2DROS::resetLayers does, while
// travelling east: every cell after every cycle.
TEST_F(MovingWindowTest, ResetWhileMovingEveryCellEveryCycle)
{
  forEachInflation([this](Inflation inflation) {
      auto path = withStop(line(20.5, 20.5, 1.0, 0.0, 12));
      path[6].event = Step::reset;
      EXPECT_TRUE(drive(inflation, path, 0, landChart()));
    });
}

// A finer chart's grid arrives late, for tiles already in the window, while
// travelling east; it shows water over the land chart. The tiles it covers
// are reported when it arrives: this guards the strips' tile rule (a tile is
// left to the strips only when it was wholly outside last cycle's window),
// which, applied to every tile or inverted, fails this case.
TEST_F(MovingWindowTest, LateGridForTilesInTheWindowEveryCellEveryCycle)
{
  forEachInflation([this](Inflation inflation) {
      auto charts = landChart();
      charts.push_back(
        {"late", makeChartGrid(28.25, 24.25, 8.0, 0.5, [](double, double) {return false;}),
          false});
      auto path = withStop(line(20.5, 20.5, 1.0, 0.0, 10));
      path[4].event = Step::grid_arrives;
      EXPECT_TRUE(drive(inflation, path, 0, charts));
    });
}

// Slow diagonal travel: a cell may wait up to 3 cycles after entering the
// window (strips on both axes take turns, see chooseStrip), and after 3
// cycles stopped every cell is right. Slow enough that a tile column or row
// enters only every 8 cycles, so the tile-entry repaints alone do not meet
// the allowance.
TEST_F(MovingWindowTest, SlowDiagonalEveryCellWithinThreeCycles)
{
  forEachInflation([this](Inflation inflation) {
      EXPECT_TRUE(drive(inflation, withStop(line(20.5, 20.5, 0.5, 0.5, 40)), 3, landChart()));
      TearDown();
      EXPECT_TRUE(drive(inflation, withStop(line(20.5, 20.5, 1.5, 0.5, 30)), 3, landChart()));
      TearDown();
      // A diagonal that reverses on both axes and runs back past its start
      // (1 m per cycle back: 0.5 m steps west or south would reach an offset
      // the two size formulas truncate differently).
      EXPECT_TRUE(
        drive(
          inflation,
          withStop(join(line(20.5, 20.5, 1.5, 0.5, 12), line(37.5, 25.5, -1.0, -1.0, 22))),
          3, landChart()));
    });
}

// The same slow diagonal over scattered land and water with unknown space
// tracked, as on a global costmap: unpainted cells read unknown, and every
// painted value is compared (with no inflation layer).
TEST_F(MovingWindowTest, SlowDiagonalTrackingUnknownMatchesAFreshWindow)
{
  forEachInflation([this](Inflation inflation) {
      EXPECT_TRUE(
        drive(
          inflation, withStop(line(20.5, 20.5, 1.5, 0.5, 30)), 3, scatteredChart(), true, true));
    });
}

// The cost of the strips: the bounds the costmap repaints are never the
// whole window, on steady diagonal travel, on a reversal or while station
// keeping. The inflation layer merges this cycle's bounds with last cycle's
// (chooseStrip), so these guard the gap the layer leaves between an x and a
// y strip, and between strips on opposite sides of the window on one axis,
// and that a span is clipped to the window before a move's cells are added
// to it (recordUncovered). The first 2 cycles, when the whole window is
// painted, are left out. getBounds keeps the last cycle's bounds through a
// cycle whose bounds are empty (on Jazzy and Rolling alike), so a layer
// after the others records whether there were any. Returns the cycles that
// repainted the whole window, or a failure if the path's window origin is
// not the same under Jazzy's and Rolling's size formulas at every step.
class WholeWindowTest : public MovingWindowTest
{
protected:
  ::testing::AssertionResult wholeWindowCycles(
    Inflation inflation, const std::vector<Step> & path, std::string & whole)
  {
    makeNode(inflation);
    window_ = build(inflation, true, false, landChart());
    auto last = std::make_shared<BoundsLayer>(0.0);
    window_.costmap->addPlugin(last);
    last->initialize(window_.costmap.get(), "last", tf_buffer_.get(), node_, nullptr);
    auto * master = window_.costmap->getCostmap();
    double rolling_x = 0.0, rolling_y = 0.0, jazzy_x = 0.0, jazzy_y = 0.0;
    whole.clear();
    for (std::size_t k = 0; k < path.size(); k++) {
      window_.costmap->updateMap(path[k].x, path[k].y, 0.0);
      rolling_x = nextOrigin(rolling_x, path[k].x, kCells);
      rolling_y = nextOrigin(rolling_y, path[k].y, kCells);
      jazzy_x = nextOrigin(jazzy_x, path[k].x, kCells - 0.5);
      jazzy_y = nextOrigin(jazzy_y, path[k].y, kCells - 0.5);
      if (rolling_x != jazzy_x || rolling_y != jazzy_y || master->getOriginX() != rolling_x ||
        master->getOriginY() != rolling_y)
      {
        return ::testing::AssertionFailure() << "cycle " << k << ", robot at (" << path[k].x <<
               ", " << path[k].y << "): window origin (" << master->getOriginX() << ", " <<
               master->getOriginY() << "), Rolling's formula gives (" << rolling_x << ", " <<
               rolling_y << "), Jazzy's (" << jazzy_x << ", " << jazzy_y << ")";
      }
      const bool any = last->given[0] < last->given[2] && last->given[1] < last->given[3];
      unsigned int x0, xn, y0, yn;
      window_.costmap->getBounds(&x0, &xn, &y0, &yn);
      if (k >= 2 && any && x0 == 0 && y0 == 0 && xn == kCells && yn == kCells) {
        whole += " " + std::to_string(k);
      }
    }
    return ::testing::AssertionSuccess();
  }

  void expectNoWholeWindow(const std::vector<Step> & path)
  {
    forEachInflation([this, &path](Inflation inflation) {
        std::string whole;
        ASSERT_TRUE(wholeWindowCycles(inflation, path, whole));
        EXPECT_TRUE(whole.empty()) << "these cycles repainted the whole window:" << whole;
      });
  }
};

TEST_F(WholeWindowTest, DiagonalTravelNeverRepaintsTheWholeWindow)
{
  for (const auto & velocity : std::vector<std::pair<double, double>>{{0.5, 0.5}, {1.5, 0.5},
      {1.0, 1.0}})
  {
    SCOPED_TRACE(
      "velocity (" + std::to_string(velocity.first) + ", " + std::to_string(velocity.second) +
      ") m per cycle");
    expectNoWholeWindow(line(20.5, 20.5, velocity.first, velocity.second, 40));
  }
}

// East then west, north then south, and north-east then, after a stop of 2
// cycles, south-west, at 1 m per cycle. East and north turn back with a
// first step of 2 m, so that the window moves back in the very next cycle
// (from x.5, a step of 1 m back truncates to no move, a free gap).
TEST_F(WholeWindowTest, ReversalNeverRepaintsTheWholeWindow)
{
  {
    SCOPED_TRACE("east then west");
    expectNoWholeWindow(join(line(20.5, 20.5, 1.0, 0.0, 8), line(25.5, 20.5, -1.0, 0.0, 12)));
  }
  {
    SCOPED_TRACE("north then south");
    expectNoWholeWindow(join(line(20.5, 20.5, 0.0, 1.0, 8), line(20.5, 25.5, 0.0, -1.0, 12)));
  }
  {
    SCOPED_TRACE("north-east, a stop, then south-west");
    expectNoWholeWindow(
      join(line(20.5, 20.5, 1.0, 1.0, 8), line(27.5, 27.5, -1.0, -1.0, 12)));
  }
}

// The one case the strips cannot serve without the whole window: diagonal
// travel turned back on both axes at once. Cells on all four sides are then
// waiting, and two of them would wait 4 cycles, so chooseStrip reports the
// whole window once (the inflation layer's merge repaints it in that cycle
// and the next). This pins that cost at one report.
TEST_F(WholeWindowTest, SharpDiagonalReversalRepaintsTheWholeWindowOnce)
{
  forEachInflation([this](Inflation inflation) {
      std::string whole;
      ASSERT_TRUE(
        wholeWindowCycles(
          inflation, join(line(20.5, 20.5, 1.0, 1.0, 8), line(26.5, 26.5, -1.0, -1.0, 12)),
          whole));
      EXPECT_EQ(whole, inflation == Inflation::none ? " 11" : " 11 12");
    });
}

// Station keeping: the position jitters around (30.5, 30.5) by whole
// metres, a fixed pseudo-random sequence on each axis, at x.5 so that the
// window origin is the same under both size formulas. spread is the
// largest offset in cells.
std::vector<Step> stationKeeping(int spread, int cycles)
{
  std::vector<Step> path;
  unsigned int state = 12345u;
  auto jitter = [&state, spread]() {
      state = state * 1103515245u + 12345u;
      return static_cast<double>((state >> 16) % (2 * spread + 1)) - spread;
    };
  for (int k = 0; k < cycles; k++) {
    const double dx = jitter();
    const double dy = jitter();
    path.push_back({30.5 + dx, 30.5 + dy});
  }
  return path;
}

// The window origins a path takes on one axis from cycle 2 on (the cycles
// wholeWindowCycles counts), by Rolling's size formula (wholeWindowCycles
// asserts Jazzy's gives the same).
std::set<double> originsFromCycle2(const std::vector<Step> & path, bool x)
{
  std::set<double> origins;
  double origin = 0.0;
  for (std::size_t k = 0; k < path.size(); k++) {
    origin = nextOrigin(origin, x ? path[k].x : path[k].y, kCells);
    if (k >= 2) {
      origins.insert(origin);
    }
  }
  return origins;
}

// Jitter of up to one cell per update on each axis keeps the window on two
// origins per axis, and the strips never report the whole window (#57).
// This pins that side of the rule; the next case pins the other.
TEST_F(WholeWindowTest, StationKeepingWithinOneCellNeverRepaintsTheWholeWindow)
{
  const auto path = stationKeeping(1, 80);
  EXPECT_EQ(originsFromCycle2(path, true).size(), 2u);
  EXPECT_EQ(originsFromCycle2(path, false).size(), 2u);
  expectNoWholeWindow(path);
}

// Jitter of up to two cells per update moves the window over four origins
// per axis, so cells can wait on a third and fourth side, and the whole
// window is reported whenever a second side's cells have waited 2 updates
// (chooseStrip). This pins the cycles that rule gives for this sequence
// (an integer model of recordUncovered and chooseStrip predicts the same
// four, 7, 26, 46 and 72); with an inflation layer its merge repaints the
// whole window in the next cycle as well. A change that makes station
// keeping with this much jitter cheaper or dearer fails here.
TEST_F(WholeWindowTest, StationKeepingWithinTwoCellsRepaintsTheWholeWindowAtThePredictedCycles)
{
  const auto path = stationKeeping(2, 80);
  EXPECT_EQ(originsFromCycle2(path, true).size(), 4u);
  EXPECT_EQ(originsFromCycle2(path, false).size(), 4u);
  forEachInflation([this, &path](Inflation inflation) {
      std::string whole;
      ASSERT_TRUE(wholeWindowCycles(inflation, path, whole));
      EXPECT_EQ(whole, inflation == Inflation::none ? " 7 26 46 72" : " 7 8 26 27 46 47 72 73");
    });
}

// Control: a window that does not move, with the robot moving inside it.
// Nothing is uncovered, so this passes with and without the fix; it shows
// the strips leave a fixed window as it was.
TEST_F(MovingWindowTest, FixedWindowControl)
{
  forEachInflation([this](Inflation inflation) {
      EXPECT_TRUE(drive(inflation, withStop(line(5.5, 5.5, 1.0, 0.5, 10)), 0, landChart(), false));
    });
}

}  // namespace

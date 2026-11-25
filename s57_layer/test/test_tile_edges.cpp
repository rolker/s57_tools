// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

// generateTile with chart cell edges on tile and tile-cell edges
// (rolker/s57_tools#63). Each chart cell paints the tile cells it overlaps
// with positive area; one that only touches a tile cell's edge paints
// nothing, and no access falls outside a tile. Every cell of the window is
// checked against an expectation computed from the chart alone, and every
// getCost and setCost call generateTile makes on a tile is bounds-checked
// (see the definitions of Costmap2D::getCost and setCost below).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
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

// What the checked accessors saw since the last reset().
struct AccessLog
{
  std::uint64_t tile_calls = 0;  // calls on costmaps of the tile's size
  std::uint64_t tile_sets = 0;   // setCost calls among them
  std::uint64_t all_calls = 0;
  std::uint64_t out_of_range = 0;
  std::string first_out_of_range;
  unsigned int tile_size = 0;

  void reset(unsigned int size)
  {
    *this = AccessLog();
    tile_size = size;
  }
};

AccessLog access_log;

// Counts one access to (mx, my) in a size_x x size_y costmap; returns
// whether its linear index is inside the buffer. An index past the end of a
// row but inside the buffer (the east-edge wrap) is counted as out of range
// and still made, so it also shows by value.
bool checkAccess(
  unsigned int size_x, unsigned int size_y, unsigned int mx, unsigned int my, bool set)
{
  access_log.all_calls++;
  if (size_x == access_log.tile_size && size_y == access_log.tile_size) {
    access_log.tile_calls++;
    if (set) {
      access_log.tile_sets++;
    }
  }
  if (mx >= size_x || my >= size_y) {
    if (access_log.out_of_range == 0) {
      std::ostringstream first;
      first << (set ? "setCost" : "getCost") << "(" << mx << ", " << my << ") in a " <<
        size_x << " x " << size_y << " costmap";
      access_log.first_out_of_range = first.str();
    }
    access_log.out_of_range++;
  }
  return static_cast<std::uint64_t>(my) * size_x + mx <
         static_cast<std::uint64_t>(size_x) * size_y;
}

}  // namespace

// Bounds-checking definitions of two functions nav2_costmap_2d_core declares
// in costmap_2d.hpp and defines out of line. By ELF symbol interposition the
// dynamic linker binds every call made through the PLT, the layer's and
// possibly Nav2's own, to these definitions in the executable, ahead of
// Nav2's. The bodies are Nav2's, plus the check; an access past the buffer
// is skipped (a read returns NO_INFORMATION). Should interposition ever stop
// taking effect, each test fails loudly: it asserts that the calls on tiles
// reached here at least once per painted cell.
namespace nav2_costmap_2d
{

unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const
{
  if (!checkAccess(size_x_, size_y_, mx, my, false)) {
    return NO_INFORMATION;
  }
  return costmap_[getIndex(mx, my)];
}

void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
  if (!checkAccess(size_x_, size_y_, mx, my, true)) {
    return;
  }
  costmap_[getIndex(mx, my)] = cost;
}

}  // namespace nav2_costmap_2d

namespace
{

// Gives the layer a chart directly, as getDatasetsCallback and
// mapGridCallback would (as in test_rolling_window.cpp).
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

constexpr unsigned int kCells = 12;     // the window, in cells, on each axis
constexpr unsigned int kTileSize = 4;   // tile size, in cells
constexpr int kOriginX = -8;            // window origin, in cells
constexpr int kOriginY = 4;

// Chart cell (m, n), counted from the chart's west and south edges: no data
// (about 20%), land or 10 m of water, by a hash of the index.
enum class Charted { none, land, water };

Charted chartedAt(int m, int n)
{
  std::uint32_t h = (static_cast<std::uint32_t>(m) * 73856093u) ^
    (static_cast<std::uint32_t>(n) * 19349663u);
  h ^= h >> 13;
  h *= 0x5bd1e995u;
  h ^= h >> 15;
  if (h % 5 == 0) {
    return Charted::none;
  }
  return ((h >> 8) % 2) ? Charted::land : Charted::water;
}

unsigned char costOf(Charted charted)
{
  switch (charted) {
    case Charted::land: return nav2_costmap_2d::LETHAL_OBSTACLE;
    case Charted::water: return nav2_costmap_2d::FREE_SPACE;
    default: return nav2_costmap_2d::NO_INFORMATION;
  }
}

// A chart and window in quarter cells, so every edge is an integer. The
// chart's west and south edges lie west_q and south_q quarter cells from the
// window origin, its cells are cell_q quarter cells on a side.
struct Geometry
{
  double resolution;  // window cell, metres
  double origin_x;    // window origin, metres
  double origin_y;
  int cell_q;
  int west_q;
  int south_q;

  // Chart cells per side: from its west/south edge to 3 window cells past
  // the window's east/north edge, rounded up.
  int chartCells() const
  {
    const int start = std::min(west_q, south_q);
    const int end = 4 * (static_cast<int>(kCells) + 3);
    return (end - start + cell_q - 1) / cell_q;
  }
  double chartResolution() const {return cell_q * resolution / 4.0;}
  double chartWest() const {return origin_x + west_q * resolution / 4.0;}
  double chartSouth() const {return origin_y + south_q * resolution / 4.0;}
};

// The chart grid with every channel the layer reads, all NaN but elevation,
// which follows chartedAt by the cell's index counted from the chart's own
// west and south edges.
std::shared_ptr<grid_map::GridMap> makeChart(const Geometry & g)
{
  auto grid = std::make_shared<grid_map::GridMap>(
    std::vector<std::string>{"elevation", "overhead", "restricted", "unsurveyed", "caution"});
  const double chart_resolution = g.chartResolution();
  const double length = g.chartCells() * chart_resolution;
  grid->setGeometry(
    grid_map::Length(length, length), chart_resolution,
    grid_map::Position(g.chartWest() + length / 2.0, g.chartSouth() + length / 2.0));
  for (const auto & name : grid->getLayers()) {
    (*grid)[name].setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  for (grid_map::GridMapIterator it(*grid); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    grid->getPosition(*it, position);
    // Cell centres lie half a cell from any edge, so rounding cannot move
    // them to a neighbouring index.
    const int m = static_cast<int>(std::floor((position.x() - g.chartWest()) / chart_resolution));
    const int n = static_cast<int>(std::floor((position.y() - g.chartSouth()) / chart_resolution));
    switch (chartedAt(m, n)) {
      case Charted::land: grid->at("elevation", *it) = 1.0f; break;
      case Charted::water: grid->at("elevation", *it) = -10.0f; break;
      default: break;
    }
  }
  return grid;
}

// What window cell (i, j) should hold, from the chart alone, in integer
// quarter cells; never with the layer's (edge - tile origin) / resolution
// quotient, so a mistake there cannot pass on both sides. The chart cell
// that overlaps the cell with positive area and has data, east-most and
// then north-most (SubmapIterator's order; the first writer wins), or
// NO_INFORMATION if there is none.
unsigned char expectedCost(const Geometry & g, unsigned int i, unsigned int j)
{
  const int cells = g.chartCells();
  for (int m = cells - 1; m >= 0; m--) {
    const int west = g.west_q + m * g.cell_q;
    if (!(west < 4 * static_cast<int>(i) + 4 && west + g.cell_q > 4 * static_cast<int>(i))) {
      continue;
    }
    for (int n = cells - 1; n >= 0; n--) {
      const int south = g.south_q + n * g.cell_q;
      if (!(south < 4 * static_cast<int>(j) + 4 && south + g.cell_q > 4 * static_cast<int>(j))) {
        continue;
      }
      const auto charted = chartedAt(m, n);
      if (charted != Charted::none) {
        return costOf(charted);
      }
    }
  }
  return nav2_costmap_2d::NO_INFORMATION;
}

class TileEdgesTest : public ::testing::Test
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

  void TearDown() override
  {
    layered_costmap_.reset();
    layer_.reset();
    node_.reset();
  }

  // A fixed (not rolling) 12 x 12 window that tracks unknown, so a cell the
  // layer does not paint reads NO_INFORMATION; tiles of 4 cells starting at
  // the window origin; the chart; one update. The access log covers the
  // update only.
  void paint(const Geometry & g)
  {
    node_ = std::make_shared<TestLifecycleNode>("test_tile_edges");
    node_->declare_parameter("chart_layer.tile_size", static_cast<int>(kTileSize));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, true);
    layered_costmap_->resizeMap(kCells, kCells, g.resolution, g.origin_x, g.origin_y);
    layer_ = std::make_shared<S57LayerForTest>();
    layered_costmap_->addPlugin(layer_);
    layer_->initialize(layered_costmap_.get(), "chart_layer", tf_buffer_.get(), node_, nullptr);
    layer_->addChart("chart", makeChart(g));

    access_log.reset(kTileSize);
    layered_costmap_->updateMap(
      g.origin_x + kCells * g.resolution / 2.0, g.origin_y + kCells * g.resolution / 2.0, 0.0);
    log_ = access_log;
  }

  // No tile access out of range, and the tile accesses reached the checked
  // definitions: at least one setCost on a tile per painted window cell.
  ::testing::AssertionResult accessesInRange()
  {
    auto * master = layered_costmap_->getCostmap();
    std::uint64_t painted = 0;
    for (unsigned int j = 0; j < kCells; j++) {
      for (unsigned int i = 0; i < kCells; i++) {
        if (master->getCharMap()[master->getIndex(i, j)] != nav2_costmap_2d::NO_INFORMATION) {
          painted++;
        }
      }
    }
    if (painted == 0 || log_.tile_sets < painted) {
      return ::testing::AssertionFailure() <<
             "the checked accessors saw " << log_.tile_sets << " setCost calls on " <<
             kTileSize << " x " << kTileSize << " tiles for " << painted <<
             " painted cells: the layer's tile accesses are not reaching them";
    }
    if (log_.out_of_range != 0) {
      return ::testing::AssertionFailure() <<
             log_.out_of_range << " of " << log_.tile_calls <<
             " tile accesses out of range; first " << log_.first_out_of_range;
    }
    return ::testing::AssertionSuccess();
  }

  // Every window cell equals expectedCost.
  ::testing::AssertionResult cellsAsCharted(const Geometry & g)
  {
    auto * master = layered_costmap_->getCostmap();
    if (master->getOriginX() != g.origin_x || master->getOriginY() != g.origin_y) {
      return ::testing::AssertionFailure() << "window origin is (" << master->getOriginX() <<
             ", " << master->getOriginY() << "), not (" << g.origin_x << ", " << g.origin_y << ")";
    }
    unsigned int wrong = 0;
    std::ostringstream first;
    for (unsigned int j = 0; j < kCells; j++) {
      for (unsigned int i = 0; i < kCells; i++) {
        const int cost = master->getCharMap()[master->getIndex(i, j)];
        const int expected = expectedCost(g, i, j);
        if (cost != expected) {
          if (wrong == 0) {
            first << "first (" << i << ", " << j << ") is " << cost << ", expected " << expected;
          }
          wrong++;
        }
      }
    }
    if (wrong == 0) {
      return ::testing::AssertionSuccess();
    }
    return ::testing::AssertionFailure() <<
           wrong << " of " << kCells * kCells << " cells differ from the expectation; " <<
           first.str();
  }

  // The binary-exact cases: window origin (-8, 4) cells.
  void checkExact(double resolution, int cell_q, int west_q, int south_q)
  {
    const Geometry g{resolution, kOriginX * resolution, kOriginY * resolution,
      cell_q, west_q, south_q};
    paint(g);
    EXPECT_TRUE(accessesInRange());
    EXPECT_TRUE(cellsAsCharted(g));
  }

  // At 0.1 m no edge can be placed exactly: access only.
  void checkAccessOnly(int cell_q)
  {
    const Geometry g{0.1, -12.3, 4.7, cell_q, -16, -16};
    paint(g);
    EXPECT_TRUE(accessesInRange());
  }

  std::shared_ptr<TestLifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layered_costmap_;
  std::shared_ptr<S57LayerForTest> layer_;
  AccessLog log_;
};

// Chart cells the size of a window cell, starting 3 cells west and south of
// the window (-12 quarter cells): every chart cell edge on that axis lies
// on a tile-cell edge, the tiles' east (or north) edges among them. A
// quarter cell more (-11) keeps that axis off every edge.

TEST_F(TileEdgesTest, EastTileEdges1m) {checkExact(1.0, 4, -12, -11);}
TEST_F(TileEdgesTest, EastTileEdgesHalfMetre) {checkExact(0.5, 4, -12, -11);}
TEST_F(TileEdgesTest, NorthTileEdges1m) {checkExact(1.0, 4, -11, -12);}
TEST_F(TileEdgesTest, NorthTileEdgesHalfMetre) {checkExact(0.5, 4, -11, -12);}
TEST_F(TileEdgesTest, EastAndNorthTileEdges1m) {checkExact(1.0, 4, -12, -12);}
TEST_F(TileEdgesTest, EastAndNorthTileEdgesHalfMetre) {checkExact(0.5, 4, -12, -12);}

// Chart cells of two window cells, 3 cells west and south of the window:
// their edges lie on odd cell edges, interior tile-cell edges, never on a
// tile's edge.
TEST_F(TileEdgesTest, InteriorEdgesOnly1m) {checkExact(1.0, 8, -12, -12);}
TEST_F(TileEdgesTest, InteriorEdgesOnlyHalfMetre) {checkExact(0.5, 8, -12, -12);}

// Control: no chart cell edge on any tile-cell edge.
TEST_F(TileEdgesTest, QuarterCellOffControl1m) {checkExact(1.0, 4, -11, -11);}
TEST_F(TileEdgesTest, QuarterCellOffControlHalfMetre) {checkExact(0.5, 4, -11, -11);}

// 0.1 m, origin (-12.3, 4.7): chart cells of half, one and two window cells,
// starting 4 cells west and south of the window, so that each chart has cell
// edges on the tiles' edges, as nearly as doubles allow.
TEST_F(TileEdgesTest, TenthMetreHalfCellChart) {checkAccessOnly(2);}
TEST_F(TileEdgesTest, TenthMetreOneCellChart) {checkAccessOnly(4);}
TEST_F(TileEdgesTest, TenthMetreTwoCellChart) {checkAccessOnly(8);}

}  // namespace

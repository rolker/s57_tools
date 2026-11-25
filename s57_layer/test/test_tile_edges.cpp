// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

// generateTile with chart cell edges on tile and tile-cell edges
// (rolker/s57_tools#63). Each chart cell paints the tile cells it overlaps
// with positive area; one that only touches a tile cell's edge paints
// nothing, and no access falls outside a tile. Every cell of the window is
// checked against an expectation computed from the chart alone, and every
// getCost and setCost call generateTile makes on a tile is bounds-checked
// (see the definitions of Costmap2D::getCost and setCost below).
//
// Charts whose edges or seams lie inside a tile, wherever the tile's centre
// falls and wherever the tile lies relative to the map's x = 0 and y = 0
// lines (rolker/s57_tools#59): each chart contributes every cell that
// overlaps the tile, the finest chart first.

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
#include "grid_map_core/SubmapGeometry.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"
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
  std::uint64_t tile_out_of_range = 0;  // tile_calls out of range
  std::string first_tile_out_of_range;
  std::uint64_t other_calls = 0;  // calls on any other costmap
  std::uint64_t other_out_of_range = 0;
  std::string first_other_out_of_range;
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
  const bool tile = size_x == access_log.tile_size && size_y == access_log.tile_size;
  if (tile) {
    access_log.tile_calls++;
    if (set) {
      access_log.tile_sets++;
    }
  } else {
    access_log.other_calls++;
  }
  if (mx >= size_x || my >= size_y) {
    auto & count = tile ? access_log.tile_out_of_range : access_log.other_out_of_range;
    if (count == 0) {
      std::ostringstream first;
      first << (set ? "setCost" : "getCost") << "(" << mx << ", " << my << ") in a " <<
        size_x << " x " << size_y << " costmap";
      (tile ? access_log.first_tile_out_of_range : access_log.first_other_out_of_range) =
        first.str();
    }
    count++;
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
  // The chart's stated bounds (chart_bounds_, which getDatasetsCallback
  // takes from the dataset) are the grid's extent, shrunk on every side by
  // bounds_inset metres (grown, if negative).
  void addChart(
    const std::string & label, std::shared_ptr<grid_map::GridMap> grid, double bounds_inset = 0.0)
  {
    s57_msgs::msg::DatasetInfo info;
    info.label = label;
    current_charts_.push_back(info);

    const auto & position = grid->getPosition();
    const auto & length = grid->getLength();
    geometry_msgs::msg::Point min_point, max_point;
    min_point.x = position.x() - length.x() / 2.0 + bounds_inset;
    min_point.y = position.y() - length.y() / 2.0 + bounds_inset;
    max_point.x = position.x() + length.x() / 2.0 - bounds_inset;
    max_point.y = position.y() + length.y() / 2.0 - bounds_inset;
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
// window origin, its cells are cell_q quarter cells on a side. ulps_x and
// ulps_y move the chart's centre that many representable doubles east or
// north (west or south if negative) of where those integers put it.
struct Geometry
{
  double resolution;  // window cell, metres
  double origin_x;    // window origin, metres
  double origin_y;
  int cell_q;
  int west_q;
  int south_q;
  int ulps_x = 0;
  int ulps_y = 0;

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

double stepUlps(double value, int ulps)
{
  for (int k = 0; k < std::abs(ulps); k++) {
    value = std::nextafter(value, ulps > 0 ? HUGE_VAL : -HUGE_VAL);
  }
  return value;
}

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
    grid_map::Position(
      stepUlps(g.chartWest() + length / 2.0, g.ulps_x),
      stepUlps(g.chartSouth() + length / 2.0, g.ulps_y)));
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
// NO_INFORMATION if there is none. With chart cells smaller than a window
// cell, several overlap it: of those with data, the one with the greatest
// m, then the greatest n. A chart cell larger than a tile paints every cell
// of each tile it overlaps. ulps_x and ulps_y are ignored: the expectation
// is that of the exact quarter-cell geometry.
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

// The chart cells generateTile's SubmapIterator yields for tile (tx, ty)
// that lie wholly east of the tile (east == true) or wholly north of it, by
// their quarter-cell edges: a chart cell whose west (south) edge is on or
// past the tile's east (north) edge. The request here is the tile's own
// centre and size. generateTile sends exactly that only where the tile lies
// within the chart grid's cell centres on both axes (otherwise it clips the
// request), so this returns -1 if the tile does not, and the caller asserts
// that it does.
int chartCellsBeyondTile(
  const Geometry & g, const grid_map::GridMap & chart, int tx, int ty, bool east)
{
  const double tile_length = g.resolution * kTileSize;
  const double world_min_x = g.origin_x + tx * g.resolution * kTileSize;
  const double world_min_y = g.origin_y + ty * g.resolution * kTileSize;
  const double tile_lo[2] = {world_min_x, world_min_y};
  const double tile_hi[2] = {world_min_x + tile_length, world_min_y + tile_length};
  for (int a = 0; a < 2; a++) {
    const double half = chart.getResolution() / 2.0;
    const double grid_lo = chart.getPosition()[a] - chart.getLength()[a] / 2.0;
    const double grid_hi = chart.getPosition()[a] + chart.getLength()[a] / 2.0;
    if (tile_lo[a] < grid_lo + half || tile_hi[a] > grid_hi - half) {
      return -1;
    }
  }
  const grid_map::Position tile_center(
    world_min_x + g.resolution * kTileSize / 2.0, world_min_y + g.resolution * kTileSize / 2.0);
  bool ok = false;
  grid_map::SubmapGeometry overlap(
    chart, tile_center, grid_map::Length(tile_length, tile_length), ok);
  if (!ok) {
    return 0;
  }
  int beyond = 0;
  for (grid_map::SubmapIterator it(overlap); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    chart.getPosition(*it, position);
    const double chart_resolution = g.chartResolution();
    const int m = static_cast<int>(std::floor((position.x() - g.chartWest()) / chart_resolution));
    const int n = static_cast<int>(std::floor((position.y() - g.chartSouth()) / chart_resolution));
    const int near_q = east ? g.west_q + m * g.cell_q : g.south_q + n * g.cell_q;
    const int tile_far_q = 4 * static_cast<int>(kTileSize) * ((east ? tx : ty) + 1);
    if (near_q >= tile_far_q) {
      beyond++;
    }
  }
  return beyond;
}

// ---- rolker/s57_tools#59: several charts, any window, tile size and origin ----
// Every chart edge is an integer number of quarter window cells from the
// window origin, which is also the tiles' origin.
//
// A chart's stated bounds (chart_bounds_) are its grid's extent, inset by
// bounds_inset_q quarter window cells on every side (outset if negative).
// S57Dataset::getGrid makes the grid round(stated length / r) cells long,
// centred on the stated bounds, so the two differ by the same amount on
// both sides, less than a quarter chart cell: check(Scene) asserts that a
// spec is one getGrid could produce (4 |bounds_inset_q| < cell_q, strictly,
// so the rounding is not a tie).
enum class Fill { hashed, land, water };
struct ChartSpec
{
  std::string label;
  int cell_q, west_q, south_q, cells_x, cells_y;
  Fill fill = Fill::hashed;
  int seed = 0;
  int bounds_inset_q = 0;
};
struct Scene
{
  double resolution;
  int origin_x, origin_y;  // window origin, in window cells
  unsigned int cells;      // window, in cells, on each axis
  unsigned int tile;       // tile size, in cells
  std::vector<ChartSpec> charts;
};

Charted chartedAt(const ChartSpec & c, int m, int n)
{
  if (c.fill == Fill::land) {return Charted::land;}
  if (c.fill == Fill::water) {return Charted::water;}
  return chartedAt(m + 7919 * c.seed, n);
}

std::shared_ptr<grid_map::GridMap> makeChart(const Scene & s, const ChartSpec & c)
{
  auto grid = std::make_shared<grid_map::GridMap>(
    std::vector<std::string>{"elevation", "overhead", "restricted", "unsurveyed", "caution"});
  const double r = c.cell_q * s.resolution / 4.0;
  const double west = (s.origin_x + c.west_q / 4.0) * s.resolution;
  const double south = (s.origin_y + c.south_q / 4.0) * s.resolution;
  grid->setGeometry(grid_map::Length(c.cells_x * r, c.cells_y * r), r,
    grid_map::Position(west + c.cells_x * r / 2.0, south + c.cells_y * r / 2.0));
  for (const auto & name : grid->getLayers()) {
    (*grid)[name].setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  for (grid_map::GridMapIterator it(*grid); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    grid->getPosition(*it, position);
    const int m = static_cast<int>(std::floor((position.x() - west) / r));
    const int n = static_cast<int>(std::floor((position.y() - south) / r));
    switch (chartedAt(c, m, n)) {
      case Charted::land: grid->at("elevation", *it) = 1.0f; break;
      case Charted::water: grid->at("elevation", *it) = -10.0f; break;
      default: break;
    }
  }
  return grid;
}

// Window cell (i, j): the charts finest first (then by label, as the layer's
// map orders them); within a chart, the east-most then north-most cell that
// overlaps the window cell with positive area and has data. A chart counts
// only in the tiles generateTile's pre-filter passes: those its stated
// bounds overlap or touch (the pre-filter compares with > and <). That
// pins current behaviour where the grid reaches past the stated bounds and
// the pre-filter skips a tile the grid overlaps (GridLargerThanBounds
// PreFilterSkips1m); elsewhere it changes nothing, as a chart whose grid
// only touches a tile paints nothing in it.
unsigned char expectedCost(const Scene & s, unsigned int i, unsigned int j)
{
  std::vector<const ChartSpec *> order;
  for (const auto & c : s.charts) {order.push_back(&c);}
  std::sort(order.begin(), order.end(), [](const ChartSpec * a, const ChartSpec * b) {
      return std::make_pair(a->cell_q, a->label) < std::make_pair(b->cell_q, b->label);});
  const int x0 = 4 * static_cast<int>(i), y0 = 4 * static_cast<int>(j);
  const int tile_q = 4 * static_cast<int>(s.tile);
  const int tile_x0 = tile_q * static_cast<int>(i / s.tile);
  const int tile_y0 = tile_q * static_cast<int>(j / s.tile);
  for (const ChartSpec * c : order) {
    const int inset = c->bounds_inset_q;
    if (c->west_q + inset > tile_x0 + tile_q ||
      c->west_q + c->cells_x * c->cell_q - inset < tile_x0 ||
      c->south_q + inset > tile_y0 + tile_q ||
      c->south_q + c->cells_y * c->cell_q - inset < tile_y0)
    {
      continue;
    }
    for (int m = c->cells_x - 1; m >= 0; m--) {
      const int west = c->west_q + m * c->cell_q;
      if (!(west < x0 + 4 && west + c->cell_q > x0)) {continue;}
      for (int n = c->cells_y - 1; n >= 0; n--) {
        const int south = c->south_q + n * c->cell_q;
        if (!(south < y0 + 4 && south + c->cell_q > y0)) {continue;}
        const auto charted = chartedAt(*c, m, n);
        if (charted != Charted::none) {return costOf(charted);}
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

  // No tile access out of range, and (if require_reach) the tile accesses
  // reached the checked definitions: at least one setCost on a tile per
  // painted window cell, and some cell painted.
  ::testing::AssertionResult accessesInRange(bool require_reach = true)
  {
    auto * master = layered_costmap_->getCostmap();
    std::uint64_t painted = 0;
    const unsigned int cells = master->getSizeInCellsX();
    const unsigned int tile = log_.tile_size;
    for (unsigned int j = 0; j < cells; j++) {
      for (unsigned int i = 0; i < cells; i++) {
        if (master->getCharMap()[master->getIndex(i, j)] != nav2_costmap_2d::NO_INFORMATION) {
          painted++;
        }
      }
    }
    if (require_reach && (painted == 0 || log_.tile_sets < painted)) {
      return ::testing::AssertionFailure() <<
             "the checked accessors saw " << log_.tile_sets << " setCost calls on " <<
             tile << " x " << tile << " tiles for " << painted <<
             " painted cells: the layer's tile accesses are not reaching them";
    }
    if (log_.tile_out_of_range != 0) {
      return ::testing::AssertionFailure() <<
             log_.tile_out_of_range << " of " << log_.tile_calls <<
             " getCost/setCost calls on " << tile << " x " << tile <<
             " tiles out of range; first " << log_.first_tile_out_of_range;
    }
    if (log_.other_out_of_range != 0) {
      return ::testing::AssertionFailure() <<
             log_.other_out_of_range << " of " << log_.other_calls <<
             " getCost/setCost calls on costmaps other than tiles out of range; first " <<
             log_.first_other_out_of_range;
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

  // As checkExact, the chart's centre moved one double west (east == true)
  // or south of its exact place; first asserts that generateTile's
  // SubmapIterator then yields, for tile (tx, ty), chart cells wholly east
  // (north) of that tile, so the case cannot pass without reaching them.
  void checkOneUlpBeyond(
    double resolution, int west_q, int south_q, bool east, int tx, int ty)
  {
    Geometry g{resolution, kOriginX * resolution, kOriginY * resolution, 4, west_q, south_q};
    (east ? g.ulps_x : g.ulps_y) = -1;
    const int beyond = chartCellsBeyondTile(g, *makeChart(g), tx, ty, east);
    ASSERT_NE(beyond, -1) << "tile (" << tx << ", " << ty << ") is not within the chart grid's " <<
      "cell centres, so generateTile clips its request and the check below does not apply";
    ASSERT_GT(beyond, 0) <<
      "the iterator yields no chart cell wholly " << (east ? "east" : "north") <<
      " of tile (" << tx << ", " << ty << "): this case would not reach the near clamp";
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

  // As paint(Geometry) for a Scene: its window, tile size and charts.
  void paint(const Scene & s)
  {
    node_ = std::make_shared<TestLifecycleNode>("test_tile_edges");
    node_->declare_parameter("chart_layer.tile_size", static_cast<int>(s.tile));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, true);
    layered_costmap_->resizeMap(s.cells, s.cells, s.resolution,
      s.origin_x * s.resolution, s.origin_y * s.resolution);
    layer_ = std::make_shared<S57LayerForTest>();
    layered_costmap_->addPlugin(layer_);
    layer_->initialize(layered_costmap_.get(), "chart_layer", tf_buffer_.get(), node_, nullptr);
    for (const auto & c : s.charts) {
      layer_->addChart(c.label, makeChart(s, c), c.bounds_inset_q * s.resolution / 4.0);
    }
    access_log.reset(s.tile);
    layered_costmap_->updateMap(
      (s.origin_x + s.cells / 2.0) * s.resolution, (s.origin_y + s.cells / 2.0) * s.resolution, 0.0);
    log_ = access_log;
  }

  // Every window cell equals expectedCost(Scene).
  ::testing::AssertionResult cellsAsCharted(const Scene & s)
  {
    auto * master = layered_costmap_->getCostmap();
    if (master->getOriginX() != s.origin_x * s.resolution ||
      master->getOriginY() != s.origin_y * s.resolution)
    {
      return ::testing::AssertionFailure() << "window origin is (" << master->getOriginX() <<
             ", " << master->getOriginY() << "), not the scene's";
    }
    unsigned int wrong = 0;
    std::ostringstream first;
    for (unsigned int j = 0; j < s.cells; j++) {
      for (unsigned int i = 0; i < s.cells; i++) {
        const int cost = master->getCharMap()[master->getIndex(i, j)];
        const int expected = expectedCost(s, i, j);
        if (cost != expected) {
          if (wrong == 0) {
            first << "first (" << i << ", " << j << ") is " << cost << ", expected " << expected;
          }
          wrong++;
        }
      }
    }
    if (wrong == 0) {return ::testing::AssertionSuccess();}
    return ::testing::AssertionFailure() << wrong << " of " << s.cells * s.cells <<
           " cells differ from the expectation; " << first.str();
  }

  // The cells first: where they are wrong (as where a chart is dropped and
  // nothing is painted), that is the failure reported, and the accesses are
  // checked for range only; where they are right, the accessors must also
  // have seen the painting.
  void check(const Scene & s)
  {
    for (const auto & c : s.charts) {
      ASSERT_LT(4 * std::abs(c.bounds_inset_q), c.cell_q) << "chart " << c.label <<
        ": getGrid cannot make a grid that far from its stated bounds";
    }
    paint(s);
    const auto cells = cellsAsCharted(s);
    EXPECT_TRUE(cells);
    EXPECT_TRUE(accessesInRange(static_cast<bool>(cells)));
  }

  // A chart grid one cell long on axis short_axis (0: x, 1: y), the cell
  // 3.75 m on a side centred at -7.97 m on that axis; four cells from
  // -12.6 m on the other. The window: 12 x 12 cells of 1 m from (-12, -12) m,
  // tiles of 4 cells; one update. Asserts first that the grid's west and east
  // (south and north) cell centres, computed as generateTile computes them,
  // cross; then every window cell and the accesses. In units of 5 mm every
  // edge is an integer: the cell spans -1969 to -1219 on the short axis, the
  // other cells' edges are -2520 + 750 n, window cell k spans -2400 + 200 k
  // to -2200 + 200 k.
  void checkOneCellAxis(int short_axis)
  {
    const double r = 3.75;
    auto grid = std::make_shared<grid_map::GridMap>(
      std::vector<std::string>{"elevation", "overhead", "restricted", "unsurveyed", "caution"});
    grid->setGeometry(
      short_axis == 0 ? grid_map::Length(r, 4 * r) : grid_map::Length(4 * r, r), r,
      short_axis == 0 ? grid_map::Position(-7.97, -5.1) : grid_map::Position(-5.1, -7.97));
    ASSERT_EQ(grid->getSize()[short_axis], 1);
    ASSERT_EQ(grid->getSize()[1 - short_axis], 4);
    const double half = grid->getResolution() / 2.0;
    const double grid_lo = grid->getPosition()[short_axis] - grid->getLength()[short_axis] / 2.0;
    const double grid_hi = grid->getPosition()[short_axis] + grid->getLength()[short_axis] / 2.0;
    ASSERT_GT(grid_lo + half, grid_hi - half) <<
      "the cell centres do not cross here, so this case would not reach the crossed bounds";

    for (const auto & name : grid->getLayers()) {
      (*grid)[name].setConstant(std::numeric_limits<float>::quiet_NaN());
    }
    // Long-axis cell n, counted from the south (west) edge, holds chartedAt(0, n).
    for (grid_map::GridMapIterator it(*grid); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      grid->getPosition(*it, position);
      const int n = static_cast<int>(std::floor((position[1 - short_axis] + 12.6) / r));
      switch (chartedAt(0, n)) {
        case Charted::land: grid->at("elevation", *it) = 1.0f; break;
        case Charted::water: grid->at("elevation", *it) = -10.0f; break;
        default: break;
      }
    }

    node_ = std::make_shared<TestLifecycleNode>("test_tile_edges");
    node_->declare_parameter("chart_layer.tile_size", 4);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, true);
    layered_costmap_->resizeMap(12, 12, 1.0, -12.0, -12.0);
    layer_ = std::make_shared<S57LayerForTest>();
    layered_costmap_->addPlugin(layer_);
    layer_->initialize(layered_costmap_.get(), "chart_layer", tf_buffer_.get(), node_, nullptr);
    layer_->addChart("c", grid);
    access_log.reset(4);
    layered_costmap_->updateMap(-6.0, -6.0, 0.0);
    log_ = access_log;

    auto * master = layered_costmap_->getCostmap();
    unsigned int wrong = 0;
    std::ostringstream first;
    for (unsigned int j = 0; j < 12; j++) {
      for (unsigned int i = 0; i < 12; i++) {
        const int s = -2400 + 200 * static_cast<int>(short_axis == 0 ? i : j);
        const int l = -2400 + 200 * static_cast<int>(short_axis == 0 ? j : i);
        unsigned char expected = nav2_costmap_2d::NO_INFORMATION;
        if (s < -1219 && s + 200 > -1969) {
          // The cell with data that is north-most (east-most) of those overlapping.
          for (int n = 3; n >= 0; n--) {
            const int south = -2520 + 750 * n;
            if (south < l + 200 && south + 750 > l && chartedAt(0, n) != Charted::none) {
              expected = costOf(chartedAt(0, n));
              break;
            }
          }
        }
        const int cost = master->getCharMap()[master->getIndex(i, j)];
        if (cost != expected) {
          if (wrong == 0) {
            first << "first (" << i << ", " << j << ") is " << cost << ", expected " <<
              static_cast<int>(expected);
          }
          wrong++;
        }
      }
    }
    EXPECT_EQ(wrong, 0u) << first.str();
    EXPECT_TRUE(accessesInRange(wrong == 0));
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

// Chart cells of half a window cell, two per window cell on each axis. A
// window cell takes, of the four chart cells over it, the east one with
// data, and of the two east or two west ones the north one with data
// (expectedCost). From 3 cells west and south of the window (-12 quarter
// cells) a chart cell edge lies on every tile-cell edge, the tiles' edges
// among them; a quarter cell more (-11) keeps every edge off them (a guard:
// the pre-fix code also passes it).
TEST_F(TileEdgesTest, HalfCellChartOnEdges1m) {checkExact(1.0, 2, -12, -12);}
TEST_F(TileEdgesTest, HalfCellChartOnEdgesHalfMetre) {checkExact(0.5, 2, -12, -12);}
TEST_F(TileEdgesTest, HalfCellChartOffEdgesGuard1m) {checkExact(1.0, 2, -11, -11);}

// Chart cells of 8 window cells, two tiles across. From 4 cells west and
// south of the window (-16 quarter cells) their edges lie at -4, 4 and 12
// cells: on tile edges only, the window's east and north edges among them.
// From 3 cells (-12) they lie at -3, 5 and 13: on interior tile-cell edges,
// a guard: the pre-fix code also passes it, as with this chart's data each
// tile cell it spread a chart cell into already held its own chart cell's
// value.
TEST_F(TileEdgesTest, LargeCellChartOnTileEdges1m) {checkExact(1.0, 32, -16, -16);}
TEST_F(TileEdgesTest, LargeCellChartOnTileEdgesHalfMetre) {checkExact(0.5, 32, -16, -16);}
TEST_F(TileEdgesTest, LargeCellChartInteriorEdgesGuard1m) {checkExact(1.0, 32, -12, -12);}

// The near index's clamp to tile_size_: a chart cell wholly east (north) of
// a tile reaches the loop only when grid_map's submap corner lookup rounds
// across the tile's edge, never with exact geometry. The East and North
// cases above, with the chart moved one double west (south): grid_map then
// yields chart cells wholly east (north) of some tiles, among them the tile
// each case names, while the layer's own arithmetic puts their near edge
// exactly on the tile's edge; they must paint nothing. (The far index's
// clamp to -1 needs no such case: at a tile's west or south corner an exact
// coincidence already yields the chart cell beyond the edge, and with that
// clamp raised to 0 the EastAndNorth cases fail on either axis, and the
// North and HalfCellChartOnEdges cases on y.)
TEST_F(TileEdgesTest, EastTileEdgeOneUlpWest1m) {checkOneUlpBeyond(1.0, -12, -11, true, 2, 0);}
TEST_F(TileEdgesTest, EastTileEdgeOneUlpWestHalfMetre)
{
  checkOneUlpBeyond(0.5, -12, -11, true, 2, 0);
}
TEST_F(TileEdgesTest, NorthTileEdgeOneUlpSouth1m) {checkOneUlpBeyond(1.0, -11, -12, false, 0, 0);}
TEST_F(TileEdgesTest, NorthTileEdgeOneUlpSouthHalfMetre)
{
  checkOneUlpBeyond(0.5, -11, -12, false, 0, 0);
}

// Control: no chart cell edge on any tile-cell edge.
TEST_F(TileEdgesTest, QuarterCellOffControl1m) {checkExact(1.0, 4, -11, -11);}
TEST_F(TileEdgesTest, QuarterCellOffControlHalfMetre) {checkExact(0.5, 4, -11, -11);}

// 0.1 m, origin (-12.3, 4.7): chart cells of half, one and two window cells,
// starting 4 cells west and south of the window, so that each chart has cell
// edges on the tiles' edges, as nearly as doubles allow.
TEST_F(TileEdgesTest, TenthMetreHalfCellChart) {checkAccessOnly(2);}
TEST_F(TileEdgesTest, TenthMetreOneCellChart) {checkAccessOnly(4);}
TEST_F(TileEdgesTest, TenthMetreTwoCellChart) {checkAccessOnly(8);}

// ---- rolker/s57_tools#59 ----
// Window as above (12 x 12 cells, 4-cell tiles, origin (-8, 4) cells) unless
// stated. Tile column 1 spans window quarters 16..32, its centre at 24.
Scene small(double r, std::vector<ChartSpec> charts, int ox = -8, int oy = 4)
{
  return Scene{r, ox, oy, 12, 4, std::move(charts)};
}

// One chart over part of tile column / row 1 with the tile's centre outside it.
TEST_F(TileEdgesTest, CentreOutsideChartWest1m) {check(small(1.0, {{"c", 4, -11, -12, 8, 15}}));}
TEST_F(TileEdgesTest, CentreOutsideChartWestHalfMetre) {check(small(0.5, {{"c", 4, -11, -12, 8, 15}}));}
TEST_F(TileEdgesTest, CentreOutsideChartEast1m) {check(small(1.0, {{"c", 4, 27, -12, 9, 15}}));}
TEST_F(TileEdgesTest, CentreOutsideChartSouth1m) {check(small(1.0, {{"c", 4, -12, -11, 15, 8}}));}
TEST_F(TileEdgesTest, CentreOutsideChartNorth1m) {check(small(1.0, {{"c", 4, -12, 27, 15, 9}}));}
TEST_F(TileEdgesTest, CentreOutsideChartCorner1m) {check(small(1.0, {{"c", 4, -11, -11, 8, 8}}));}
// Two charts of one scale meeting at a seam west of, on, and east of the
// centre of tile column 1, and on the centre of tile row 1.
TEST_F(TileEdgesTest, SeamWestOfCentre1m)
{
  check(small(1.0, {{"a", 4, -11, -12, 8, 15}, {"b", 4, 21, -12, 10, 15, Fill::hashed, 1}}));
}
TEST_F(TileEdgesTest, SeamOnCentre1m)
{
  check(small(1.0, {{"a", 4, -12, -12, 9, 15}, {"b", 4, 24, -12, 9, 15, Fill::hashed, 1}}));
}
TEST_F(TileEdgesTest, SeamEastOfCentre1m)
{
  check(small(1.0, {{"a", 4, -13, -12, 10, 15}, {"b", 4, 27, -12, 9, 15, Fill::hashed, 1}}));
}
TEST_F(TileEdgesTest, SeamOnCentreNorthHalfMetre)
{
  check(small(0.5, {{"a", 4, -12, -12, 15, 9}, {"b", 4, -12, 24, 15, 9, Fill::hashed, 1}}));
}
// A chart smaller than a tile, inside tile (1, 1), off its centre; guard: over it.
TEST_F(TileEdgesTest, SmallChartOffCentre1m) {check(small(1.0, {{"c", 2, 17, 25, 3, 3}}));}
TEST_F(TileEdgesTest, SmallChartOverCentreGuard1m) {check(small(1.0, {{"c", 2, 21, 21, 3, 3}}));}
// A fine all-water chart over the west 5 quarters of tile column 1, a coarse
// all-land chart under the whole window: the fine chart's water wins there.
TEST_F(TileEdgesTest, FineWaterOverCoarseLand1m)
{
  check(small(1.0, {{"fine", 4, -11, -12, 8, 15, Fill::water}, {"coarse", 32, -16, -16, 3, 3, Fill::land}}));
}
// The map-axis band. Tile column 2 starts at map x = 0 (origin -8 cells). A
// chart 64 m or more long whose west edge is 3 quarters inside that tile and
// which covers its centre.
TEST_F(TileEdgesTest, ChartEdgeInTileAtMapXZero1m) {check(small(1.0, {{"c", 4, 35, -12, 70, 15}}));}
TEST_F(TileEdgesTest, ChartEdgeInTileAtMapXZeroHalfMetre) {check(small(0.5, {{"c", 4, 35, -12, 140, 15}}));}
TEST_F(TileEdgesTest, ChartEdgeInTileAtMapYZero1m)
{
  check(small(1.0, {{"c", 4, -12, 35, 15, 70}}, -8, -8));
}
// The same at 100-cell tiles: 200 x 200 window from (-100, -100) cells, tile
// (1, 1) from map (0, 0); the chart's west and south edges 36.25 cells in.
TEST_F(TileEdgesTest, ChartEdgeInTileAtMapOrigin100CellTiles1m)
{
  check(Scene{1.0, -100, -100, 200, 100, {{"c", 4, 545, 545, 67, 67}}});
}
// A chart whose own west edge is 0.75 m west of map x = 0, the tile from -4 m
// to 0 (its centre outside the chart). Clipping the request to the chart's
// edge (the issue's first proposal) loses grid_map's epsilon there.
TEST_F(TileEdgesTest, ChartEdgeJustWestOfMapXZero1m) {check(small(1.0, {{"c", 4, 29, -12, 70, 15}}));}
// Guards: the tile wholly inside the chart (the request is today's,
// unchanged), and a chart whose east edge only touches tile column 1.
TEST_F(TileEdgesTest, TilesInsideChartGuard1m) {check(small(1.0, {{"c", 4, -13, -13, 17, 17}}));}
TEST_F(TileEdgesTest, ChartTouchingTileEdgeGuard1m) {check(small(1.0, {{"c", 4, -12, -12, 7, 15}}));}

// A chart's stated bounds differ from its grid's extent (as getGrid makes
// it, by less than a quarter chart cell on each side); a tile edge lies on
// the stated bound. Chart cells of two window cells. The pre-filter passes
// the tile on the outer side of that edge (it compares with > and <), so
// what the chart paints there is the clip's: it must clip to the grid's
// extent, not to the stated bounds.
// The grid a quarter window cell larger than the stated bounds on every
// side: the stated west bound on tile column 1's east edge (quarter 32), the
// grid's first column from quarter 31, so it paints window column 7 from
// [31, 32]; and the stated north bound on tile row 2's south edge, the
// grid's last row to quarter 33, so it paints window row 8 from [32, 33].
// Clipping to the stated bounds skips the chart in those tiles.
TEST_F(TileEdgesTest, GridLargerThanBoundsWest1m)
{
  check(small(1.0, {{"c", 8, 31, -12, 6, 8, Fill::hashed, 0, 1}}));
}
TEST_F(TileEdgesTest, GridLargerThanBoundsNorth1m)
{
  check(small(1.0, {{"c", 8, -12, -15, 8, 6, Fill::hashed, 0, 1}}));
}
// The grid a quarter window cell smaller: the stated west bound on tile
// column 1's east edge, the grid from quarter 33; the stated north bound on
// tile row 2's south edge, the grid to quarter 31. The pre-filter passes
// tile column 1 and row 2, which the grid does not reach: the chart must
// paint nothing there. (Clipping to the stated bounds gives the same cells:
// its request then yields only the grid's edge column or row, which lies
// wholly outside the tile and paints nothing.)
TEST_F(TileEdgesTest, GridSmallerThanBounds1m)
{
  check(small(1.0, {{"c", 8, 33, -17, 6, 6, Fill::hashed, 0, -1}}));
}
// Pins current behaviour, not the behaviour wanted: the grid half a window
// cell larger than the stated bounds (chart cells of three window cells),
// the grid's first column from quarter 31, the stated west bound at 33.
// Tile column 1 ends at 32, so the pre-filter (stated bounds, left as it is
// by this change) skips it, and the grid's [31, 32] in window column 7 is
// not painted.
TEST_F(TileEdgesTest, GridLargerThanBoundsPreFilterSkips1m)
{
  check(small(1.0, {{"c", 12, 31, -12, 4, 6, Fill::hashed, 0, 2}}));
}

// A chart grid one cell long on x, then on y, whose two cell centres on that
// axis round an ulp apart in the wrong order (checkOneCellAxis asserts it):
// generateTile must order the bounds it clamps the tile to, as std::clamp
// requires. The unfixed source passed crossed bounds. Where libstdc++ checks
// its assertions (-D_GLIBCXX_ASSERTIONS, which GCC 15's libstdc++ also
// defines in an unoptimized build such as this package's default colcon
// build) it aborts in std::clamp here. Without them it returns the upper
// bound, which gives these same cells, so in an optimized build (and with
// GCC 13 on Jazzy) this case cannot fail on the unfixed source. The fixed
// source passes either way.
TEST_F(TileEdgesTest, OneCellLongOnX) {checkOneCellAxis(0);}
TEST_F(TileEdgesTest, OneCellLongOnY) {checkOneCellAxis(1);}

}  // namespace

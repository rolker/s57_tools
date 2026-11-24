// Scratch harness for rolker/s57_tools#57 (not part of the package).
//
// Compiles two copies of the layer into this executable, so no installed
// library can stand in for either (ldd shows no libs57_layer):
//   final  src/s57_layer.cpp, the checkout's source (namespace s57_layer);
//   today  today/s57_layer.cpp, the source before #57 (renamed by measure.sh
//          to namespace s57_today).
// measure.sh makes both headers' private members protected, so the test
// subclass can flag tiles as a chart-set change does and read the strips'
// pending spans. Each cycle drives Nav2's own LayeredCostmap::updateMap.
//
//   harness correct [<inflation set-up 0-4> ...]
//       The adversarial scenarios: each cell of today's and the final
//       layer's window is compared after every cycle with a fresh window
//       (the final code, moved to the same origin in its first cycle).
//   harness time <local|global> <none|native|jazzycopy> <heading deg> <m/s> <cycles> <today|final>
//       Mean, p50 and worst updateMap time on BEN's costmap sizes.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#if __has_include("nav2_ros_common/lifecycle_node.hpp")
#include "nav2_ros_common/lifecycle_node.hpp"
using HNode = nav2::LifecycleNode;
#else
#include "nav2_util/lifecycle_node.hpp"
using HNode = nav2_util::LifecycleNode;
#endif
#include "s57_msgs/msg/dataset_info.hpp"
#include "tf2_ros/buffer.h"

#include "s57_layer.h"
#include "today/s57_layer.h"
// WITH_JAZZY_COPY: Jazzy's InflationLayer copied from Nav2's origin/jazzy and
// renamed (measure.sh makes it), for running it on Rolling. On Jazzy the
// native InflationLayer is Jazzy's, so the copy is not built.
#ifdef WITH_JAZZY_COPY
#include "jazzy/inflation_layer.hpp"
#endif

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using GridPtr = std::shared_ptr<grid_map::GridMap>;

template<class L>
class ForTest : public L
{
public:
  void addChart(const std::string & label, GridPtr grid, bool present)
  {
    s57_msgs::msg::DatasetInfo info;
    info.label = label;
    this->current_charts_.push_back(info);
    const auto & p = grid->getPosition();
    const auto & l = grid->getLength();
    geometry_msgs::msg::Point a, b;
    a.x = p.x() - l.x() / 2; a.y = p.y() - l.y() / 2;
    b.x = p.x() + l.x() / 2; b.y = p.y() + l.y() / 2;
    this->chart_bounds_[label] = {a, b};
    this->grids_[label] = present ? grid : nullptr;
  }
  void setGrid(const std::string & label, GridPtr g) {this->grids_[label] = g;}
  void clearCharts()
  {
    this->current_charts_.clear(); this->chart_bounds_.clear(); this->grids_.clear();
  }
  // As getDatasetsCallback does when the chart set changes.
  void chartSetChanged()
  {
    for (auto & t : this->tiles_) {
      t.second.complete = false; t.second.chart_count = 0; t.second.needs_update = true;
    }
  }
};

enum Code {TODAY = 0, FINAL = 1};
const char * codeName(Code c) {return c == TODAY ? "today" : "final";}

struct LayerOps
{
  std::shared_ptr<nav2_costmap_2d::Layer> layer;
  std::function<void(const std::string &, GridPtr, bool)> addChart;
  std::function<void(const std::string &, GridPtr)> setGrid;
  std::function<void()> clearCharts, chartSetChanged;
  std::function<double()> spanWidth;  // widest pending span (final only), m
};

class FinalForTest : public ForTest<s57_layer::S57Layer>
{
public:
  double spanWidth() const
  {
    double w = 0;
    if (pending_x_.valid) {w = std::max(w, pending_x_.hi - pending_x_.lo);}
    if (pending_y_.valid) {w = std::max(w, pending_y_.hi - pending_y_.lo);}
    return w;
  }
};

template<class T>
LayerOps makeOps()
{
  auto l = std::make_shared<T>();
  LayerOps o;
  o.layer = l;
  o.addChart = [l](const std::string & s, GridPtr g, bool p) {l->addChart(s, g, p);};
  o.setGrid = [l](const std::string & s, GridPtr g) {l->setGrid(s, g);};
  o.clearCharts = [l]() {l->clearCharts();};
  o.chartSetChanged = [l]() {l->chartSetChanged();};
  o.spanWidth = []() {return 0.0;};
  return o;
}

LayerOps makeLayer(Code code)
{
  if (code == TODAY) {return makeOps<ForTest<s57_today::S57Layer>>();}
  auto o = makeOps<FinalForTest>();
  auto l = std::static_pointer_cast<FinalForTest>(o.layer);
  o.spanWidth = [l]() {return l->spanWidth();};
  return o;
}

using LandFn = std::function<float(double, double)>;  // returns elevation

// A chart grid with every channel the layer reads; cells of res whose edges
// lie a quarter cell off the tile edges (rolker/s57_tools#63).
GridPtr makeGrid(double cx, double cy, double len, double res, LandFn f)
{
  auto g = std::make_shared<grid_map::GridMap>(
    std::vector<std::string>{"elevation", "overhead", "restricted", "unsurveyed", "caution"});
  g->setGeometry(grid_map::Length(len, len), res, grid_map::Position(cx, cy));
  for (const auto & n : g->getLayers()) {(*g)[n].setConstant(std::numeric_limits<float>::quiet_NaN());}
  for (grid_map::GridMapIterator it(*g); !it.isPastEnd(); ++it) {
    grid_map::Position p; g->getPosition(*it, p);
    g->at("elevation", *it) = f(p.x(), p.y());
  }
  return g;
}

struct ChartSpec {std::string label; GridPtr grid; bool present;};

enum InflKind {NONE = 0, ROLLING = 1, JAZZY = 2};

// A layer that reports a box and paints nothing: before chart_layer, or
// after it (a box around the robot, or a thin rectangle at the leading x
// edge of the window).
class BoxLayer : public nav2_costmap_2d::Layer
{
public:
  explicit BoxLayer(int kind)
  : kind_(kind) {}
  void onInitialize() override {current_ = true;}
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(double rx, double ry, double, double * a, double * b, double * c, double * d) override
  {
    double x0, y0, x1, y1;
    if (kind_ == 3) {
      auto * p = layered_costmap_->getCostmap();
      x1 = p->getOriginX() + p->getSizeInCellsX() * p->getResolution();
      x0 = x1 - p->getResolution();
      y0 = ry - 2.0; y1 = ry + 2.0;
    } else {
      x0 = rx - 1.5; x1 = rx + 1.5; y0 = ry - 1.5; y1 = ry + 1.5;
    }
    *a = std::min(*a, x0); *b = std::min(*b, y0); *c = std::max(*c, x1); *d = std::max(*d, y1);
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override {}

private:
  int kind_;
};
const char * otherName(int k)
{
  static const char * n[] = {"no other layer", "box before", "box after", "edge strip after"};
  return n[k];
}

// Records the bounds it is given (placed after chart_layer: what it reported).
class SpyLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override {current_ = true;}
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(double, double, double, double * a, double * b, double * c, double * d) override
  {
    auto * p = layered_costmap_->getCostmap();
    double x0 = std::max(*a, p->getOriginX()), y0 = std::max(*b, p->getOriginY());
    double x1 = std::min(*c, p->getOriginX() + p->getSizeInCellsX() * p->getResolution());
    double y1 = std::min(*d, p->getOriginY() + p->getSizeInCellsY() * p->getResolution());
    area = (x1 > x0 && y1 > y0) ? (x1 - x0) * (y1 - y0) / p->getResolution() / p->getResolution() : 0;
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override {}
  double area = 0;
};

struct Stack
{
  std::shared_ptr<HNode> node;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> lc;
  LayerOps ops;
  std::shared_ptr<nav2_costmap_2d::Layer> infl;
  std::shared_ptr<SpyLayer> spy;
};

static int g_node_counter = 0;

void makeStack(
  Stack & s, tf2_ros::Buffer * tf, Code code, bool rolling, bool track_unknown, unsigned n,
  double res, int tile, InflKind ik, double radius, double scaling,
  const std::vector<geometry_msgs::msg::Point> & footprint, bool allow_uncharted, bool tide,
  const std::vector<ChartSpec> & charts, int other = 0, bool spy = false)
{
  rclcpp::NodeOptions opts;
  s.node = std::make_shared<HNode>("h" + std::to_string(g_node_counter++), "", opts);
  s.node->declare_parameter("chart_layer.tile_size", tile);
  s.node->declare_parameter("chart_layer.allow_uncharted", allow_uncharted);
  if (tide) {
    s.node->declare_parameter("chart_layer.chart_datum_frame", std::string("chart_datum"));
    s.node->declare_parameter("chart_layer.sea_surface_frame", std::string("map_tide"));
  }
  s.node->declare_parameter("inflation_layer.inflation_radius", radius);
  s.node->declare_parameter("inflation_layer.cost_scaling_factor", scaling);
  s.lc = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", rolling, track_unknown);
  s.lc->resizeMap(n, n, res, 0.0, 0.0);
  auto addOther = [&]() {
      auto o = std::make_shared<BoxLayer>(other);
      s.lc->addPlugin(o);
      o->initialize(s.lc.get(), "other", tf, s.node, nullptr);
    };
  if (other == 1) {addOther();}
  s.ops = makeLayer(code);
  s.lc->addPlugin(s.ops.layer);
  s.ops.layer->initialize(s.lc.get(), "chart_layer", tf, s.node, nullptr);
  for (auto & c : charts) {s.ops.addChart(c.label, c.grid, c.present);}
  if (spy) {
    s.spy = std::make_shared<SpyLayer>();
    s.lc->addPlugin(s.spy);
    s.spy->initialize(s.lc.get(), "spy", tf, s.node, nullptr);
  }
  if (other >= 2) {addOther();}
  if (ik == ROLLING) {
    s.infl = std::make_shared<nav2_costmap_2d::InflationLayer>();
  } else if (ik == JAZZY) {
#ifdef WITH_JAZZY_COPY
    s.infl = std::make_shared<jazzy_inflation::InflationLayer>();
#else
    s.infl = std::make_shared<nav2_costmap_2d::InflationLayer>();
#endif
  }
  if (s.infl) {
    s.lc->addPlugin(s.infl);
    s.infl->initialize(s.lc.get(), "inflation_layer", tf, s.node, nullptr);
  }
  if (!footprint.empty()) {s.lc->setFootprint(footprint);}
}

std::vector<geometry_msgs::msg::Point> squareFootprint(double h)
{
  std::vector<geometry_msgs::msg::Point> f(4);
  for (int k = 0; k < 4; k++) {f[k].x = (k == 0 || k == 3) ? -h : h; f[k].y = (k < 2) ? -h : h;}
  return f;
}

// ---------------------------------------------------------------- correctness
using Path = std::vector<std::pair<double, double>>;

struct Scenario
{
  std::string name;
  Path path;
  unsigned cells = 20;
  double res = 1.0;
  bool rolling = true;
  bool allow_uncharted = true;
  bool track_unknown = false;
  bool no_charts = false;
  int reset_at = -1;
  int tide_at = -1;
  int chartset_at = -1;
  int late_grid_at = -1;  // the late chart's grid arrives at this cycle; -1 no late chart
  double late_cx = 0, late_cy = 0, late_len = 6.0;
};

// Land in about one cell in seven, per cell of res.
float hashLand(double x, double y, double res)
{
  long ix = static_cast<long>(std::floor(x / res)), iy = static_cast<long>(std::floor(y / res));
  long h = (ix * 73856093L) ^ (iy * 19349663L);
  return (((h % 7) + 7) % 7 == 0) ? 1.0f : -10.0f;
}

struct Totals
{
  long runs = 0, steps = 0;
  long hazard_wrong_runs = 0;   // runs with a lethal/unknown (or, no inflation, any) mismatch
  long late_runs = 0;           // runs with such a mismatch older than 2 cycles
  int max_age = -1;             // oldest such mismatch (age 0 = the cycle the cell entered)
  int max_streak = 0;           // longest run of consecutive wrong cycles of one world cell
  long value_cells = 0;         // inflation only: other value differences
  long value_unexplained = 0;   // of those, with no cell younger than 4 cycles within the radius
  long unmarked_lethal = 0;     // cell-cycles lethal today and not lethal in final
  long unmarked_lethal_real = 0;  // of those, lethal in the fresh window too
  long unmarked_lethal_old = 0;   // of those, cells in the window 3 cycles or more
  long unmarked_inscribed = 0;  // >= INSCRIBED (not unknown) today, below in final
  long unmarked_inscribed_real = 0;
  double max_span = 0;
  long origin_mismatch = 0;
  std::string first;
};

struct RunResult
{
  int max_age = -1; int max_streak = 0; long hazard_cells = 0; long value_cells = 0;
  long value_unexplained = 0; std::string first;
};

void runScenario(
  const Scenario & sc, InflKind ik, double radius, int other, RunResult res[2], Totals & tot)
{
  auto tfnode = std::make_shared<rclcpp::Node>("tfnode" + std::to_string(g_node_counter++));
  tf2_ros::Buffer tf(tfnode->get_clock());
  auto setTide = [&](double z) {
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = "chart_datum"; t.child_frame_id = "map_tide"; t.transform.translation.z = z;
      t.transform.rotation.w = 1.0;
      tf.setTransform(t, "harness", true);
    };
  const bool tide = sc.tide_at >= 0;
  if (tide) {setTide(0.0);}
  const double r = sc.res;
  LandFn base = tide ?
    LandFn([r](double x, double y) {return hashLand(x, y, r) > 0 ? 1.0f : -1.0f;}) :
    LandFn([r](double x, double y) {return hashLand(x, y, r);});
  // Charts wide enough for every path: 300 cells around (20, 20) m scaled.
  const double span = 300.0 * r, centre = 20.0 * r + 0.25 * r;
  auto coast = makeGrid(centre, centre, span, r, base);
  auto coast2 = makeGrid(centre, centre, span, r, [r](double x, double y) {return hashLand(y + 3 * r, x, r);});
  auto late = makeGrid(sc.late_cx, sc.late_cy, sc.late_len, r / 2,
      [r](double x, double y) {return hashLand(x * 2, y * 2 + r, r);});
  std::vector<ChartSpec> charts;
  if (!sc.no_charts) {
    charts.push_back({"coast", coast, true});
    if (sc.late_grid_at >= 0) {charts.push_back({"a_late", late, false});}
  }
  auto fp = squareFootprint(0.25 * r);
  const unsigned N = sc.cells;
  const auto nofp = std::vector<geometry_msgs::msg::Point>{};
  Stack s[2];
  for (int c = 0; c < 2; c++) {
    makeStack(s[c], &tf, Code(c), sc.rolling, sc.track_unknown, N, r, 4, ik, radius, 0.5,
      ik ? fp : nofp, sc.allow_uncharted, tide, charts, other);
  }
  const int rad = static_cast<int>(std::ceil(radius / r));
  std::vector<std::vector<int>> age(N, std::vector<int>(N, 0));
  std::vector<std::vector<int>> streak[2] = {age, age};
  double prev_ox = 0, prev_oy = 0;
  for (size_t step = 0; step < sc.path.size(); step++) {
    if ((int)step == sc.tide_at) {setTide(2.0);}
    if ((int)step == sc.chartset_at) {
      charts.clear(); charts.push_back({"coast2", coast2, true});
      for (int c = 0; c < 2; c++) {
        s[c].ops.clearCharts();
        for (auto & ch : charts) {s[c].ops.addChart(ch.label, ch.grid, ch.present);}
        s[c].ops.chartSetChanged();
      }
    }
    if ((int)step == sc.late_grid_at) {
      for (int c = 0; c < 2; c++) {s[c].ops.setGrid("a_late", late);}
      for (auto & ch : charts) {if (ch.label == "a_late") {ch.present = true;}}
    }
    if ((int)step == sc.reset_at) {
      for (int c = 0; c < 2; c++) {  // as Costmap2DROS::resetLayers
        auto * top = s[c].lc->getCostmap();
        top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
        s[c].ops.layer->reset();
        if (s[c].infl) {s[c].infl->reset();}
      }
    }
    auto [rx, ry] = sc.path[step];
    for (int c = 0; c < 2; c++) {s[c].lc->updateMap(rx, ry, 0.0);}
    tot.max_span = std::max(tot.max_span, s[FINAL].ops.spanWidth());
    auto * m0 = s[FINAL].lc->getCostmap();
    const double ox = m0->getOriginX(), oy = m0->getOriginY();
    // world-cell shift since last cycle: ages and streaks follow the cells
    const long dx = step ? std::lround((ox - prev_ox) / r) : 0, dy = step ? std::lround((oy - prev_oy) / r) : 0;
    auto shift = [&](std::vector<std::vector<int>> & a, int plus) {
        std::vector<std::vector<int>> b(N, std::vector<int>(N, 0));
        for (long j = 0; j < (long)N; j++) {
          for (long i = 0; i < (long)N; i++) {
            long pi = i + dx, pj = j + dy;
            b[j][i] = (step && pi >= 0 && pj >= 0 && pi < (long)N && pj < (long)N) ? a[pj][pi] + plus : 0;
          }
        }
        a = b;
      };
    shift(age, 1); shift(streak[0], 0); shift(streak[1], 0);
    prev_ox = ox; prev_oy = oy;
    // the oracle: the final code, a fresh window moved to this origin in one cycle
    Stack o;
    makeStack(o, &tf, FINAL, sc.rolling, sc.track_unknown, N, r, 4, ik, radius, 0.5,
      ik ? fp : nofp, sc.allow_uncharted, tide, charts, 0);
    if (sc.rolling) {
      o.lc->updateMap(ox + (ox < 0 ? -0.4 : 0.4) * r + N * r / 2.0,
        oy + (oy < 0 ? -0.4 : 0.4) * r + N * r / 2.0, 0.0);
    } else {
      o.lc->updateMap(rx, ry, 0.0);
    }
    auto * om = o.lc->getCostmap();
    if (std::abs(om->getOriginX() - ox) > 1e-6 * r || std::abs(om->getOriginY() - oy) > 1e-6 * r ||
      std::abs(s[TODAY].lc->getCostmap()->getOriginX() - ox) > 1e-6 * r)
    {
      tot.origin_mismatch++;
    }
    for (int c = 0; c < 2; c++) {
      auto * m = s[c].lc->getCostmap();
      for (unsigned j = 0; j < N; j++) {
        for (unsigned i = 0; i < N; i++) {
          const unsigned char v = m->getCost(i, j), e = om->getCost(i, j);
          bool hazard, value = false;
          if (ik == NONE) {
            hazard = v != e;
          } else {
            hazard = ((v == LETHAL_OBSTACLE) != (e == LETHAL_OBSTACLE)) ||
              ((v == NO_INFORMATION) != (e == NO_INFORMATION));
            const bool far = (int)i >= rad && (int)j >= rad && (int)i < (int)N - rad && (int)j < (int)N - rad;
            value = !hazard && ((e != NO_INFORMATION && v < e) || (far && v != e));
          }
          streak[c][j][i] = (hazard || value) ? streak[c][j][i] + 1 : 0;
          res[c].max_streak = std::max(res[c].max_streak, streak[c][j][i]);
          if (hazard) {
            res[c].hazard_cells++;
            if (age[j][i] > res[c].max_age) {
              res[c].max_age = age[j][i];
              if (c == FINAL && age[j][i] > 2) {
                char buf[240];
                std::snprintf(buf, sizeof buf, "step %zu robot (%.2f, %.2f) cell (%u,%u) cost %d expected %d age %d",
                  step, rx, ry, i, j, v, e, age[j][i]);
                res[c].first = buf;
              }
            }
          }
          if (value) {
            res[c].value_cells++;
            int min_age = 1 << 20;
            for (int b = -rad; b <= rad; b++) {
              for (int a = -rad; a <= rad; a++) {
                int x = (int)i + a, y = (int)j + b;
                if (x >= 0 && y >= 0 && x < (int)N && y < (int)N) {min_age = std::min(min_age, age[y][x]);}
              }
            }
            if (min_age > 3) {res[c].value_unexplained++;}
          }
        }
      }
    }
    // nothing marked today is unmarked by the fix (unless the fresh window agrees)
    auto * mt = s[TODAY].lc->getCostmap();
    for (unsigned j = 0; j < N; j++) {
      for (unsigned i = 0; i < N; i++) {
        const unsigned char t = mt->getCost(i, j), f = m0->getCost(i, j), e = om->getCost(i, j);
        if (t == LETHAL_OBSTACLE && f != LETHAL_OBSTACLE) {
          tot.unmarked_lethal++;
          tot.unmarked_lethal_real += e == LETHAL_OBSTACLE;
          tot.unmarked_lethal_old += age[j][i] > 2;
        }
        if (t != NO_INFORMATION && t >= INSCRIBED_INFLATED_OBSTACLE &&
          (f < INSCRIBED_INFLATED_OBSTACLE || f == NO_INFORMATION))
        {
          tot.unmarked_inscribed++;
          tot.unmarked_inscribed_real += e != NO_INFORMATION && e >= INSCRIBED_INFLATED_OBSTACLE;
        }
      }
    }
    tot.steps++;
  }
}

Path line(double x0, double y0, double dx, double dy, int n)
{
  Path p;
  for (int k = 0; k < n; k++) {p.push_back({x0 + k * dx, y0 + k * dy});}
  return p;
}
Path cat(Path a, const Path & b) {a.insert(a.end(), b.begin(), b.end()); return a;}
Path walk(unsigned seed, double x0, double y0, double maxstep, int n)
{
  std::mt19937 g(seed);
  std::uniform_real_distribution<double> u(-maxstep, maxstep);
  Path p{{x0, y0}};
  for (int k = 1; k < n; k++) {p.push_back({p.back().first + u(g), p.back().second + u(g)});}
  return p;
}
Path stay(const Path & p, int n) {Path q = p; for (int k = 0; k < n; k++) {q.push_back(q.back());} return q;}

std::vector<Scenario> scenarios()
{
  std::vector<Scenario> v;
  auto add = [&](const std::string & n, Path p) {Scenario s; s.name = n; s.path = p; v.push_back(s); return &v.back();};
  for (unsigned seed = 1; seed <= 4; seed++) {add("random walk 3m seed " + std::to_string(seed), walk(seed, 20.3, 20.7, 3.0, 150));}
  for (unsigned seed = 1; seed <= 2; seed++) {add("random walk 1.2m seed " + std::to_string(seed), walk(seed + 10, 20.3, 20.7, 1.2, 150));}
  add("velocity (0.7, 1.3)", line(20.5, 20.5, 0.7, 1.3, 120));
  add("velocity (2.3, -1.7)", line(20.5, 20.5, 2.3, -1.7, 120));
  add("velocity (0.37, 0.11)", line(20.5, 20.5, 0.37, 0.11, 200));
  // zig-zags after k cycles of diagonal travel (the phase of x/gap/y/gap)
  for (int k = 0; k < 4; k++) {
    Path pre = line(20.5, 20.5, 1.3, 0.9, k + 4);
    Path zx = pre, zxy = pre;
    for (int c = 0; c < 24; c++) {
      double d = ((c / 2) % 2) ? -1.5 : 1.5;
      zx.push_back({zx.back().first + d, zx.back().second});
      zxy.push_back({zxy.back().first + d, zxy.back().second + d * 0.7});
    }
    add("zig-zag x, phase " + std::to_string(k), zx);
    add("zig-zag x+y, phase " + std::to_string(k), zxy);
    Path rev = pre;
    Path back = line(rev.back().first, rev.back().second, -1.3, -0.9, 24);
    add("reversal, phase " + std::to_string(k), cat(rev, Path(back.begin() + 1, back.end())));
  }
  for (int k = 0; k < 5; k++) {
    Path p = stay(line(20.5, 20.5, 1.3, 0.9, k + 4), 6);
    Path more = line(p.back().first, p.back().second, 1.3, 0.9, 12);
    add("stop, phase " + std::to_string(k), cat(p, Path(more.begin() + 1, more.end())));
  }
  add("one-tile steps east", line(20.5, 20.5, 4, 0, 12));
  add("one-tile steps diagonal", line(20.5, 20.5, 4, 4, 12));
  add("one-tile steps north", line(20.5, 20.5, 0, 4, 12));
  add("jump past the window and back", {{20.5, 20.5}, {21.7, 20.9}, {51.7, 45.9}, {52.9, 46.4}, {22.1, 21.3}, {23.4, 22.2}, {24.7, 22.8}, {26.0, 23.7}, {26.0, 23.7}, {26.0, 23.7}});
  add("jump half a window and back", {{20.5, 20.5}, {21.7, 20.9}, {31.7, 30.9}, {32.9, 31.4}, {22.1, 21.3}, {23.4, 22.2}, {24.7, 22.8}, {26.0, 23.7}, {26.0, 23.7}, {26.0, 23.7}});
  add("across the origin", line(-25.5, -18.5, 1.1, 0.8, 60));
  add("random walk around 0", walk(21, 0.3, -0.4, 2.0, 120));
  for (unsigned n : {21u, 22u}) {
    auto * s = add("window " + std::to_string(n) + ", diagonal", line(20.5, 20.5, 1.3, 0.7, 60)); s->cells = n;
    s = add("window " + std::to_string(n) + ", random walk", walk(30 + n, 20.3, 20.7, 2.5, 100)); s->cells = n;
  }
  for (int k = 3; k <= 7; k++) {
    auto * s = add("tide step at " + std::to_string(k), line(20.5, 20.5, 1.3, 0.9, 16)); s->tide_at = k;
    s = add("chart set at " + std::to_string(k), line(20.5, 20.5, 1.3, 0.9, 16)); s->chartset_at = k;
    s = add("reset at " + std::to_string(k), line(20.5, 20.5, 1.3, 0.9, 16)); s->reset_at = k;
  }
  // late grids: a tile at the window's edge, under the robot, ahead of it
  struct L {const char * n; double cx, cy;};
  for (L l : {L{"edge", 12.25, 26.25}, L{"under robot", 26.25, 24.25}, L{"ahead", 42.25, 30.25}}) {
    for (int k = 3; k <= 6; k++) {
      auto * s = add(std::string("late grid ") + l.n + " at " + std::to_string(k), line(20.5, 20.5, 1.3, 0.6, 16));
      s->late_grid_at = k; s->late_cx = l.cx; s->late_cy = l.cy;
    }
  }
  {auto * s = add("track unknown, fixed window", line(5.5, 5.5, 0.7, 0.4, 12)); s->track_unknown = true; s->rolling = false;}
  {auto * s = add("track unknown, random walk", walk(41, 20.3, 20.7, 2.0, 100)); s->track_unknown = true;}
  {auto * s = add("uncharted not allowed, random walk", walk(42, 20.3, 20.7, 2.0, 80)); s->no_charts = true; s->allow_uncharted = false;}
  // 0.1 m cells: a resolution whose cell edges are not binary-exact
  {auto * s = add("0.1 m, diagonal", line(2.05, 2.05, 0.13, 0.07, 80)); s->res = 0.1;}
  {auto * s = add("0.1 m, random walk", walk(43, 2.03, 2.07, 0.25, 100)); s->res = 0.1;}
  return v;
}

int correctness(int argc, char ** argv)
{
  struct IC {InflKind k; double r; const char * n;};
  std::vector<IC> ics = {{NONE, 0, "no inflation"}, {ROLLING, 2, "Rolling r2"}, {ROLLING, 6, "Rolling r6"},
    {JAZZY, 2, "Jazzy r2"}, {JAZZY, 9, "Jazzy r9"}};
#ifndef WITH_JAZZY_COPY
  ics.resize(3);
#endif
  std::vector<int> pick;
  for (int a = 2; a < argc; a++) {pick.push_back(std::atoi(argv[a]));}
  if (pick.empty()) {for (size_t k = 0; k < ics.size(); k++) {pick.push_back(k);}}
  auto scs = scenarios();
  std::printf("%zu scenarios x 4 other-layer set-ups per inflation set-up\n", scs.size());
  for (int p : pick) {
    const IC & ic = ics.at(p);
    Totals t[2];
    for (int other = 0; other < 4; other++) {
      for (auto & sc : scs) {
        RunResult res[2];
        Totals tot;
        runScenario(sc, ic.k, ic.r * sc.res, other, res, tot);
        for (int c = 0; c < 2; c++) {
          t[c].runs++;
          t[c].steps += tot.steps;
          if (res[c].hazard_cells) {t[c].hazard_wrong_runs++;}
          if (res[c].max_age > 2) {
            t[c].late_runs++;
            if (c == FINAL && t[c].first.empty()) {
              t[c].first = sc.name + " / " + otherName(other) + ": " + res[c].first;
            }
          }
          t[c].max_age = std::max(t[c].max_age, res[c].max_age);
          t[c].max_streak = std::max(t[c].max_streak, res[c].max_streak);
          t[c].value_cells += res[c].value_cells;
          t[c].value_unexplained += res[c].value_unexplained;
        }
        t[FINAL].unmarked_lethal += tot.unmarked_lethal;
        t[FINAL].unmarked_lethal_real += tot.unmarked_lethal_real;
        t[FINAL].unmarked_lethal_old += tot.unmarked_lethal_old;
        t[FINAL].unmarked_inscribed += tot.unmarked_inscribed;
        t[FINAL].unmarked_inscribed_real += tot.unmarked_inscribed_real;
        t[FINAL].max_span = std::max(t[FINAL].max_span, tot.max_span / sc.res);
        t[FINAL].origin_mismatch += tot.origin_mismatch;
      }
    }
    for (int c = 0; c < 2; c++) {
      std::printf("%-12s %-5s runs=%ld cycles=%ld runs_wrong=%ld runs_wrong_after_age2=%ld oldest_wrong_age=%d longest_wrong_streak=%d value_diffs=%ld value_diffs_unexplained=%ld\n",
        ic.n, codeName(Code(c)), t[c].runs, t[c].steps, t[c].hazard_wrong_runs, t[c].late_runs, t[c].max_age,
        t[c].max_streak, t[c].value_cells, t[c].value_unexplained);
    }
    std::printf("%-12s before/after: lethal today, not lethal in final: %ld cell-cycles (lethal in the fresh window: %ld; in the window 3 cycles or more: %ld); >= inscribed today, below in final: %ld (>= inscribed in the fresh window: %ld); widest pending span %.1f cells; origin mismatches %ld\n",
      ic.n, t[FINAL].unmarked_lethal, t[FINAL].unmarked_lethal_real, t[FINAL].unmarked_lethal_old, t[FINAL].unmarked_inscribed,
      t[FINAL].unmarked_inscribed_real, t[FINAL].max_span, t[FINAL].origin_mismatch);
    if (!t[FINAL].first.empty()) {std::printf("  first late mismatch (final): %s\n", t[FINAL].first.c_str());}
    std::fflush(stdout);
  }
  return 0;
}

// ---------------------------------------------------------------- timing
int timing(int argc, char ** argv)
{
  if (argc < 8) {std::printf("bad args\n"); return 1;}
  std::string size = argv[2], inf = argv[3];
  double heading = std::atof(argv[4]) * M_PI / 180.0, speed = std::atof(argv[5]);
  int cycles = std::atoi(argv[6]);
  Code code = std::string(argv[7]) == "today" ? TODAY : FINAL;
  bool global = size == "global";
  unsigned n = global ? 5000 : 700; double res = global ? 1.0 : 0.5; double hz = global ? 1.0 : 5.0;
  double step = speed / hz;
  InflKind ik = inf == "native" ? ROLLING : inf == "jazzycopy" ? JAZZY : NONE;
  auto tfnode = std::make_shared<rclcpp::Node>("tftime");
  tf2_ros::Buffer tf(tfnode->get_clock());
  // coast to the north-east and 40 m islands on a 300 m lattice; 10 m chart
  // cells offset 0.37 m from the tile edges (rolker/s57_tools#63)
  auto chart = makeGrid(250.37, 250.37, 6000.0, 10.0, [](double x, double y) {
        double mx = std::fmod(x + 10000.0, 300.0), my = std::fmod(y + 10000.0, 300.0);
        return (x + y > 4600.0 || (mx < 40.0 && my < 40.0)) ? 1.0f : -10.0f;
      });
  std::vector<geometry_msgs::msg::Point> fp;
  double ben[][2] = {{2.65, 0}, {2.6, 0.1}, {2.4, 0.4}, {1.9, 0.6}, {1.0, 0.85}, {-1.3, 0.85}, {-1.35, 0.4}, {-1.6, 0.35}, {-1.6, -0.35}, {-1.35, -0.4}, {-1.3, -0.85}, {1.0, -0.85}, {1.9, -0.6}, {2.4, -0.4}, {2.6, -0.1}};
  for (auto & p : ben) {geometry_msgs::msg::Point q; q.x = p[0]; q.y = p[1]; fp.push_back(q);}
  Stack s;
  makeStack(s, &tf, code, true, global, n, res, 100, ik, 150.0, 0.05, ik ? fp : std::vector<geometry_msgs::msg::Point>{}, true, false, {{"coast", chart, true}}, 0, true);
  double x = 0.3, y = 0.3;
  for (int k = 0; k < 4; k++) {s.lc->updateMap(x, y, 0.0);}
  std::vector<double> t;
  double area = 0;
  for (int k = 0; k < cycles; k++) {
    x += step * std::cos(heading); y += step * std::sin(heading);
    auto a = std::chrono::steady_clock::now();
    s.lc->updateMap(x, y, 0.0);
    auto b = std::chrono::steady_clock::now();
    t.push_back(std::chrono::duration<double, std::milli>(b - a).count());
    area += s.spy->area;
  }
  double sum = 0, worst = 0; for (double v : t) {sum += v; worst = std::max(worst, v);}
  std::vector<double> sorted = t; std::sort(sorted.begin(), sorted.end());
  std::printf("%s %s hdg=%s v=%s %s cycles=%d mean=%.2f ms p50=%.2f worst=%.2f ms mean_reported_cells=%.0f\n",
    size.c_str(), inf.c_str(), argv[4], argv[5], codeName(code), cycles, sum / t.size(), sorted[sorted.size() / 2], worst, area / cycles);
  return 0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  int rc = 1;
  if (argc > 1 && std::string(argv[1]) == "correct") {rc = correctness(argc, argv);}
  if (argc > 1 && std::string(argv[1]) == "time") {rc = timing(argc, argv);}
  rclcpp::shutdown();
  return rc;
}

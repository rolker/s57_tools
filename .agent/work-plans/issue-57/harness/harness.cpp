// Scratch harness for issue 57 plan: correctness scenarios and timings.
// Scratch harness for rolker/s57_tools#57: compiles a variant copy of
// s57_layer.cpp (layer_variants.patch) into this executable and drives Nav2's
// LayeredCostmap::updateMap. Not part of the package.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
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
// WITH_JAZZY_COPY: Jazzy's InflationLayer copied from Nav2's origin/jazzy and
// renamed (measure.sh makes it), for timing it on Rolling. On Jazzy the
// native InflationLayer is Jazzy's, so the copy is not built.
#ifdef WITH_JAZZY_COPY
#include "jazzy/inflation_layer.hpp"
#endif

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

class S57LayerForTest : public s57_layer::S57Layer
{
public:
  void addChart(const std::string & label, std::shared_ptr<grid_map::GridMap> grid,
    bool grid_present = true)
  {
    s57_msgs::msg::DatasetInfo info;
    info.label = label;
    current_charts_.push_back(info);
    const auto & p = grid->getPosition();
    const auto & l = grid->getLength();
    geometry_msgs::msg::Point a, b;
    a.x = p.x() - l.x() / 2; a.y = p.y() - l.y() / 2;
    b.x = p.x() + l.x() / 2; b.y = p.y() + l.y() / 2;
    chart_bounds_[label] = {a, b};
    grids_[label] = grid_present ? grid : nullptr;
  }
  void setGrid(const std::string & label, std::shared_ptr<grid_map::GridMap> g) {grids_[label] = g;}
  void clearCharts() {current_charts_.clear(); chart_bounds_.clear(); grids_.clear();}
};

using LandFn = std::function<float(double, double)>;  // returns elevation

std::shared_ptr<grid_map::GridMap> makeGrid(double cx, double cy, double len, double res, LandFn f)
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

struct ChartSpec { std::string label; std::shared_ptr<grid_map::GridMap> grid; bool present; };

enum InflKind {NONE = 0, ROLLING = 1, JAZZY = 2};

struct Stack
{
  std::shared_ptr<HNode> node;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> lc;
  std::shared_ptr<S57LayerForTest> layer;
  std::shared_ptr<nav2_costmap_2d::Layer> infl;
};

static int g_node_counter = 0;

void makeStack(Stack & s, tf2_ros::Buffer * tf, bool rolling, bool track_unknown, unsigned sx, unsigned sy,
  double res, int tile, InflKind ik, double radius, double scaling,
  const std::vector<geometry_msgs::msg::Point> & footprint, int mode, bool allow_uncharted,
  bool tide, const std::vector<ChartSpec> & charts)
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
  s.lc->resizeMap(sx, sy, res, 0.0, 0.0);
  int saved = s57_layer::S57Layer::g_mode;
  s57_layer::S57Layer::g_mode = mode;
  s.layer = std::make_shared<S57LayerForTest>();
  s57_layer::S57Layer::g_mode = saved;
  s.lc->addPlugin(s.layer);
  s.layer->initialize(s.lc.get(), "chart_layer", tf, s.node, nullptr);
  for (auto & c : charts) {s.layer->addChart(c.label, c.grid, c.present);}
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
struct Scenario
{
  std::string name;
  std::vector<std::pair<double, double>> path;
  bool rolling = true;
  bool allow_uncharted = true;
  bool track_unknown = false;
  bool no_charts = false;
  int reset_at = -1;
  int tide_at = -1;
  int chartset_at = -1;
  int late_grid_at = -1;  // late grid present from this step; -1 = no late chart
  double late_cx = 24.25, late_cy = 10.25, late_len = 30.0;
};

float hashLand(double x, double y)
{
  long ix = static_cast<long>(std::floor(x)), iy = static_cast<long>(std::floor(y));
  long h = (ix * 73856093L) ^ (iy * 19349663L);
  return (((h % 7) + 7) % 7 == 0) ? 1.0f : -10.0f;
}

struct Result { int wrong_steps = 0; long wrong_cells = 0; int max_age = -1; long unexplained = 0; bool current_false_late = false; int not_current = 0; std::string first; };

Result runScenario(const Scenario & sc, int mode, InflKind ik, double radius, bool uncharted_fix, int current_flagged, int stale_allow)
{
  s57_layer::S57Layer::g_uncharted_fix = uncharted_fix;
  s57_layer::S57Layer::g_current_flagged = current_flagged;
  rclcpp::Clock::SharedPtr clk = std::make_shared<rclcpp::Clock>();
  auto tfnode = std::make_shared<rclcpp::Node>("tfnode" + std::to_string(g_node_counter++));
  tf2_ros::Buffer tf(tfnode->get_clock());
  auto setTide = [&](double z) {
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = "chart_datum"; t.child_frame_id = "map_tide"; t.transform.translation.z = z;
      t.transform.rotation.w = 1.0;
      tf.setTransform(t, "harness", true);
    };
  bool tide = sc.tide_at >= 0;
  if (tide) {setTide(0.0);}
  LandFn base = tide ? LandFn([](double x, double y) {return hashLand(x, y) > 0 ? 1.0f : -1.0f;}) : LandFn(hashLand);
  auto coast = makeGrid(20.25, 20.25, 220.0, 1.0, base);
  auto coast2 = makeGrid(20.25, 20.25, 220.0, 1.0, [](double x, double y) {return hashLand(y + 3, x) ;});
  // late chart: finer (0.5 m) so it wins, over a sub-area
  auto late = makeGrid(sc.late_cx, sc.late_cy, sc.late_len, 0.5, [](double x, double y) {return hashLand(x * 2, y * 2 + 1);});
  std::vector<ChartSpec> charts;
  if (!sc.no_charts) {
    charts.push_back({"coast", coast, true});
    if (sc.late_grid_at >= 0) {charts.push_back({"a_late", late, false});}
  }
  auto fp = squareFootprint(0.25);
  const unsigned N = 20;
  Stack s;
  makeStack(s, &tf, sc.rolling, sc.track_unknown, N, N, 1.0, 4, ik, radius, 0.5, ik ? fp : std::vector<geometry_msgs::msg::Point>{}, mode, sc.allow_uncharted, tide, charts);
  if (!sc.rolling) {s.lc->resizeMap(N, N, 1.0, 0.0, 0.0);}
  Result r;
  std::vector<std::vector<int>> age(N, std::vector<int>(N, 0));
  double prev_ox = std::nan(""), prev_oy = std::nan("");
  for (size_t step = 0; step < sc.path.size(); step++) {
    if ((int)step == sc.tide_at) {setTide(2.0);}
    if ((int)step == sc.chartset_at) {
      s.layer->clearCharts(); charts.clear(); charts.push_back({"coast2", coast2, true});
      for (auto & c : charts) {s.layer->addChart(c.label, c.grid, c.present);}
      s.layer->testChartSetChanged();
    }
    if ((int)step == sc.late_grid_at) {
      s.layer->setGrid("a_late", late);
      for (auto & c : charts) {if (c.label == "a_late") {c.present = true;}}
    }
    if ((int)step == sc.reset_at) {
      auto * top = s.lc->getCostmap();
      top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
      s.layer->reset();
      if (s.infl) {s.infl->reset();}
    }
    auto [rx, ry] = sc.path[step];
    s.lc->updateMap(rx, ry, 0.0);
    auto * m = s.lc->getCostmap();
    // ages
    double ox = m->getOriginX(), oy = m->getOriginY();
    std::vector<std::vector<int>> nage(N, std::vector<int>(N, 0));
    for (unsigned j = 0; j < N; j++) {
      for (unsigned i = 0; i < N; i++) {
        if (step == 0 || std::isnan(prev_ox)) {nage[j][i] = 0; continue;}
        long pi = std::lround((ox - prev_ox) / 1.0) + i, pj = std::lround((oy - prev_oy) / 1.0) + j;
        nage[j][i] = (pi >= 0 && pj >= 0 && pi < (long)N && pj < (long)N) ? age[pj][pi] + 1 : 0;
      }
    }
    age = nage; prev_ox = ox; prev_oy = oy;
    // oracle
    Stack o;
    makeStack(o, &tf, sc.rolling, sc.track_unknown, N, N, 1.0, 4, ik, radius, 0.5, ik ? fp : std::vector<geometry_msgs::msg::Point>{}, 1, sc.allow_uncharted, tide, charts);
    bool save_fix = s57_layer::S57Layer::g_uncharted_fix; s57_layer::S57Layer::g_uncharted_fix = true;
    if (!sc.rolling) {o.lc->resizeMap(N, N, 1.0, 0.0, 0.0);}
    if (sc.rolling) {o.lc->updateMap(ox + N / 2.0, oy + N / 2.0, 0.0);} else {o.lc->updateMap(rx, ry, 0.0);}
    s57_layer::S57Layer::g_uncharted_fix = save_fix;
    auto * om = o.lc->getCostmap();
    if (om->getOriginX() != ox || om->getOriginY() != oy) {std::printf("ORIGIN MISMATCH\n");}
    int rad = static_cast<int>(std::ceil(radius));
    long wrong = 0;
    for (unsigned j = 0; j < N; j++) {
      for (unsigned i = 0; i < N; i++) {
        unsigned char c = m->getCost(i, j), e = om->getCost(i, j);
        bool bad;
        if (ik == NONE) {
          bad = c != e;
        } else {
          bool far = (int)i >= rad && (int)j >= rad && (int)i < (int)N - rad && (int)j < (int)N - rad;
          bad = ((c == LETHAL_OBSTACLE) != (e == LETHAL_OBSTACLE)) || (e == NO_INFORMATION) != (c == NO_INFORMATION) ||
            (e != NO_INFORMATION && c < e) || (far && c != e);
        }
        if (bad) {
          wrong++;
          // explained by a recently uncovered cell within rad?
          int min_age = 1 << 20;
          for (int dj = -rad; dj <= rad; dj++) {
            for (int di = -rad; di <= rad; di++) {
              int a = (int)i + di, b = (int)j + dj;
              if (a >= 0 && b >= 0 && a < (int)N && b < (int)N) {min_age = std::min(min_age, age[b][a]);}
            }
          }
          if (min_age > stale_allow) {
            r.unexplained++;
            if (r.first.empty()) {
              char buf[200];
              std::snprintf(buf, sizeof buf, "step %zu cell (%u,%u) cost %d expected %d age %d", step, i, j, c, e, age[j][i]);
              r.first = buf;
            }
          }
          r.max_age = std::max(r.max_age, ik == NONE ? age[j][i] : min_age);
        }
      }
    }
    if (wrong) {r.wrong_steps++; r.wrong_cells += wrong;}
    if (step + 1 == sc.path.size() && !s.layer->isCurrentFlag()) {r.current_false_late = true;}
    if (step >= 2 && !s.layer->isCurrentFlag()) {r.not_current++;}
  }
  return r;
}

std::vector<std::pair<double, double>> line(double x0, double y0, double dx, double dy, int n)
{
  std::vector<std::pair<double, double>> p;
  for (int k = 0; k < n; k++) {p.push_back({x0 + k * dx, y0 + k * dy});}
  return p;
}

int correctness(int argc, char ** argv)
{
  (void)argc; (void)argv;
  std::vector<Scenario> scs;
  {Scenario s; s.name = "east 1m"; s.path = line(20.5, 20.5, 1, 0, 16); scs.push_back(s);}
  {Scenario s; s.name = "north 1m"; s.path = line(20.5, 20.5, 0, 1, 16); scs.push_back(s);}
  {Scenario s; s.name = "diag 1,1"; s.path = line(20.5, 20.5, 1, 1, 16); scs.push_back(s);}
  {Scenario s; s.name = "diag .6,.35"; s.path = line(20.5, 20.5, 0.6, 0.35, 30); scs.push_back(s);}
  {Scenario s; s.name = "diag 3,2"; s.path = line(20.5, 20.5, 3, 2, 12); scs.push_back(s);}
  {Scenario s; s.name = "reversal"; auto a = line(20.5, 20.5, 1, 0.5, 10); auto b = line(30.5, 25.5, -1, -0.5, 20); a.insert(a.end(), b.begin(), b.end()); s.path = a; scs.push_back(s);}
  {Scenario s; s.name = "jump+back"; s.path = {{20.5, 20.5}, {21.5, 20.5}, {71.5, 40.5}, {72.5, 41.5}, {21.5, 20.5}, {22.5, 21.5}, {23.5, 21.5}}; scs.push_back(s);}
  {Scenario s; s.name = "reset"; s.path = line(20.5, 20.5, 1, 1, 12); s.reset_at = 5; scs.push_back(s);}
  {Scenario s; s.name = "fixed window"; s.rolling = false; s.path = line(5.5, 5.5, 1, 1, 8); scs.push_back(s);}
  {Scenario s; s.name = "tide"; s.path = line(20.5, 20.5, 1, 1, 12); s.tide_at = 5; scs.push_back(s);}
  {Scenario s; s.name = "chart set"; s.path = line(20.5, 20.5, 1, 1, 12); s.chartset_at = 5; scs.push_back(s);}
  {Scenario s; s.name = "late grid"; s.path = line(20.5, 20.5, 1, 0.5, 14); s.late_grid_at = 7; scs.push_back(s);}
  {Scenario s; s.name = "late grid never"; s.path = line(20.5, 20.5, 1, 0.5, 14); s.late_grid_at = 1000; scs.push_back(s);}
  {Scenario s; s.name = "uncharted roll"; s.no_charts = true; s.allow_uncharted = false; s.path = line(20.5, 20.5, 1, 1, 10); scs.push_back(s);}
  {Scenario s; s.name = "uncharted fixed"; s.no_charts = true; s.allow_uncharted = false; s.rolling = false; s.path = line(5.5, 5.5, 1, 0, 4); scs.push_back(s);}
  {Scenario s; s.name = "unknown east"; s.track_unknown = true; s.path = line(20.5, 20.5, 1, 0, 16); scs.push_back(s);}

  const char * mname[] = {"now", "(a)", "(c)", "(d)"};
  struct IC {InflKind k; double r; const char * n;};
  std::vector<IC> ics = {{NONE, 0, "no-infl"}, {ROLLING, 2, "R r2"}, {ROLLING, 6, "R r6"}, {JAZZY, 2, "J r2"}, {JAZZY, 6, "J r6"}};
  for (int fix = 0; fix <= 1; fix++) {
    std::printf("=== uncharted_fix=%d\n", fix);
    for (auto & sc : scs) {
      for (auto & ic : ics) {
        std::printf("%-16s %-8s", sc.name.c_str(), ic.n);
        for (int mode = 0; mode < 4; mode++) {
          Result r = runScenario(sc, mode, ic.k, ic.r, fix, false, mode == 3 ? 3 : -1);
          std::printf(" | %s steps=%2d cells=%4ld unexpl=%4ld maxage=%2d cur=%s", mname[mode], r.wrong_steps, r.wrong_cells, r.unexplained, r.max_age, r.current_false_late ? "F" : "T");
        }
        std::printf("\n");
      }
    }
  }
  // current_: an edge tile whose grid never arrives (one tile, far from the
  // robot), and the same under the robot; rolling north, and a fixed window.
  const char * cname[] = {"today", "flagged", "core3x3"};
  for (int cf = 0; cf <= 2; cf++) {
    for (int mode = 0; mode < 4; mode++) {
      for (int infl = 0; infl <= 1; infl++) {
        Scenario e; e.path = line(20.5, 20.5, 0, 1, 14); e.late_grid_at = 1000;
        e.late_cx = 29.25; e.late_cy = 22.25; e.late_len = 2.0;
        Result q = runScenario(e, mode, infl ? ROLLING : NONE, 6, false, cf, 3);
        Scenario c = e; c.late_cx = 22.25; c.late_cy = 25.25;
        Result w = runScenario(c, mode, infl ? ROLLING : NONE, 6, false, cf, 3);
        Scenario f = e; f.rolling = false; f.path = line(15.5, 15.5, 0, 0, 8); f.late_cx = 19.25; f.late_cy = 5.25;
        Result x = runScenario(f, mode, infl ? ROLLING : NONE, 6, false, cf, 3);
        std::printf("current_=%-8s %s %-7s not-current cycles (of 12 after step 2): edge tile waits %2d | tile under robot waits %2d | fixed window, tile waits %d of 6\n",
          cname[cf], mname[mode], infl ? "infl r6" : "no-infl", q.not_current, w.not_current, x.not_current);
      }
    }
  }
  return 0;
}

// ---------------------------------------------------------------- timing
int timing(int argc, char ** argv)
{
  // args: time <local|global> <none|native|jazzycopy> <heading_deg> <speed_m_s> <cycles> <mode>
  // mode: 0 today, 1 (a), 2 (c), 3 (d), 4 (d) without the gap cycle
  if (argc < 8) {std::printf("bad args\n"); return 1;}
  std::string size = argv[2], inf = argv[3];
  double heading = std::atof(argv[4]) * M_PI / 180.0, speed = std::atof(argv[5]);
  int cycles = std::atoi(argv[6]), mode = std::atoi(argv[7]);
  bool global = size == "global";
  unsigned n = global ? 5000 : 700; double res = global ? 1.0 : 0.5; double hz = global ? 1.0 : 5.0;
  double step = speed / hz;
  // native: this distro's InflationLayer; jazzycopy: the copied Jazzy one.
  InflKind ik = inf == "native" ? ROLLING : inf == "jazzycopy" ? JAZZY : NONE;
  auto tfnode = std::make_shared<rclcpp::Node>("tftime");
  tf2_ros::Buffer tf(tfnode->get_clock());
  // coast to the north-east and 40 m islands on a 300 m lattice; chart edges
  // offset from tile edges (rolker/s57_tools#63)
  auto chart = makeGrid(250.37, 250.37, 6000.0, 10.0, [](double x, double y) {
        double mx = std::fmod(x + 10000.0, 300.0), my = std::fmod(y + 10000.0, 300.0);
        return (x + y > 4600.0 || (mx < 40.0 && my < 40.0)) ? 1.0f : -10.0f;
      });
  std::vector<geometry_msgs::msg::Point> fp;
  double ben[][2] = {{2.65, 0}, {2.6, 0.1}, {2.4, 0.4}, {1.9, 0.6}, {1.0, 0.85}, {-1.3, 0.85}, {-1.35, 0.4}, {-1.6, 0.35}, {-1.6, -0.35}, {-1.35, -0.4}, {-1.3, -0.85}, {1.0, -0.85}, {1.9, -0.6}, {2.4, -0.4}, {2.6, -0.1}};
  for (auto & p : ben) {geometry_msgs::msg::Point q; q.x = p[0]; q.y = p[1]; fp.push_back(q);}
  Stack s;
  makeStack(s, &tf, true, global, n, n, res, 100, ik, 150.0, 0.05, ik ? fp : std::vector<geometry_msgs::msg::Point>{}, mode, true, false, {{"coast", chart, true}});
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
    area += s.layer->last_report_area_;
  }
  double sum = 0, worst = 0; for (double v : t) {sum += v; worst = std::max(worst, v);}
  std::vector<double> sorted = t; std::sort(sorted.begin(), sorted.end());
  std::printf("%s %s hdg=%s v=%s mode=%d cycles=%d mean=%.2f ms p50=%.2f worst=%.2f ms mean_reported_cells=%.0f\n",
    size.c_str(), inf.c_str(), argv[4], argv[5], mode, cycles, sum / t.size(), sorted[sorted.size() / 2], worst, area / cycles);
  return 0;
}

class WideLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override {current_ = true;}
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(double, double, double, double * a, double * b, double * c, double * d) override
  {
    auto * p = layered_costmap_->getCostmap();
    *a = std::min(*a, p->getOriginX() - 50); *b = std::min(*b, p->getOriginY() - 50);
    *c = std::max(*c, p->getOriginX() + p->getSizeInMetersX() + 50);
    *d = std::max(*d, p->getOriginY() + p->getSizeInMetersY() + 50);
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override {}
};

int clampCheck()
{
  auto tfnode = std::make_shared<rclcpp::Node>("tfclamp");
  tf2_ros::Buffer tf(tfnode->get_clock());
  auto coast = makeGrid(20.25, 20.25, 220.0, 1.0, hashLand);
  long diff = 0;
  Stack st[2];
  for (int k = 0; k < 2; k++) {
    rclcpp::NodeOptions opts;
    st[k].node = std::make_shared<HNode>("clamp" + std::to_string(k), "", opts);
    st[k].node->declare_parameter("chart_layer.tile_size", 4);
    st[k].lc = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", true, false);
    st[k].lc->resizeMap(20, 20, 1.0, 0.0, 0.0);
    auto wide = std::make_shared<WideLayer>();
    st[k].lc->addPlugin(wide);
    wide->initialize(st[k].lc.get(), "wide", &tf, st[k].node, nullptr);
    s57_layer::S57Layer::g_mode = k == 0 ? 0 : 1;
    st[k].layer = std::make_shared<S57LayerForTest>();
    st[k].lc->addPlugin(st[k].layer);
    st[k].layer->initialize(st[k].lc.get(), "chart_layer", &tf, st[k].node, nullptr);
    st[k].layer->addChart("coast", coast);
  }
  for (int step = 0; step < 10; step++) {
    for (int k = 0; k < 2; k++) {st[k].lc->updateMap(20.5 + step, 20.5 + step * 0.5, 0.0);}
    auto * a = st[0].lc->getCostmap(); auto * b = st[1].lc->getCostmap();
    for (unsigned j = 0; j < 20; j++) {for (unsigned i = 0; i < 20; i++) {diff += a->getCost(i, j) != b->getCost(i, j);}}
  }
  std::printf("clamp check: cells differing between today's clamp and min/max widening, with an earlier layer reporting beyond the window: %ld\n", diff);
  return 0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  int rc = 1;
  if (argc > 1 && std::string(argv[1]) == "correct") {rc = correctness(argc, argv);}
  if (argc > 1 && std::string(argv[1]) == "time") {rc = timing(argc, argv);}
  if (argc > 1 && std::string(argv[1]) == "clamp") {rc = clampCheck();}
  rclcpp::shutdown();
  return rc;
}

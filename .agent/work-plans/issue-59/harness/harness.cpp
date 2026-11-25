// Scratch comparison for rolker/s57_tools#59 (plan phase; not part of the package).
// Two copies of the layer compiled in: today (s57_today, the checkout's source) and
// aprime (s57_aprime, today's plus the (a') request). generateTile is called on each
// for the same tile and charts; every tile cell is compared between the two and with an
// expectation computed from the chart specifications alone.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/GridMapMath.hpp"
#include "grid_map_core/SubmapGeometry.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "s57_msgs/msg/dataset_info.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "today/s57_layer.h"
#include "aprime/s57_layer.h"

long harness_aprime_failures = 0;

namespace
{
unsigned int g_tile = 0;
std::uint64_t g_calls = 0, g_oor = 0;
bool g_check = true;
}

namespace nav2_costmap_2d
{
unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const
{
  if (g_check && size_x_ == g_tile && size_y_ == g_tile) {
    g_calls++;
    if (mx >= size_x_ || my >= size_y_) {g_oor++; return NO_INFORMATION;}
  }
  return costmap_[getIndex(mx, my)];
}
void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
  if (g_check && size_x_ == g_tile && size_y_ == g_tile) {
    g_calls++;
    if (mx >= size_x_ || my >= size_y_) {g_oor++; return;}
  }
  costmap_[getIndex(mx, my)] = cost;
}
}  // namespace nav2_costmap_2d

using GridPtr = std::shared_ptr<grid_map::GridMap>;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

template<class L>
class Layer : public L
{
public:
  void setup(double res, int ts, double ox, double oy)
  {
    this->resolution_ = res; this->tile_size_ = ts; this->x_origin_ = ox; this->y_origin_ = oy;
    this->current_charts_.clear(); this->chart_bounds_.clear(); this->grids_.clear(); this->tiles_.clear();
  }
  void add(const std::string & label, GridPtr grid)
  {
    s57_msgs::msg::DatasetInfo info; info.label = label; this->current_charts_.push_back(info);
    const auto & p = grid->getPosition(); const auto & l = grid->getLength();
    geometry_msgs::msg::Point a, b;
    a.x = p.x() - l.x() / 2.0; a.y = p.y() - l.y() / 2.0;
    b.x = p.x() + l.x() / 2.0; b.y = p.y() + l.y() / 2.0;
    this->chart_bounds_[label] = std::make_pair(a, b);
    this->grids_[label] = grid;
  }
  std::shared_ptr<nav2_costmap_2d::Costmap2D> gen(int tx, int ty)
  {
    auto id = std::make_pair(tx, ty);
    this->tiles_.erase(id);
    this->generateTile(id);
    return this->tiles_[id].costmap;
  }
};

// A chart in its specification: cells of cr metres, nx x ny, west/south edge.
// Exact charts: every edge is an integer number q of quarter costmap cells from the
// layer's origin (wq, sq, kq), and the costmap resolution is 1 or 0.5.
struct Chart
{
  std::string label;
  double west, south, cr;
  int nx, ny;
  int mode;  // 0 hash with no-data, 1 all land, 2 all water
  std::uint32_t seed;
  bool exact = false;
  int ulps_x = 0, ulps_y = 0;
  long long wq = 0, sq = 0, kq = 0;
  GridPtr grid;
};

int chartedAt(const Chart & c, int m, int n)  // 0 none, 1 land, 2 water
{
  if (c.mode == 1) {return 1;}
  if (c.mode == 2) {return 2;}
  std::uint32_t h = (static_cast<std::uint32_t>(m) * 73856093u) ^
    (static_cast<std::uint32_t>(n) * 19349663u) ^ (c.seed * 83492791u);
  h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
  if (h % 5 == 0) {return 0;}
  return ((h >> 8) % 2) ? 1 : 2;
}
unsigned char costOf(int v) {return v == 1 ? LETHAL_OBSTACLE : v == 2 ? FREE_SPACE : NO_INFORMATION;}

double step(double v, int u)
{
  for (int k = 0; k < std::abs(u); k++) {v = std::nextafter(v, u > 0 ? HUGE_VAL : -HUGE_VAL);}
  return v;
}

void makeGrid(Chart & c)
{
  c.grid = std::make_shared<grid_map::GridMap>(
    std::vector<std::string>{"elevation", "overhead", "restricted", "unsurveyed", "caution"});
  c.grid->setGeometry(grid_map::Length(c.nx * c.cr, c.ny * c.cr), c.cr,
    grid_map::Position(step(c.west + c.nx * c.cr / 2.0, c.ulps_x), step(c.south + c.ny * c.cr / 2.0, c.ulps_y)));
  if (c.grid->getSize()(0) != c.nx || c.grid->getSize()(1) != c.ny) {
    std::fprintf(stderr, "grid size mismatch\n"); std::exit(2);
  }
  for (const auto & name : c.grid->getLayers()) {
    (*c.grid)[name].setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  auto & e = (*c.grid)["elevation"];
  // startIndex 0: index (i, j) is at max x - (i + 0.5) cr, so m = nx - 1 - i.
  for (int i = 0; i < c.nx; i++) {
    for (int j = 0; j < c.ny; j++) {
      int v = chartedAt(c, c.nx - 1 - i, c.ny - 1 - j);
      if (v == 1) {e(i, j) = 1.0f;} else if (v == 2) {e(i, j) = -10.0f;}
    }
  }
}

struct Case
{
  std::string family;
  double res; int T; double ox, oy;
  bool exact;
  std::vector<Chart> charts;
  std::vector<std::pair<int, int>> tiles;
};

// Range of chart cells overlapping [a, b) with positive area, a/b given in quarter
// units (exact) or metres. Returns false (empty) if none; sets ambiguous when an edge is
// within tol of a chart cell edge (inexact cases only).
bool rangeExact(long long a, long long b, long long w, long long k, int n, int & lo, int & hi)
{
  // cells m with w + m k < b and w + (m + 1) k > a
  auto floordiv = [](long long p, long long q) {long long d = p / q; if ((p % q != 0) && ((p < 0) != (q < 0))) {d--;} return d;};
  long long mlo = floordiv(a - w, k);            // smallest m with w + (m+1)k > a
  long long mhi = -floordiv(-(b - w), k) - 1;    // ceil((b-w)/k) - 1
  mlo = std::max<long long>(mlo, 0); mhi = std::min<long long>(mhi, n - 1);
  if (mlo > mhi) {return false;}
  lo = static_cast<int>(mlo); hi = static_cast<int>(mhi); return true;
}
bool rangeInexact(long double a, long double b, long double w, long double k, int n, int & lo, int & hi, bool & amb)
{
  const long double tol = 1e-6L;
  long double qa = (a - w) / k, qb = (b - w) / k;
  if (std::fabs(qa - std::round(qa)) * k < tol || std::fabs(qb - std::round(qb)) * k < tol) {
    // only ambiguous if the edge is inside the chart's extent (or at its ends)
    if (qb > -1e-9L && qa < n + 1e-9L) {amb = true;}
  }
  long long mlo = static_cast<long long>(std::floor(qa));
  long long mhi = static_cast<long long>(std::ceil(qb)) - 1;
  mlo = std::max<long long>(mlo, 0); mhi = std::min<long long>(mhi, n - 1);
  if (mlo > mhi) {return false;}
  lo = static_cast<int>(mlo); hi = static_cast<int>(mhi); return true;
}

// Expected cost of tile (tx, ty) cell (i, j): charts finest first (by grid resolution,
// then label), within a chart the east-most then north-most overlapping cell with data.
unsigned char expected(const Case & cs, const std::vector<const Chart *> & order, int tx, int ty, int i, int j, bool & amb)
{
  for (const Chart * c : order) {
    int mlo, mhi, nlo, nhi;
    if (cs.exact) {
      long long cx = 4LL * (static_cast<long long>(tx) * cs.T + i), cy = 4LL * (static_cast<long long>(ty) * cs.T + j);
      if (!rangeExact(cx, cx + 4, c->wq, c->kq, c->nx, mlo, mhi)) {continue;}
      if (!rangeExact(cy, cy + 4, c->sq, c->kq, c->ny, nlo, nhi)) {continue;}
    } else {
      long double x0 = (long double)cs.ox + ((long double)tx * cs.T + i) * cs.res;
      long double y0 = (long double)cs.oy + ((long double)ty * cs.T + j) * cs.res;
      bool a1 = false;
      bool okx = rangeInexact(x0, x0 + cs.res, c->west, c->cr, c->nx, mlo, mhi, a1);
      bool oky = rangeInexact(y0, y0 + cs.res, c->south, c->cr, c->ny, nlo, nhi, a1);
      if (a1) {amb = true;}
      if (!okx || !oky) {continue;}
    }
    for (int m = mhi; m >= mlo; m--) {
      for (int n = nhi; n >= nlo; n--) {
        int v = chartedAt(*c, m, n);
        if (v) {return costOf(v);}
      }
    }
  }
  return NO_INFORMATION;
}

struct Stats
{
  long cases = 0, tiles = 0;
  long s_tiles = 0, s_cells = 0, s_diff = 0, s_wrong_today = 0, s_wrong_aprime = 0;  // today ok for all charts
  long f_tiles = 0, f_cells = 0, f_wrong_today = 0, f_wrong_aprime = 0, f_changed = 0;
  long newly = 0, l2f = 0, f2l = 0, other_change = 0, unmarked = 0;
  long amb_cells = 0, aprime_fail = 0, oor = 0, chart_requests = 0, today_fail_requests = 0;
  long cells_iter_today = 0, cells_iter_aprime = 0;
  long ok_clamped = 0, ok_submap_diff = 0, zero_width = 0, today_idx_oor = 0, today_idx_oor_tiles = 0;
  void add(const Stats & o)
  {
    cases += o.cases; tiles += o.tiles; s_tiles += o.s_tiles; s_cells += o.s_cells; s_diff += o.s_diff;
    s_wrong_today += o.s_wrong_today; s_wrong_aprime += o.s_wrong_aprime; f_tiles += o.f_tiles;
    f_cells += o.f_cells; f_wrong_today += o.f_wrong_today; f_wrong_aprime += o.f_wrong_aprime;
    f_changed += o.f_changed; newly += o.newly; l2f += o.l2f; f2l += o.f2l; other_change += o.other_change;
    unmarked += o.unmarked; amb_cells += o.amb_cells; aprime_fail += o.aprime_fail; oor += o.oor;
    chart_requests += o.chart_requests; today_fail_requests += o.today_fail_requests;
    ok_clamped += o.ok_clamped; today_idx_oor += o.today_idx_oor; today_idx_oor_tiles += o.today_idx_oor_tiles; zero_width += o.zero_width; ok_submap_diff += o.ok_submap_diff;
    cells_iter_today += o.cells_iter_today; cells_iter_aprime += o.cells_iter_aprime;
  }
};

Layer<s57_today::S57Layer> * g_today;
Layer<s57_aprime::S57Layer> * g_aprime;
bool g_verbose = false;

// The (a') request as the patched layer makes it; returns false if no overlap.
bool aprimeRequest(const grid_map::GridMap & g, double wminx, double wminy, double wmaxx, double wmaxy,
  const grid_map::Position & tc, const grid_map::Length & tl, grid_map::Position & c, grid_map::Length & l)
{
  c = tc; l = tl;
  const double lo_t[2] = {wminx, wminy}, hi_t[2] = {wmaxx, wmaxy};
  const double h = g.getResolution() / 2.0;
  for (int a = 0; a < 2; a++) {
    const double glo = g.getPosition()[a] - g.getLength()[a] / 2.0;
    const double ghi = g.getPosition()[a] + g.getLength()[a] / 2.0;
    if (glo >= hi_t[a] || ghi <= lo_t[a]) {return false;}
    const double lo = std::clamp(lo_t[a], glo + h, ghi - h);
    const double hi = std::clamp(hi_t[a], glo + h, ghi - h);
    if (lo != lo_t[a] || hi != hi_t[a]) {c[a] = lo + (hi - lo) / 2.0; l[a] = hi - lo;}
  }
  return true;
}

double tx_min(const Case & cs, int tx) {return cs.ox + tx * cs.res * cs.T;}

Stats runCase(Case & cs)
{
  Stats st; st.cases = 1;
  for (auto & c : cs.charts) {makeGrid(c);}
  g_today->setup(cs.res, cs.T, cs.ox, cs.oy);
  g_aprime->setup(cs.res, cs.T, cs.ox, cs.oy);
  for (auto & c : cs.charts) {g_today->add(c.label, c.grid); g_aprime->add(c.label, c.grid);}
  std::vector<const Chart *> order;
  for (auto & c : cs.charts) {order.push_back(&c);}
  std::sort(order.begin(), order.end(), [](const Chart * a, const Chart * b) {
    return std::make_pair(a->grid->getResolution(), a->label) < std::make_pair(b->grid->getResolution(), b->label);});
  g_tile = cs.T;
  for (auto [tx, ty] : cs.tiles) {
    // Replicate generateTile's geometry and pre-filter, today's request outcome.
    double wminx = cs.ox + tx * cs.res * cs.T, wmaxx = wminx + cs.res * cs.T;
    double wminy = cs.oy + ty * cs.res * cs.T, wmaxy = wminy + cs.res * cs.T;
    grid_map::Position tc(wminx + cs.res * cs.T / 2.0, wminy + cs.res * cs.T / 2.0);
    grid_map::Length tl(cs.res * cs.T, cs.res * cs.T);
    bool all_ok = true; int passed = 0;
    for (auto & c : cs.charts) {
      const auto & p = c.grid->getPosition(); const auto & l = c.grid->getLength();
      double bx0 = p.x() - l.x() / 2.0, by0 = p.y() - l.y() / 2.0, bx1 = p.x() + l.x() / 2.0, by1 = p.y() + l.y() / 2.0;
      if (bx0 > wmaxx || bx1 < wminx || by0 > wmaxy || by1 < wminy) {continue;}
      passed++;
      bool ok = false;
      grid_map::SubmapGeometry sg(*c.grid, tc, tl, ok);
      st.chart_requests++;
      if (!ok) {
        all_ok = false; st.today_fail_requests++;
      } else {
        st.cells_iter_today += static_cast<long>(sg.getSize().prod());
        long oor = 0;
        for (grid_map::SubmapIterator it(sg); !it.isPastEnd(); ++it) {
          if (!grid_map::checkIfIndexInRange(*it, c.grid->getSize())) {oor++;}
        }
        st.today_idx_oor += oor; if (oor) {st.today_idx_oor_tiles++;}
      }
      grid_map::Position ac; grid_map::Length al;
      if (aprimeRequest(*c.grid, wminx, wminy, wmaxx, wmaxy, tc, tl, ac, al)) {
        bool ok2 = false;
        if (al(0) == 0.0 || al(1) == 0.0) {st.zero_width++;}
        grid_map::SubmapGeometry sg2(*c.grid, ac, al, ok2);
        if (ok2) {st.cells_iter_aprime += static_cast<long>(sg2.getSize().prod());}
        if (ok && ((ac.array() != tc.array()).any() || (al != tl).any())) {
          st.ok_clamped++;
          if (!ok2 || (sg.getStartIndex() != sg2.getStartIndex()).any() || (sg.getSize() != sg2.getSize()).any()) {
            st.ok_submap_diff++;
            if (g_verbose) {
              const auto & gp = c.grid->getPosition(); const auto & gl = c.grid->getLength();
              std::printf("  SUBMAP-DIFF %s res=%g T=%d cr=%.17g tile x[%.17g,%.17g] y[%.17g,%.17g] grid x[%.17g,%.17g] y[%.17g,%.17g] today start(%d,%d) size(%d,%d) aprime start(%d,%d) size(%d,%d)\n",
                cs.family.c_str(), cs.res, cs.T, c.cr, wminx, wmaxx, wminy, wmaxy, gp.x() - gl.x() / 2, gp.x() + gl.x() / 2, gp.y() - gl.y() / 2, gp.y() + gl.y() / 2,
                sg.getStartIndex()(0), sg.getStartIndex()(1), sg.getSize()(0), sg.getSize()(1),
                sg2.getStartIndex()(0), sg2.getStartIndex()(1), sg2.getSize()(0), sg2.getSize()(1));
            }
          }
        }
      }
    }
    if (passed == 0) {continue;}
    st.tiles++;
    g_calls = 0; g_oor = 0;
    long before = harness_aprime_failures;
    auto t0 = g_today->gen(tx, ty);
    auto t1 = g_aprime->gen(tx, ty);
    st.aprime_fail += harness_aprime_failures - before;
    st.oor += g_oor;
    if (!t0 || !t1) {std::fprintf(stderr, "no costmap\n"); std::exit(3);}
    long diff = 0, wt = 0, wa = 0, cells = 0;
    for (int j = 0; j < cs.T; j++) {
      for (int i = 0; i < cs.T; i++) {
        unsigned char a = t0->getCharMap()[t0->getIndex(i, j)];
        unsigned char b = t1->getCharMap()[t1->getIndex(i, j)];
        bool amb = false;
        unsigned char e = expected(cs, order, tx, ty, i, j, amb);
        if (a != b) {
          diff++;
          if (!all_ok) {
            if (a == NO_INFORMATION) {st.newly++;}
            else if (b == NO_INFORMATION) {st.unmarked++;}
            else if (a == LETHAL_OBSTACLE && b == FREE_SPACE) {st.l2f++;}
            else if (a == FREE_SPACE && b == LETHAL_OBSTACLE) {st.f2l++;}
            else {st.other_change++;}
          }
        }
        if (amb) {st.amb_cells++; continue;}
        cells++;
        if (a != e) {wt++;}
        if (b != e) {wa++;}
      }
    }
    if (all_ok) {
      st.s_tiles++; st.s_cells += cs.T * cs.T; st.s_diff += diff; st.s_wrong_today += wt; st.s_wrong_aprime += wa;
      if ((diff || wa || wt) && g_verbose) {
        std::printf("  S-DIFF %s tile(%d,%d) diff=%ld wt=%ld wa=%ld\n", cs.family.c_str(), tx, ty, diff, wt, wa);
      }
    } else {
      st.f_tiles++; st.f_cells += cs.T * cs.T; st.f_wrong_today += wt; st.f_wrong_aprime += wa; st.f_changed += diff;
      if (wa && g_verbose) {
        std::printf("  F-WRONG %s tile(%d,%d) wa=%ld\n", cs.family.c_str(), tx, ty, wa);
      }
    }
  }
  return st;
}

// ---- case generation ----
std::mt19937_64 rng;
double U(double a, double b) {return std::uniform_real_distribution<double>(a, b)(rng);}
long long I(long long a, long long b) {return std::uniform_int_distribution<long long>(a, b)(rng);}
template<class T> T pick(const std::vector<T> & v) {return v[I(0, v.size() - 1)];}

// Chart from quarter units (exact) relative to the layer origin.
Chart exactChart(const Case & cs, const std::string & label, long long wq, long long sq, long long kq, int nx, int ny, int mode)
{
  Chart c; c.label = label; c.exact = true; c.wq = wq; c.sq = sq; c.kq = kq; c.nx = nx; c.ny = ny; c.mode = mode;
  c.seed = static_cast<std::uint32_t>(I(1, 1000000));
  c.cr = kq * cs.res / 4.0; c.west = cs.ox + wq * cs.res / 4.0; c.south = cs.oy + sq * cs.res / 4.0;
  return c;
}
Chart freeChart(const std::string & label, double west, double south, double cr, int nx, int ny, int mode)
{
  Chart c; c.label = label; c.west = west; c.south = south; c.cr = cr; c.nx = nx; c.ny = ny; c.mode = mode;
  c.seed = static_cast<std::uint32_t>(I(1, 1000000));
  return c;
}

long long tileOfQ(const Case & cs, long long q) {  // tile index holding quarter unit q (floor)
  long long tq = 4LL * cs.T; long long d = q / tq; if (q % tq != 0 && q < 0) {d--;} return d;
}
int tileOfM(const Case & cs, double x, double o) {return static_cast<int>(std::floor((x - o) / (cs.res * cs.T)));}

void addTilesForChart(Case & cs, const Chart & c, int edge_samples)
{
  std::set<std::pair<int, int>> s(cs.tiles.begin(), cs.tiles.end());
  int x0 = tileOfM(cs, c.west, cs.ox), x1 = tileOfM(cs, c.west + c.nx * c.cr, cs.ox);
  int y0 = tileOfM(cs, c.south, cs.oy), y1 = tileOfM(cs, c.south + c.ny * c.cr, cs.oy);
  for (int cx : {x0, x1}) {for (int cy : {y0, y1}) {
    for (int dx = -1; dx <= 1; dx++) {for (int dy = -1; dy <= 1; dy++) {s.insert({cx + dx, cy + dy});}}}}
  for (int k = 0; k < edge_samples; k++) {
    int tx = static_cast<int>(I(x0, x1)), ty = static_cast<int>(I(y0, y1));
    s.insert({tx, y0}); s.insert({tx, y1}); s.insert({x0, ty}); s.insert({x1, ty});
    s.insert({tx, y0 + 1}); s.insert({x0 + 1, ty});
    s.insert({tx, ty});
  }
  // map axes
  int ax = tileOfM(cs, 0.0, cs.ox), ay = tileOfM(cs, 0.0, cs.oy);
  for (int dx = -1; dx <= 0; dx++) {
    if (ax + dx >= x0 - 1 && ax + dx <= x1 + 1) {
      s.insert({ax + dx, y0}); s.insert({ax + dx, y1}); s.insert({ax + dx, static_cast<int>(I(y0, y1))});
    }
  }
  for (int dy = -1; dy <= 0; dy++) {
    if (ay + dy >= y0 - 1 && ay + dy <= y1 + 1) {
      s.insert({x0, ay + dy}); s.insert({x1, ay + dy}); s.insert({static_cast<int>(I(x0, x1)), ay + dy});
    }
  }
  if (ax >= x0 - 1 && ax <= x1 + 1 && ay >= y0 - 1 && ay <= y1 + 1) {
    for (int dx = -1; dx <= 0; dx++) {for (int dy = -1; dy <= 0; dy++) {s.insert({ax + dx, ay + dy});}}
  }
  cs.tiles.assign(s.begin(), s.end());
}

// Random case: 1-3 charts.
Case randomCase(bool exact)
{
  Case cs; cs.exact = exact; cs.family = exact ? "random-exact" : "random-inexact";
  cs.res = exact ? pick<double>({1.0, 0.5}) : pick<double>({1.0, 0.5, 0.1});
  cs.T = static_cast<int>(pick<long long>({4, 100}));
  const double tileLen = cs.res * cs.T;
  int oc = static_cast<int>(I(0, 3));
  if (exact) {
    // origin a multiple of res/4 (exactly representable)
    double q = cs.res / 4.0;
    cs.ox = oc == 0 ? 0.0 : oc == 1 ? I(-4000, 4000) * q : oc == 2 ? I(-400000, 400000) * q + 1e5 : -1e5 + I(-4000, 4000) * q;
    cs.oy = oc == 0 ? 0.0 : oc == 1 ? I(-4000, 4000) * q : oc == 2 ? I(-400000, 400000) * q - 1e5 : 1e5 + I(-4000, 4000) * q;
  } else {
    cs.ox = oc == 0 ? 0.0 : oc == 1 ? U(-300, 300) : oc == 2 ? U(-1e5, 1e5) : U(-1.2e5, -0.8e5);
    cs.oy = oc == 0 ? 0.0 : oc == 1 ? U(-300, 300) : oc == 2 ? U(-1e5, 1e5) : U(0.8e5, 1.2e5);
  }
  int ncharts = static_cast<int>(I(1, 3));
  // primary chart
  double L = pick<double>({0.4 * tileLen, 0.9 * tileLen, 2.5 * tileLen, 500.0, 5000.0, 20000.0, 100000.0});
  double fac = exact ? pick<double>({0.5, 1.0, 2.0, 3.25, 7.0, 30.0}) : U(0.3, 30.0);
  double cr = fac * cs.res;
  if (L / cr > 1500) {cr = L / 1500;}
  long long kq = exact ? std::max<long long>(1, static_cast<long long>(std::ceil(cr / (cs.res / 4.0)))) : 0;
  if (exact) {cr = kq * cs.res / 4.0;}
  int nx = std::max(1, static_cast<int>(std::round(L / cr))), ny = std::max(1, static_cast<int>(std::round(L * U(0.6, 1.4) / cr)));
  ny = std::min(ny, 1500);
  // west/south positions: around an axis, SW edge near an axis, around the origin's tiles, or far
  int pc = static_cast<int>(I(0, 3));
  double west, south;
  double ext = nx * cr, exty = ny * cr;
  if (pc == 0) {west = U(-ext, 0); south = U(-exty, 0);}            // straddles both axes
  else if (pc == 1) {west = U(-0.05, 0.05) * ext; south = U(-0.05, 0.05) * exty;}  // SW edge near an axis
  else if (pc == 2) {west = cs.ox + U(-3, 3) * tileLen; south = cs.oy + U(-3, 3) * tileLen;}
  else {west = pick<double>({1e5, -1e5}) + U(-2000, 2000); south = pick<double>({1e5, -1e5}) + U(-2000, 2000);}
  if (exact) {
    long long wq = static_cast<long long>(std::llround((west - cs.ox) / (cs.res / 4.0)));
    long long sq = static_cast<long long>(std::llround((south - cs.oy) / (cs.res / 4.0)));
    cs.charts.push_back(exactChart(cs, "c0", wq, sq, kq, nx, ny, 0));
  } else {
    cs.charts.push_back(freeChart("c0", west, south, cr, nx, ny, 0));
  }
  const Chart p = cs.charts[0];
  for (int k = 1; k < ncharts; k++) {
    int kind = static_cast<int>(I(0, 3));  // 0 seam east, 1 seam north, 2 coarse under, 3 random overlap
    double f2 = exact ? pick<double>({0.5, 1.0, 2.0, 3.25, 7.0, 30.0}) : U(0.3, 30.0);
    double cr2 = f2 * cs.res;
    if (kind == 2) {cr2 = std::max(cr2, p.cr * pick<double>({2.0, 4.0, 8.0}));}
    double L2 = kind == 2 ? std::max(ext, exty) * 1.6 + 4 * tileLen : pick<double>({0.4 * tileLen, 2.5 * tileLen, 500.0, 5000.0});
    if (L2 / cr2 > 1500) {cr2 = L2 / 1500;}
    long long kq2 = exact ? std::max<long long>(1, static_cast<long long>(std::ceil(cr2 / (cs.res / 4.0)))) : 0;
    if (exact) {cr2 = kq2 * cs.res / 4.0;}
    int n2 = std::max(1, static_cast<int>(std::round(L2 / cr2)));
    n2 = std::min(n2, 1500);
    double w2, s2;
    if (kind == 0) {w2 = p.west + ext; s2 = p.south + U(-0.5, 0.5) * exty;}
    else if (kind == 1) {s2 = p.south + exty; w2 = p.west + U(-0.5, 0.5) * ext;}
    else if (kind == 2) {w2 = p.west - (n2 * cr2 - ext) / 2 + U(-1, 1) * tileLen; s2 = p.south - (n2 * cr2 - exty) / 2 + U(-1, 1) * tileLen;}
    else {w2 = p.west + U(-0.5, 1.0) * ext; s2 = p.south + U(-0.5, 1.0) * exty;}
    std::string label = "c" + std::to_string(k);
    int mode = kind == 2 ? static_cast<int>(I(0, 2)) : 0;
    if (exact) {
      long long wq = kind == 0 ? p.wq + p.kq * p.nx : static_cast<long long>(std::llround((w2 - cs.ox) / (cs.res / 4.0)));
      long long sq = kind == 1 ? p.sq + p.kq * p.ny : static_cast<long long>(std::llround((s2 - cs.oy) / (cs.res / 4.0)));
      cs.charts.push_back(exactChart(cs, label, wq, sq, kq2, n2, n2, mode));
    } else {
      cs.charts.push_back(freeChart(label, w2, s2, cr2, n2, n2, mode));
    }
  }
  for (const auto & c : cs.charts) {addTilesForChart(cs, c, c.label == "c0" ? 3 : 1);}
  return cs;
}

// ---- deterministic families (exact; quarter units of the costmap cell) ----
std::vector<Case> families()
{
  std::vector<Case> out;
  for (double res : {1.0, 0.5}) {
    for (int T : {4, 100}) {
      const long long tq = 4LL * T;
      std::vector<long long> edges;  // positions inside tile 0, quarter units
      if (T == 4) {for (long long e = 1; e < 16; e++) {edges.push_back(e);}}
      else {edges = {4, 145, 197, 199, 200, 201, 203, 257, 396};}
      for (double origin : {1000.25, 0.0}) {
        for (long long kq : {2LL, 4LL, 8LL, 13LL}) {
          const double o = origin * res;
          // one chart: west part / east part / south / north / corners of tile (0,0)
          for (long long e : edges) {
            const long long span = 6 * tq;  // chart extends far beyond the tile
            int ncell = static_cast<int>((span + kq - 1) / kq);
            struct Side {const char * name; long long wq, sq; int nx, ny;};
            long long W = e - ncell * kq;  // chart east edge at e
            std::vector<Side> sides = {
              {"one-chart-west-part", W, -2 * tq, ncell, ncell},
              {"one-chart-east-part", e, -2 * tq, ncell, ncell},
              {"one-chart-south-part", -2 * tq, W, ncell, ncell},
              {"one-chart-north-part", -2 * tq, e, ncell, ncell},
              {"corner-SW", W, W, ncell, ncell},
              {"corner-NE", e, e, ncell, ncell},
            };
            for (auto & sd : sides) {
              Case cs; cs.family = sd.name; cs.res = res; cs.T = T; cs.ox = o; cs.oy = o; cs.exact = true;
              cs.charts.push_back(exactChart(cs, "c0", sd.wq, sd.sq, kq, sd.nx, sd.ny, 0));
              cs.tiles = {{0, 0}, {-1, 0}, {0, -1}, {-1, -1}, {1, 0}, {0, 1}};
              out.push_back(cs);
            }
            // seam at e between two charts, x and y
            for (int axis = 0; axis < 2; axis++) {
              Case cs; cs.family = "seam"; cs.res = res; cs.T = T; cs.ox = o; cs.oy = o; cs.exact = true;
              if (axis == 0) {
                cs.charts.push_back(exactChart(cs, "c0", W, -2 * tq, kq, ncell, ncell, 0));
                cs.charts.push_back(exactChart(cs, "c1", e, -2 * tq, kq, ncell, ncell, 0));
              } else {
                cs.charts.push_back(exactChart(cs, "c0", -2 * tq, W, kq, ncell, ncell, 0));
                cs.charts.push_back(exactChart(cs, "c1", -2 * tq, e, kq, ncell, ncell, 0));
              }
              cs.tiles = {{0, 0}, {-1, 0}, {0, -1}, {-1, -1}};
              out.push_back(cs);
            }
            // fine water chart on the west part over a coarse all-land chart
            {
              Case cs; cs.family = "fine-water-over-coarse-land"; cs.res = res; cs.T = T; cs.ox = o; cs.oy = o; cs.exact = true;
              cs.charts.push_back(exactChart(cs, "fine", W, -2 * tq, kq, ncell, ncell, 2));
              long long kc = kq * 4;
              int nc = static_cast<int>((12 * tq) / kc) + 1;
              cs.charts.push_back(exactChart(cs, "coarse", -6 * tq - 3, -6 * tq - 1, kc, nc, nc, 1));
              cs.tiles = {{0, 0}, {-1, 0}};
              out.push_back(cs);
            }
          }
          // small chart inside tile (0,0), off centre and over it
          for (int k = 0; k < 6; k++) {
            Case cs; cs.family = k < 4 ? "small-chart-off-centre" : "small-chart-over-centre";
            cs.res = res; cs.T = T; cs.ox = o; cs.oy = o; cs.exact = true;
            long long sz = tq / 3;
            int n = static_cast<int>(std::max<long long>(1, sz / kq));
            long long wq = k == 0 ? 1 : k == 1 ? tq - n * kq - 1 : k == 2 ? 3 : k == 3 ? tq / 2 + 1 : tq / 2 - (n * kq) / 2 - (k == 5);
            long long sq = k == 0 ? 2 : k == 1 ? 1 : k == 2 ? tq - n * kq - 2 : k == 3 ? 5 : tq / 2 - (n * kq) / 2 - 1;
            if (n * kq >= tq) {continue;}
            cs.charts.push_back(exactChart(cs, "c0", wq, sq, kq, n, n, 0));
            cs.tiles = {{0, 0}};
            out.push_back(cs);
          }
        }
      }
    }
  }
  // map-axis band: origin 0, a 5 km chart whose west/south edge lies inside the tile at
  // the axis, corners from -400 m to 400 m (tile 0..); and the chart's own SW edge near the axis
  for (double res : {1.0, 0.5}) {
    for (int T : {4, 100}) {
      const long long tq = 4LL * T;
      for (long long kq : {4LL, 13LL, 30LL}) {
        for (int r = 0; r < 40; r++) {
          Case cs; cs.family = "axis-band"; cs.res = res; cs.T = T; cs.ox = 0.0; cs.oy = 0.0; cs.exact = true;
          // tile index near the axis band (+-400 m)
          long long tiles_band = static_cast<long long>(std::ceil(400.0 / (res * T)));
          long long tx = I(-tiles_band, tiles_band), ty = I(-tiles_band, tiles_band);
          long long wq = tx * tq + I(1, tq - 1), sq = ty * tq + I(1, tq - 1);
          int n = static_cast<int>((5000 * 4 / res) / kq);
          n = std::min(n, 1500);
          cs.charts.push_back(exactChart(cs, "c0", wq, sq, kq, n, n, 0));
          cs.tiles = {{static_cast<int>(tx), static_cast<int>(ty)}, {static_cast<int>(tx) + 1, static_cast<int>(ty)},
            {static_cast<int>(tx), static_cast<int>(ty) + 1}, {static_cast<int>(tx) + 1, static_cast<int>(ty) + 1}};
          out.push_back(cs);
          // the same chart, a tile further west/south than its edge (the clipped-to-bounds (a) case)
          Case c2 = cs; c2.family = "chart-edge-near-axis";
          c2.tiles = {{static_cast<int>(tx), static_cast<int>(ty)}, {static_cast<int>(tx) - 0, static_cast<int>(ty) + 2}, {static_cast<int>(tx) + 2, static_cast<int>(ty)}};
          out.push_back(c2);
        }
      }
    }
  }
  return out;
}

int main(int argc, char ** argv)
{
  std::string mode = argc > 1 ? argv[1] : "all";
  long nrandom = argc > 2 ? std::atol(argv[2]) : 500;
  unsigned long seed = argc > 3 ? std::strtoul(argv[3], nullptr, 10) : 1;
  g_verbose = std::getenv("VERBOSE") != nullptr;
  rng.seed(seed);
  Layer<s57_today::S57Layer> today; Layer<s57_aprime::S57Layer> aprime;
  g_today = &today; g_aprime = &aprime;
  std::map<std::string, Stats> per;
  auto run = [&](Case & cs) {Stats s = runCase(cs); per[cs.family].add(s); cs.charts.clear();};
  if (mode == "band") {
    // Today's request for a tile whose west (south) edge lies 'inset' metres west (south)
    // of a 1:12000 chart's west (south) edge, chart covering the tile's centre; tile corner swept.
    for (double res : {1.0, 0.5}) {
      for (double inset : {0.25, 10.0, 36.25, 49.0}) {
        for (int axis = 0; axis < 2; axis++) {
          const int T = 100; const double tl = res * T; const double cr = 3.75; const int nx = 1627, ny = 2213;
          double lo = 1e9, hi = -1e9; long fails = 0, n = 0;
          for (double X = -3000.0; X <= 3000.0; X += 0.37) {
            double cw = X + inset * res;  // chart's near edge
            Chart c = freeChart("a", axis == 0 ? cw : 5.0, axis == 0 ? 5.0 : cw, cr, nx, ny, 0);
            // geometry only: no data needed
            grid_map::GridMap g; g.setGeometry(grid_map::Length(nx * cr, ny * cr), cr,
              grid_map::Position(c.west + nx * cr / 2.0, c.south + ny * cr / 2.0));
            double wx = axis == 0 ? X : 20.0, wy = axis == 0 ? 20.0 : X;
            grid_map::Position tc(wx + tl / 2.0, wy + tl / 2.0); grid_map::Length tlen(tl, tl);
            bool ok = false; grid_map::SubmapGeometry sg(g, tc, tlen, ok); n++;
            grid_map::Position ac; grid_map::Length al; bool ok2 = false;
            aprimeRequest(g, wx, wy, wx + tl, wy + tl, tc, tlen, ac, al);
            grid_map::SubmapGeometry sg2(g, ac, al, ok2);
            if (!ok2) {std::printf("APRIME FAIL X=%g\n", X);}
            if (!ok) {fails++; lo = std::min(lo, X); hi = std::max(hi, X);}
          }
          std::printf("res %.1f axis %c inset %5.2f cells: today fails %ld of %ld tile corners, from %.1f to %.1f m\n",
            res, axis ? 'y' : 'x', inset, fails, n, lo, hi);
        }
      }
    }
    return 0;
  }
  if (mode == "time") {
    g_check = false;
    // 1:12000 cells (3.75 m), 6.1 x 8.3 km, two side by side (seam at x = west + 6100 m).
    for (double res : {1.0, 0.5}) {
      const int T = 100;
      Case cs; cs.res = res; cs.T = T; cs.ox = 0.0; cs.oy = 0.0; cs.exact = false;
      const double cr = 3.75; const int nx = 1627, ny = 2213;
      const double west = 1234.567, south = -2345.678;
      cs.charts.push_back(freeChart("a", west, south, cr, nx, ny, 0));
      cs.charts.push_back(freeChart("b", west + nx * cr, south, cr, nx, ny, 0));
      for (auto & c : cs.charts) {makeGrid(c);}
      g_today->setup(res, T, 0.0, 0.0); g_aprime->setup(res, T, 0.0, 0.0);
      for (auto & c : cs.charts) {g_today->add(c.label, c.grid); g_aprime->add(c.label, c.grid);}
      const double tl = res * T;
      struct TT {const char * name; int tx, ty;};
      const double seam = west + nx * cr;
      std::vector<TT> tts = {
        {"interior", static_cast<int>(std::floor((west + 2000) / tl)), static_cast<int>(std::floor((south + 3000) / tl))},
        {"west edge (chart a)", static_cast<int>(std::floor(west / tl)), static_cast<int>(std::floor((south + 3000) / tl))},
        {"east edge (chart b)", static_cast<int>(std::floor((west + 2 * nx * cr) / tl)), static_cast<int>(std::floor((south + 3000) / tl))},
        {"seam a|b", static_cast<int>(std::floor(seam / tl)), static_cast<int>(std::floor((south + 3000) / tl))},
        {"SW corner (chart a)", static_cast<int>(std::floor(west / tl)), static_cast<int>(std::floor(south / tl))},
      };
      for (auto & t : tts) {
        for (int which = 0; which < 2; which++) {
          std::vector<double> times;
          for (int r = 0; r < 2000; r++) {
            auto a = std::chrono::steady_clock::now();
            if (which == 0) {g_today->gen(t.tx, t.ty);} else {g_aprime->gen(t.tx, t.ty);}
            auto b = std::chrono::steady_clock::now();
            times.push_back(std::chrono::duration<double, std::micro>(b - a).count());
          }
          std::sort(times.begin(), times.end());
          // chart cells iterated
          double wminx = tx_min(cs, t.tx), wminy = cs.oy + t.ty * res * T;
          long cells = 0; int fails = 0;
          grid_map::Position tc(wminx + res * T / 2.0, wminy + res * T / 2.0); grid_map::Length tlen(res * T, res * T);
          for (auto & c : cs.charts) {
            const auto & p = c.grid->getPosition(); const auto & l = c.grid->getLength();
            if (p.x() - l.x() / 2 > wminx + tl || p.x() + l.x() / 2 < wminx || p.y() - l.y() / 2 > wminy + tl || p.y() + l.y() / 2 < wminy) {continue;}
            grid_map::Position ac = tc; grid_map::Length al = tlen; bool ok = false;
            if (which == 1 && !aprimeRequest(*c.grid, wminx, wminy, wminx + tl, wminy + tl, tc, tlen, ac, al)) {continue;}
            grid_map::SubmapGeometry sg(*c.grid, ac, al, ok);
            if (ok) {cells += static_cast<long>(sg.getSize().prod());} else {fails++;}
          }
          std::printf("res %.1f %-22s %-6s failed requests %d chart cells %6ld  median %8.1f us  p95 %8.1f us\n", res, t.name, which ? "aprime" : "today", fails, cells,
            times[times.size() / 2], times[times.size() * 95 / 100]);
        }
      }
      cs.charts.clear();
    }
    // map-axis band width for the 1:12000 cell: SW edge at x0 inside tile, tile corner swept
    return 0;
  }
  if (mode == "all" || mode == "families") {
    auto fam = families();
    for (auto & cs : fam) {run(cs);}
  }
  if (mode == "all" || mode == "random") {
    for (long k = 0; k < nrandom; k++) {
      Case cs = randomCase(k % 5 < 3);
      if (k % 5 == 2) {  // exact geometry nudged by a few ulps: edges within rounding of tile-cell edges
        cs.family = "random-ulp-nudged"; cs.exact = false;
        for (auto & c : cs.charts) {c.ulps_x = static_cast<int>(I(-3, 3)); c.ulps_y = static_cast<int>(I(-3, 3));}
      }
      run(cs);
    }
  }
  Stats tot;
  std::printf("%-30s %6s %6s | %7s %10s %6s %5s %5s | %6s %9s %9s %9s %8s | %7s %6s %6s %5s %5s | %6s %4s %4s %5s\n",
    "family", "cases", "tiles", "S-tiles", "S-cells", "S-diff", "S-wT", "S-wA", "F-tile", "F-cells", "F-wrongT", "F-wrongA", "F-chg",
    "newly", "L->F", "F->L", "othr", "unmk", "amb", "aFl", "oor", "req");
  for (auto & [k, s] : per) {
    tot.add(s);
    std::printf("%-30s %6ld %6ld | %7ld %10ld %6ld %5ld %5ld | %6ld %9ld %9ld %9ld %8ld | %7ld %6ld %6ld %5ld %5ld | %6ld %4ld %4ld %5ld\n",
      k.c_str(), s.cases, s.tiles, s.s_tiles, s.s_cells, s.s_diff, s.s_wrong_today, s.s_wrong_aprime, s.f_tiles, s.f_cells,
      s.f_wrong_today, s.f_wrong_aprime, s.f_changed, s.newly, s.l2f, s.f2l, s.other_change, s.unmarked, s.amb_cells,
      s.aprime_fail, s.oor, s.chart_requests);
  }
  auto & s = tot;
  std::printf("%-30s %6ld %6ld | %7ld %10ld %6ld %5ld %5ld | %6ld %9ld %9ld %9ld %8ld | %7ld %6ld %6ld %5ld %5ld | %6ld %4ld %4ld %5ld\n",
    "TOTAL", s.cases, s.tiles, s.s_tiles, s.s_cells, s.s_diff, s.s_wrong_today, s.s_wrong_aprime, s.f_tiles, s.f_cells,
    s.f_wrong_today, s.f_wrong_aprime, s.f_changed, s.newly, s.l2f, s.f2l, s.other_change, s.unmarked, s.amb_cells,
    s.aprime_fail, s.oor, s.chart_requests);
  std::printf("requests today succeeded on that (a') clamped: %ld; of those with a different submap: %ld\n", s.ok_clamped, s.ok_submap_diff);
  std::printf("today's requests yielding indices outside the chart grid: %ld (indices %ld)\n", s.today_idx_oor_tiles, s.today_idx_oor);
  std::printf("(a') requests of zero width on an axis: %ld\n", s.zero_width);
  std::printf("today-failed requests %ld of %ld; chart cells iterated today %ld, aprime %ld\n",
    s.today_fail_requests, s.chart_requests, s.cells_iter_today, s.cells_iter_aprime);
  return 0;
}

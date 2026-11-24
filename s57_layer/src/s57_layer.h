#ifndef S57_LAYER_H
#define S57_LAYER_H

#include <algorithm>
#include <future>
#include <limits>
#include <unordered_set>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "s57_msgs/srv/get_datasets.hpp"


namespace s57_layer
{

class S57Layer: public nav2_costmap_2d::Layer
{
public:
  S57Layer();
  ~S57Layer();

  void onInitialize() override;

  void reset() override;

  bool isClearable() override { return false; }

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double* min_x, double* min_y,
    double* max_x, double* max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i, int min_j, int max_i, int max_j)  override;

  void matchSize() override;

protected:
  // Exposed for unit testing (see test/test_tide_offset.cpp).
  // Subclassed-test access only; not part of the layer's public API.
  unsigned char get_cost_from_grid(grid_map::GridMap &grid, const grid_map::Index &index);

  // Same as above — declared protected so the test fixture can set
  // tide_offset_ directly instead of driving it through the full TF
  // lookup path in updateBounds().  Moved out of the private block
  // below; the rest of the layer's internal helpers and state stay
  // private.
  double tide_offset_ = 0.0;

  // ADR-0010 D10 mode switch. True (default) keeps the historical behavior:
  // this layer computes depth costs from charted DEPARE bands via the
  // caution-depth ramp. False suppresses the depth ramp — bathymetry_layer
  // is the single depth authority — and this layer retains only non-depth
  // semantics (land, restricted, overhead, caution/unsurveyed) plus discrete
  // charted hazards (UWTROC/WRECKS/PIPSOL via the grid's "hazard" channel,
  // kept LETHAL regardless of charted depth). Protected for the same
  // test-fixture reason as tide_offset_.
  bool depth_costs_ = true;

  // Protected so a test fixture can give the layer a chart (see test/test_rolling_window.cpp).
  std::vector<s57_msgs::msg::DatasetInfo> current_charts_;
  std::map<std::string, std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> > chart_bounds_;
  std::map<std::string, std::shared_ptr<grid_map::GridMap> > grids_;

private:
  using GetDatasetsClient =
    rclcpp::Client<s57_msgs::srv::GetDatasets>;

  geographic_msgs::msg::GeoPoint worldToLatLon(double x, double y);
  geometry_msgs::msg::Point llToWorld(const geographic_msgs::msg::GeoPoint& geo_point);

  void getDatasetsCallback(GetDatasetsClient::SharedFuture future);

  void mapGridCallback(std::string grid_name,
    std::shared_ptr<grid_map_msgs::msg::GridMap> grid_map);


  GetDatasetsClient::SharedPtr get_datasets_client_;
  bool pending_datasets_request_ = false;

  std::string s57_grids_namespace_;

  // Bounds of the costmap with an extra buffer to
  // help have the data precached
  geometry_msgs::msg::PointStamped buffered_min_;
  geometry_msgs::msg::PointStamped buffered_max_;


  std::string global_frame_id_;

  // minimum depth considered not lethal and start of caution area
  double minimum_depth_ = 0.0;

  // maximum depth used for caution area
  double maximum_caution_depth_ = 5.0;

  // cost assigned to unsurveyed areas
  unsigned char unsurveyed_cost_ = 100;

  // minimum height required (meters)
  double overhead_clearance_ = 10.0;

  std::map<std::string, rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr> grid_subscriptions_;


  double x_origin_ = 0.0;
  double y_origin_ = 0.0;
  double resolution_ = 1.0;

  int tile_size_ = 100;

  double update_timeout_ = 0.5;

  // Fraction of the costmap window size to extend the buffered chart
  // request bounds. The inner ring (1×) acts as a pre-load buffer so
  // chart data is ready before the rolling window scrolls into it; the
  // outer ring (2×) limits how often a new get_datasets service call
  // is sent. A larger value pre-loads more area at the cost of more
  // memory and processing per request. The 0.05 default preserves the
  // historical hardcoded value.
  double buffer_fraction_ = 0.05;

  // When true, allow navigation in areas without ENC chart coverage.
  // When false, uncharted areas are marked NO_INFORMATION to block planning.
  bool allow_uncharted_ = true;

  // Optional tide correction via chart datum and sea surface frames.
  // When both are set, the layer calls
  //   lookupTransform(chart_datum_frame, sea_surface_frame)
  // to get the sea surface position expressed in the chart datum frame.
  // The Z component is the water height above MLLW (chart datum).
  // (tide_offset_ itself is declared in the protected section above so
  // the test fixture can set it directly.)
  std::string chart_datum_frame_;
  std::string sea_surface_frame_ = "map_tide";

  // Tide changes smaller than this (in meters) do not invalidate cached
  // tile state. The default 1 cm matches typical real-world tide rate
  // (~0.5 m/hr near peak ⇒ 1 cm step every ~72 s). Sim runs with
  // accelerated tide should override to a larger value.
  double tide_invalidate_threshold_ = 0.01;

  // A rectangle in world coordinates; empty until something is included.
  struct Rect
  {
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    bool empty() const {return min_x >= max_x || min_y >= max_y;}
    void include(double x0, double y0, double x1, double y1)
    {
      min_x = std::min(min_x, x0);
      min_y = std::min(min_y, y0);
      max_x = std::max(max_x, x1);
      max_y = std::max(max_y, y1);
    }
    void clip(double x0, double y0, double x1, double y1)
    {
      min_x = std::max(min_x, x0);
      min_y = std::max(min_y, y0);
      max_x = std::min(max_x, x1);
      max_y = std::min(max_y, y1);
    }
  };

  // Repainting the cells a rolling window's move uncovers (#57).
  //
  // When the window moves, Costmap2D::updateOrigin keeps the cells the old
  // and new windows share and resets the others to the default value, and
  // LayeredCostmap::updateMap repaints only the bounds its layers report.
  // So this layer reports the uncovered cells itself, as strips: an x strip
  // is a span of columns at the window's west or east edge over its full
  // height, a y strip a span of rows at its south or north edge over its
  // full width. chooseStrip says when each is reported.

  // A span [lo, hi] of world coordinates on one axis; empty unless valid.
  // Every real span is whole cells wide, so anything narrower than
  // min_width is rounding between two equal cell edges and is dropped.
  struct Span
  {
    double lo = 0.0;
    double hi = 0.0;
    bool valid = false;
    // Cycles since the oldest cell in the span was uncovered (0 in that
    // cycle). After a clip the oldest cells may be gone; it then overstates,
    // which only makes the span more urgent.
    int age = 0;

    void add(double a, double b, double min_width)
    {
      if(b - a < min_width)
        return;
      if(!valid)
        age = 0;
      lo = valid ? std::min(lo, a) : a;
      hi = valid ? std::max(hi, b) : b;
      valid = true;
    }
    void clip(double a, double b, double min_width)
    {
      lo = std::max(lo, a);
      hi = std::min(hi, b);
      valid = valid && hi - lo >= min_width;
    }
  };

  // The four strips (each an index into pending_), and what a cycle can
  // report instead of one: no strip (a gap), or the whole window.
  enum Strip {x_low, x_high, y_low, y_high, none, whole};

  // True when the next rolling cycle must report the whole window: no
  // window has been seen yet, or reset() or matchSize() ran since.
  // last_window_ is meaningful only while this is false.
  bool whole_pending_ = true;
  // The window of the last rolling cycle, by its cell edges.
  Rect last_window_;
  // For each strip, the columns or rows a move uncovered on that side of the
  // window that have not been reported since. Each lies within last_window_:
  // its cells are still in the window and not yet painted from the charts.
  // The two sides of an axis are kept apart: after a reversal both can be
  // pending, and one strip over both would span the window.
  Span pending_[4];
  // What this layer reported in the last rolling cycle: none in a gap.
  Strip last_reported_ = Strip::none;

  // Phase 1 of a rolling cycle: clip the pending spans to window, age them,
  // and add the columns and rows this cycle's move uncovered. Returns false
  // when strips cannot describe the move (whole_pending_, or no overlap
  // with last_window_): the whole window must be reported.
  bool recordUncovered(const Rect & window, double half_cell);
  // Phase 3: what to report this cycle when recordUncovered returned true.
  Strip chooseStrip() const;

  typedef std::pair<int, int> TileID;

  struct TileInfo
  {
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
    bool complete = false;
    std::size_t chart_count = 0;
    bool needs_update = false;
  };

  std::map<TileID, TileInfo> tiles_;

  TileID worldToTile(double x, double y);
  void generateTile(TileID id);

  // std::atomic<bool> abort_flag_ = false;
};

} // namespace s57_layer

#endif

#ifndef S57_LAYER_H
#define S57_LAYER_H

#include <future>
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

private:
  using GetDatasetsClient =
    rclcpp::Client<s57_msgs::srv::GetDatasets>;

  unsigned char get_cost_from_grid(grid_map::GridMap &grid, const grid_map::Index &index);

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

  std::vector<s57_msgs::msg::DatasetInfo> current_charts_;
  std::map<std::string, std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> > chart_bounds_;

  std::map<std::string, rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr> grid_subscriptions_;

  std::map<std::string, std::shared_ptr<grid_map::GridMap> > grids_;


  double x_origin_ = 0.0;
  double y_origin_ = 0.0;
  double resolution_ = 1.0;

  int tile_size_ = 100;

  double update_timeout_ = 0.5;

  // When true, allow navigation in areas without ENC chart coverage.
  // When false, uncharted areas are marked NO_INFORMATION to block planning.
  bool allow_uncharted_ = true;

  // Optional tide correction via chart datum and sea surface frames.
  // When both are set, the layer calls
  //   lookupTransform(chart_datum_frame, sea_surface_frame)
  // to get the sea surface position expressed in the chart datum frame.
  // The Z component is the water height above MLLW (chart datum).
  std::string chart_datum_frame_;
  std::string sea_surface_frame_ = "map_tide";
  double tide_offset_ = 0.0;

  // Tide changes smaller than this (in meters) do not invalidate cached
  // tile state. The default 1 cm matches typical real-world tide rate
  // (~0.5 m/hr near peak ⇒ 1 cm step every ~72 s). Sim runs with
  // accelerated tide should override to a larger value.
  double tide_invalidate_threshold_ = 0.01;

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

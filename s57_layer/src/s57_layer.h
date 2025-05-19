#ifndef S57_LAYER_H
#define S57_LAYER_H

#include <future>
#include <unordered_set>

#include "grid_map_core/GridMap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"


namespace marine_charts
{
  class S57Catalog;
}

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

  bool llToWorld(double lat, double lon, double &x, double &y);
  bool worldToLatLon(double x, double y, double &lat, double &lon);

  double minimumDepth() const;
  double maximumCautionDepth() const;
  double overheadClearance() const;
  unsigned char unsurveyedCost() const;

private:
  unsigned char get_cost_from_grid(grid_map::GridMap &grid, const grid_map::Index &index);
  
  std::shared_ptr<marine_charts::S57Catalog> m_s57Catalog;

  std::string m_global_frame_id;

  // minimum depth considered not lethal and start of caution area
  double m_minimum_depth = 0.0;

  // maximum depth used for caution area
  double m_maximum_caution_depth = 5.0;

  // cost assigned to unsurveyed areas
  unsigned char m_unsurveyed_cost = 100;

  // minimum height required (meters)
  double m_overhead_clearance = 10.0;

  using GridsByName = std::map<std::string, std::shared_ptr<grid_map::GridMap> >;
  GridsByName grids_;

  using GridFuturesByName = std::map<std::string, std::future<std::shared_ptr<grid_map::GridMap> > >;
  GridFuturesByName pending_grids_;

  double m_origin_x = 0.0;
  double m_origin_y = 0.0;
  double m_resolution = 1.0;

  int m_tile_size = 100;

  double m_update_timeout = 0.5;

  typedef std::pair<int, int> TileID;

  struct TileInfo
  {
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
    bool complete = false;
    std::size_t chart_count = 0;
    bool needs_update = false;
  };

  std::map<TileID, TileInfo> m_tiles;

  TileID worldToTile(double x, double y);
  void generateTile(TileID id);

  std::atomic<bool> abort_flag_ = false;
};

} // namespace s57_layer

#endif

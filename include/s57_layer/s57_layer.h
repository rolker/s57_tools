#ifndef S57_LAYER_H
#define S57_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <s57_layer/S57LayerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <unordered_set>

namespace s57_layer
{

class S57Catalog;

class S57Layer: public costmap_2d::Layer
{
public:
  S57Layer();

  virtual void onInitialize() override;
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)  override;  

  virtual void matchSize() override;

  bool llToWorld(double lat, double lon, double &x, double &y);
  bool worldToLatLon(double x, double y, double &lat, double &lon);

  double minimumDepth() const;
  double maximumCautionDepth() const;
  double overheadClearance() const;
  unsigned char maximumCautionCost() const;
  unsigned char unsurveyedCost() const;

private:
  void reconfigureCallback(S57LayerConfig &config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<S57LayerConfig> > m_reconfigureServer;

  std::shared_ptr<S57Catalog> m_s57Catalog;

  std::string m_global_frame_id;

  // minimum depth considered considered not lethal and start of caution area
  double m_minimum_depth;

  // maximum depth used for caution area
  double m_maximum_caution_depth;

  // cost assigned at shallowest end of caution area
  unsigned char m_maximum_caution_cost;

  unsigned char m_unsurveyed_cost;

  double m_overhead_clearance;

  std::map<std::string, std::shared_ptr<costmap_2d::Costmap2D> > m_costmap_cache;

  double m_origin_x = 0.0;
  double m_origin_y = 0.0;
  double m_resolution = 1.0;

  int m_tile_size = 100;

  typedef std::pair<int, int> TileID;

  std::map<TileID, std::shared_ptr<costmap_2d::Costmap2D>> m_tiles;

  TileID worldToTile(double x, double y);
  void generateTile(TileID id);

};

} // namespace s57_layer

#endif

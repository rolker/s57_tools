#ifndef S57_LAYER_H
#define S57_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <s57_layer/S57LayerConfig.h>
#include <dynamic_reconfigure/server.h>

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

  bool llToWorld(double lat, double lon, double &x, double &y);
  bool worldToLatLon(double x, double y, double &lat, double &lon);

private:
  void reconfigureCallback(S57LayerConfig &config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<S57LayerConfig> > m_reconfigureServer;

  std::shared_ptr<S57Catalog> m_s57Catalog;

  std::string m_global_frame_id;

};

} // namespace s57_layer

#endif

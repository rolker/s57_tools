#include "s57_layer/s57_layer.h"
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include "s57_layer/s57_catalog.h"

PLUGINLIB_EXPORT_CLASS(s57_layer::S57Layer, costmap_2d::Layer)

namespace s57_layer
{

S57Layer::S57Layer()
{

}

void S57Layer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = false;

  std::string enc_root = ros::package::getPath("s57_layer")+"/data/ENC_ROOT";
  nh.param("enc_root", enc_root, enc_root);
  m_s57Catalog = std::shared_ptr<S57Catalog>(new S57Catalog(enc_root));

  m_reconfigureServer = std::shared_ptr<dynamic_reconfigure::Server<S57LayerConfig> >(new dynamic_reconfigure::Server<S57LayerConfig>(nh));
  m_reconfigureServer->setCallback(std::bind(&S57Layer::reconfigureCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_global_frame_id = layered_costmap_->getGlobalFrameID();

}

void S57Layer::reconfigureCallback(S57LayerConfig &config, uint32_t level)
{
  if(enabled_ != config.enabled)
  {
    enabled_ = config.enabled;
    current_ = false;
  }
}

void S57Layer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
}

void S57Layer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
}

} // namespace s57_layer

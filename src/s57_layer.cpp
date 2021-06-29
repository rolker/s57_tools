#include "s57_layer/s57_layer.h"
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include "s57_layer/s57_catalog.h"
#include "s57_layer/s57_dataset.h"
#include <tf2/utils.h>

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

  costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
  double x1 = master->getOriginX();
  double x2 = x1 + master->getResolution() * master->getSizeInCellsX();
  double y1 = master->getOriginY();
  double y2 = y1 + master->getResolution() * master->getSizeInCellsY();
  *min_x = std::min(x1, x2);
  *max_x = std::max(x1, x2);
  *min_y = std::min(y1, y2);
  *max_y = std::max(y1, y2);
}

void S57Layer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  geometry_msgs::PoseStamped world_min, world_max;
  master_grid.mapToWorld(min_i, min_j, world_min.pose.position.x, world_min.pose.position.y);
  master_grid.mapToWorld(max_i, max_j, world_max.pose.position.x, world_max.pose.position.y);
  if(tf_->canTransform("earth", m_global_frame_id, ros::Time()))
  {
    world_min.header.frame_id = m_global_frame_id;
    world_max.header.frame_id = m_global_frame_id;
    geometry_msgs::PoseStamped ecef_min, ecef_max;
    tf_->transform(world_min, ecef_min, "earth");
    tf_->transform(world_max, ecef_max, "earth");
    auto min_ll = m_s57Catalog->ecefToLatLong(ecef_min.pose.position.x, ecef_min.pose.position.y, ecef_min.pose.position.z);
    auto max_ll = m_s57Catalog->ecefToLatLong(ecef_max.pose.position.x, ecef_max.pose.position.y, ecef_max.pose.position.z);
    std::cerr << min_ll.first << ", " << min_ll.second << "  " << max_ll.first << ", " << max_ll.second << std::endl;
    auto charts = m_s57Catalog->intersectingCharts(min_ll.first, min_ll.second, max_ll.first, max_ll.second);
    std::cerr << charts.size() << " charts" << std::endl;
    for(auto c: charts)
      std::cerr << "  " << c->filePath() << std::endl;

  }
}

} // namespace s57_layer

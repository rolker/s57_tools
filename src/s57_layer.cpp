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

  nh.param("minimum_depth", m_minimum_depth, 0.0);
  nh.param("maximum_caution_depth", m_maximum_caution_depth, 5.0);
  nh.param("overhead_clearance", m_overhead_clearance, 10.0);
  
  int cost;
  nh.param("maximum_caution_cost", cost, 100);
  m_maximum_caution_cost = cost;

  nh.param("unsurveyed_cost", cost, 128);
  m_unsurveyed_cost = cost;

  std::string enc_root = ros::package::getPath("s57_layer")+"/data/ENC_ROOT";
  nh.param("enc_root", enc_root, enc_root);
  nh.param("full_resolution_distance", m_full_resolution_distance, 100.0);
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

  m_center_x = robot_x;
  m_center_y = robot_y;

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

  for(int j = min_j; j < max_j; j++)
    for(int i = min_i; i < max_i; i++)
      master_grid.setCost(i, j, costmap_2d::NO_INFORMATION);

  double base_resolution = master_grid.getResolution();

  double  world_min_x, world_min_y, world_max_x, world_max_y;
  master_grid.mapToWorld(min_i, min_j, world_min_x, world_min_y);
  master_grid.mapToWorld(max_i, max_j, world_max_x, world_max_y);
  double minLat, minLon, maxLat, maxLon;
  if(worldToLatLon(world_min_x, world_min_y, minLat, minLon) && worldToLatLon(world_max_x, world_max_y, maxLat, maxLon))
  {
    auto charts = m_s57Catalog->intersectingCharts(minLat, minLon, maxLat, maxLon);
    std::cerr << charts.size() << " charts" << std::endl;
    std::map<std::pair<double, std::string>, std::shared_ptr<costmap_2d::Costmap2D> > costmaps;
    for(auto c: charts)
    {
      double distance_to_chart = c->distanceTo(m_center_x, m_center_y, *this);
      if(isnan(distance_to_chart))
        continue;
      double target_resolution = base_resolution;
      std::cerr << "distance to " << c->label() << ": " << distance_to_chart << std::endl;
      if(distance_to_chart > m_full_resolution_distance)
        target_resolution *= distance_to_chart/m_full_resolution_distance;

      std::cerr << "  " << c->filePath() << std::endl;
      std::cerr << "    min pix size: " << c->minimumPixelSize() << " target res: " << target_resolution << std::endl;
      if(c->minimumPixelSize() >= target_resolution)
      {
        if(m_costmap_cache.find(c->label()) == m_costmap_cache.end())
          m_costmap_cache[c->label()] = c->getCosts(minLat, minLon, maxLat, maxLon, *this);
        auto costmap = m_costmap_cache[c->label()];
        if(costmap)
        {
          //costmap->saveMap(c->label()+".pgm");
          costmaps[std::make_pair(costmap->getResolution(), c->label())] = costmap;
        }
      }
    }
    for(auto cm: costmaps)
    {
      std::cerr << cm.first.second << " " << cm.first.first << std::endl;
      double half_res = cm.second->getResolution()/2.0;
      int minx, miny, maxx, maxy;
      cm.second->worldToMapEnforceBounds(world_min_x, world_min_y, minx, miny);
      cm.second->worldToMapEnforceBounds(world_max_x, world_max_y, maxx, maxy);
      for(unsigned int row = miny; row < maxy; row++)
        for(unsigned int col = minx; col < maxx; col++)
        {
          if( cm.second->getCost(col, row) == costmap_2d::NO_INFORMATION)
            continue;
          double wx, wy;
          cm.second->mapToWorld(col, row, wx, wy);
          int mx1, my1, mx2, my2;
          master_grid.worldToMapEnforceBounds(wx-half_res, wy-half_res, mx1, my1);
          master_grid.worldToMapEnforceBounds(wx+half_res, wy+half_res, mx2, my2);

          for(int my = my1; my <= my2; my++)
            for(int mx = mx1; mx <= mx2; mx++)
              if(mx >= min_i && mx < max_i && my >= min_j && my < max_j && master_grid.getCost(mx, my) == costmap_2d::NO_INFORMATION)
                master_grid.setCost(mx, my, cm.second->getCost(col, row));
        }
    }
  }
}

bool S57Layer::worldToLatLon(double x, double y, double &lat, double &lon)
{
  if(tf_->canTransform("earth", m_global_frame_id, ros::Time()))
  { 
    geometry_msgs::PoseStamped world;
    world.pose.position.x = x;
    world.pose.position.y = y;
    world.header.frame_id = m_global_frame_id;
    geometry_msgs::PoseStamped ecef;
    tf_->transform(world, ecef, "earth");
    return m_s57Catalog->ecefToLatLong(ecef.pose.position.x, ecef.pose.position.y, ecef.pose.position.z, lat, lon);
  }
  return false;
}

bool S57Layer::llToWorld(double lat, double lon, double &x, double &y)
{
  if(tf_->canTransform(m_global_frame_id, "earth", ros::Time()))
  {
    geometry_msgs::PoseStamped ecef;
    ecef.header.frame_id = "earth";
    if(m_s57Catalog->llToECEF(lat, lon, ecef.pose.position.x, ecef.pose.position.y, ecef.pose.position.z))
    {
      geometry_msgs::PoseStamped world;
      tf_->transform(ecef, world, m_global_frame_id);
      x = world.pose.position.x;
      y = world.pose.position.y;
      return true;
    }
  }
  return false;
}

double S57Layer::minimumDepth() const
{
  return m_minimum_depth;
}

double S57Layer::maximumCautionDepth() const
{
  return m_maximum_caution_depth;
}

unsigned char S57Layer::maximumCautionCost() const
{
  return m_maximum_caution_cost;
}

unsigned char S57Layer::unsurveyedCost() const
{
  return m_unsurveyed_cost;
}

double S57Layer::overheadClearance() const
{
  return m_overhead_clearance;
}

} // namespace s57_layer

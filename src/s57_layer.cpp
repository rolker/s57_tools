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
  m_s57Catalog = std::shared_ptr<S57Catalog>(new S57Catalog(enc_root));

  m_reconfigureServer = std::shared_ptr<dynamic_reconfigure::Server<S57LayerConfig> >(new dynamic_reconfigure::Server<S57LayerConfig>(nh));
  m_reconfigureServer->setCallback(std::bind(&S57Layer::reconfigureCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_global_frame_id = layered_costmap_->getGlobalFrameID();

  matchSize();

}

void S57Layer::matchSize()
{
  costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
  m_origin_x = master->getOriginX();
  m_origin_y = master->getOriginY();
  m_resolution = master->getResolution();
  m_tiles.clear();
}

void S57Layer::reconfigureCallback(S57LayerConfig &config, uint32_t level)
{
  if(enabled_ != config.enabled)
  {
    enabled_ = config.enabled;
    current_ = false;
  }
}

S57Layer::TileID S57Layer::worldToTile(double x, double y)
{
  int ix = (x - m_origin_x)/m_resolution/m_tile_size;
  int iy = (y - m_origin_y)/m_resolution/m_tile_size;
  return std::make_pair(ix, iy);
}


void S57Layer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();

  double  world_min_x, world_min_y, world_max_x, world_max_y;
  world_min_x = master->getOriginX();
  world_max_x = world_min_x+master->getSizeInMetersX();
  world_min_y = master->getOriginY();
  world_max_y = world_min_y+master->getSizeInMetersY();

  TileID start_tile = worldToTile(world_min_x, world_min_y);
  TileID end_tile = worldToTile(world_max_x, world_max_y);

  std::cerr << "start_tile: " << start_tile.first << ", " << start_tile.second << std::endl;
  std::cerr << "end_tile: " << end_tile.first << ", " << end_tile.second << std::endl;

  for(int i = start_tile.first; i <= end_tile.first; i++)
    for(int j = start_tile.second; j <= end_tile.second; j++)
      if(m_tiles.find(std::make_pair(i,j)) == m_tiles.end())
        generateTile(std::make_pair(i,j));


  *min_x = std::min(*min_x, world_min_x);
  *max_x = std::max(*max_x, world_max_x);
  *min_y = std::min(*min_y, world_min_y);
  *max_y = std::max(*max_y, world_max_y);
  std::cerr << "updateBounds: " << *min_x << ", " << *min_y << " to " << *max_x << ", " << *max_y << std::endl;
}


void S57Layer::generateTile(TileID id)
{
  std::cerr << "Generate tile: " << id.first << ", " << id.second << std::endl;

  double world_min_x = m_origin_x+id.first*m_resolution*m_tile_size;
  double world_max_x = world_min_x + m_resolution*m_tile_size;
  double world_min_y = m_origin_y+id.second*m_resolution*m_tile_size;
  double world_max_y = world_min_y + m_resolution*m_tile_size;

  double minLat, minLon, maxLat, maxLon;
  if(worldToLatLon(world_min_x, world_min_y, minLat, minLon) && worldToLatLon(world_max_x, world_max_y, maxLat, maxLon))
  {
    auto charts = m_s57Catalog->intersectingCharts(minLat, minLon, maxLat, maxLon);
    std::cerr << charts.size() << " charts" << std::endl;
    std::map<std::pair<double, std::string>, std::shared_ptr<costmap_2d::Costmap2D> > costmaps;
    for(auto c: charts)
    {
      std::cerr << "  " << c->label() << std::endl;
      std::cerr << "    chart scale: " << c->chartScale() << std::endl;
      double resolution = 0.5*c->chartScale()*0.0003125;
      std::cerr << "    resolution: " << resolution << std::endl;
      if(resolution >= m_resolution)
      {
        if(m_costmap_cache.find(c->label()) == m_costmap_cache.end())
          m_costmap_cache[c->label()] = c->getCosts(*this, resolution);
        auto costmap = m_costmap_cache[c->label()];
        if(costmap)
        {
          //costmap->saveMap(c->label()+".pgm");
          costmaps[std::make_pair(costmap->getResolution(), c->label())] = costmap;
        }
      }
    }

    std::shared_ptr<costmap_2d::Costmap2D> tile(new costmap_2d::Costmap2D(m_tile_size, m_tile_size, m_resolution, world_min_x, world_min_y, costmap_2d::NO_INFORMATION ));

    for(auto cm: costmaps)
    {
      std::cerr << cm.first.second << " " << cm.first.first << std::endl;
      double source_res = cm.second->getResolution();
      double half_res = source_res/2.0;
      int minx, miny, maxx, maxy;
      cm.second->worldToMapEnforceBounds(world_min_x, world_min_y, minx, miny);
      cm.second->worldToMapEnforceBounds(world_max_x+source_res, world_max_y+source_res, maxx, maxy);
      for(unsigned int row = miny; row < maxy; row++)
        for(unsigned int col = minx; col < maxx; col++)
        {
          if( cm.second->getCost(col, row) == costmap_2d::NO_INFORMATION)
            continue;
          double wx, wy;
          cm.second->mapToWorld(col, row, wx, wy);
          int mx1, my1, mx2, my2;
          tile->worldToMapEnforceBounds(wx-half_res, wy-half_res, mx1, my1);
          tile->worldToMapEnforceBounds(wx+half_res, wy+half_res, mx2, my2);

          for(int my = my1; my <= my2; my++)
            for(int mx = mx1; mx <= mx2; mx++)
              if(tile->getCost(mx, my) == costmap_2d::NO_INFORMATION)
                tile->setCost(mx, my, cm.second->getCost(col, row));
        }
    }

    //tile->saveMap(std::to_string(id.first)+"_"+std::to_string(id.second)+".pgm");
 
    m_tiles[id] = tile;
  }
}

void S57Layer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  double world_min_x, world_min_y, world_max_x, world_max_y;
  master_grid.mapToWorld(min_i, min_j, world_min_x, world_min_y);
  master_grid.mapToWorld(max_i, max_j, world_max_x, world_max_y);

  TileID start_tile = worldToTile(world_min_x, world_min_y);
  TileID end_tile = worldToTile(world_max_x, world_max_y);

  for(int i = start_tile.first; i <= end_tile.first; i++)
    for(int j = start_tile.second; j <= end_tile.second; j++)
    {
      TileID tile = std::make_pair(i,j);
      //std::cerr << "updateCosts: tile: " << tile.first << ", " << tile.second << std::endl;
      if(m_tiles.find(tile) != m_tiles.end())
      {
        auto current_tile = m_tiles[tile];
        int tile_offset_x = -tile.first*m_tile_size + (master_grid.getOriginX()-m_origin_x)/m_resolution;
        int tile_offset_y = -tile.second*m_tile_size + (master_grid.getOriginY()-m_origin_y)/m_resolution;
        int start_i = std::max(min_i, min_i-tile_offset_x);
        int i_count = std::min(max_i-start_i, m_tile_size);
        for(int j = std::max(min_j, min_j-tile_offset_y); j < max_j && j+tile_offset_y < m_tile_size; j++)
        {
          unsigned int source_index = current_tile->getIndex(start_i+tile_offset_x, j+tile_offset_y);
          unsigned int target_index = master_grid.getIndex(start_i, j);
          for(int i = 0; i < i_count; i++)
          {
            unsigned char cost = current_tile->getCharMap()[source_index+i];
            if(cost != costmap_2d::NO_INFORMATION)
              master_grid.getCharMap()[target_index+i]=cost;
          }
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

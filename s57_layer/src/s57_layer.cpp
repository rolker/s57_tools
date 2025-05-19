#include "s57_layer.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"
#include "marine_charts/grid_creation_context.h"
#include "marine_charts/s57_catalog.h"
#include "marine_charts/s57_dataset.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "s57_msgs/srv/get_datasets.hpp"
#include <cstdlib>


namespace s57_layer
{

S57Layer::S57Layer()
{

}

S57Layer::~S57Layer()
{
  abort_flag_ = true;
}

void S57Layer::onInitialize()
{
  auto node = node_.lock();
  current_ = false;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter("enabled", enabled_);
  // above doesn't seem to work
  enabled_ = true;

  declareParameter("minimum_depth", rclcpp::ParameterValue(m_minimum_depth));
  node->get_parameter("minimum_depth", m_minimum_depth);

  declareParameter("maximum_caution_depth", rclcpp::ParameterValue(m_maximum_caution_depth));
  node->get_parameter("maximum_caution_depth", m_maximum_caution_depth);

  declareParameter("overhead_clearance", rclcpp::ParameterValue(m_overhead_clearance));
  node->get_parameter("overhead_clearance", m_overhead_clearance);

  declareParameter("unsurveyed_cost", rclcpp::ParameterValue(m_unsurveyed_cost));
  node->get_parameter("unsurveyed_cost", m_unsurveyed_cost);

  declareParameter("update_timeout", rclcpp::ParameterValue(m_update_timeout));
  node->get_parameter("update_timeout", m_update_timeout);

  std::string enc_root;
  if(const char * enc_root_env = std::getenv("ROS_S57_ENC_ROOT"))
  {
    enc_root = enc_root_env;
    RCLCPP_INFO_STREAM(logger_, "Initializing parameter with ENC_ROOT from environment: " << enc_root);
  }
  declareParameter("enc_root", rclcpp::ParameterValue(enc_root));
  node->get_parameter("enc_root", enc_root);

  m_s57Catalog = std::make_shared<marine_charts::S57Catalog>(enc_root);


  declareParameter("tile_size", rclcpp::ParameterValue(m_tile_size));
  node->get_parameter("tile_size", m_tile_size);

  m_global_frame_id = layered_costmap_->getGlobalFrameID();

  matchSize();

}

void S57Layer::reset()
{
  //pending_grids_.clear();
  //grids_.clear();
  //m_tiles.clear();
  current_ = false;
  for(auto& t: m_tiles)
    t.second.needs_update = true;
}

void S57Layer::matchSize()
{
  auto parent = layered_costmap_->getCostmap();
  
  m_origin_x = parent->getOriginX();
  m_origin_y = parent->getOriginY();
  m_resolution = parent->getResolution();
  m_tiles.clear();
  current_ = false;
}


S57Layer::TileID S57Layer::worldToTile(double x, double y)
{
  int ix = (x - m_origin_x)/m_resolution/m_tile_size;
  int iy = (y - m_origin_y)/m_resolution/m_tile_size;
  return std::make_pair(ix, iy);
}


void S57Layer::updateBounds(double, double, double, double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  auto start_time = clock_->now();

  auto parent = layered_costmap_->getCostmap();

  double  world_min_x, world_min_y, world_max_x, world_max_y;
  world_min_x = parent->getOriginX();
  world_max_x = world_min_x + parent->getSizeInMetersX();
  world_min_y = parent->getOriginY();
  world_max_y = world_min_y + parent->getSizeInMetersY();

  TileID start_tile = worldToTile(world_min_x, world_min_y);
  TileID end_tile = worldToTile(world_max_x, world_max_y);

  bool done = false;
  int tiles_needing_update = 0;
  for(int i = start_tile.first; i <= end_tile.first && !done; i++)
    for(int j = start_tile.second; j <= end_tile.second && !done; j++)
    {
      auto id = std::make_pair(i,j);
      if(!m_tiles[id].complete)
        generateTile(id);
      if(m_tiles[id].needs_update)
      {
        tiles_needing_update += 1;
        double tile_min_x = m_origin_x+id.first*m_resolution*m_tile_size;
        double tile_max_x = tile_min_x + m_resolution*m_tile_size;
        double tile_min_y = m_origin_y+id.second*m_resolution*m_tile_size;
        double tile_max_y = tile_min_y + m_resolution*m_tile_size;

        *min_x = std::min(*min_x, tile_min_x-m_resolution);
        *max_x = std::max(*max_x, tile_max_x+m_resolution);
        *min_y = std::min(*min_y, tile_min_y-m_resolution);
        *max_y = std::max(*max_y, tile_max_y+m_resolution);
      }
      if(clock_->now() - start_time > rclcpp::Duration::from_seconds(m_update_timeout))
        done = true;
    }

  if(tiles_needing_update > 0)
  {
    *min_x = std::min(*min_x, world_min_x);
    *max_x = std::max(*max_x, world_max_x);
    *min_y = std::min(*min_y, world_min_y);
    *max_y = std::max(*max_y, world_max_y);
    
  }

}


void S57Layer::generateTile(TileID id)
{
  double world_min_x = m_origin_x+id.first*m_resolution*m_tile_size;
  double world_max_x = world_min_x + m_resolution*m_tile_size;
  double world_min_y = m_origin_y+id.second*m_resolution*m_tile_size;
  double world_max_y = world_min_y + m_resolution*m_tile_size;

  double minLat, minLon, maxLat, maxLon;
  if(worldToLatLon(world_min_x, world_min_y, minLat, minLon) && worldToLatLon(world_max_x, world_max_y, maxLat, maxLon))
  {
    auto charts = m_s57Catalog->intersectingCharts(minLat, minLon, maxLat, maxLon);

    // This map is used to sort the grids by resolution. It allows us to use the highest
    // resolution chart that is not higher than the parent costmap's resolution.
    std::map<std::pair<double, std::string>, std::shared_ptr<grid_map::GridMap> > grids;
    bool all_charts_available = true;
    for(auto c: charts)
    {
      double resolution = 0.5*c->chartScale()*0.0003125;
      if(resolution >= m_resolution)
      {
        std::string chart = c->label();
        auto grid = grids_[chart];
        if(!grid)
        {
          if(pending_grids_.count(chart) == 0)
          {
            RCLCPP_INFO_STREAM(logger_, "async call to getGrid for " << chart << " scale: " << c->chartScale() << " resolution: " << 0.5*c->chartScale()*0.0003125);
            auto context = marine_charts::GridCreationContext(m_global_frame_id, *tf_, 1.0, logger_);
            pending_grids_[chart] = std::async(&marine_charts::S57Dataset::getGrid, c.get(), context, std::ref(abort_flag_));
          }
          auto status = pending_grids_[chart].wait_for(std::chrono::milliseconds(10));
          if(status == std::future_status::ready)
          {
            RCLCPP_INFO_STREAM(logger_, "got results from getCosts for " << chart);
            grid = pending_grids_[chart].get();
            grids_[chart] = grid;
            pending_grids_.erase(chart);
          }
        }
        if(grid)
        {
          grids[std::make_pair(grid->getResolution(), c->label())] = grid;
        }
        else
          all_charts_available = false;
      }

    }
    m_tiles[id].complete = all_charts_available;
    if(grids.size() > m_tiles[id].chart_count)
    {
      auto tile = std::make_shared<nav2_costmap_2d::Costmap2D>(m_tile_size, m_tile_size, m_resolution, world_min_x, world_min_y, nav2_costmap_2d::NO_INFORMATION);

      grid_map::Position tile_center(world_min_x + m_resolution*m_tile_size/2.0, world_min_y + m_resolution*m_tile_size/2.0);
      grid_map::Length tile_size(m_resolution*m_tile_size, m_resolution*m_tile_size);

      for(auto grid: grids)
      {
        auto grid_half_resolution = grid.second->getResolution()/2.0;
        bool submap_ok = false;
        grid_map::SubmapGeometry overlap(*grid.second, tile_center, tile_size, submap_ok);
        if(submap_ok)
        {
          for(grid_map::SubmapIterator smi(overlap); !smi.isPastEnd(); ++smi)
          {
            grid_map::Index index = *smi;
            auto cost = get_cost_from_grid(*grid.second, index);
            if(cost == nav2_costmap_2d::NO_INFORMATION)
              continue;
            grid_map::Position cell_position;
            if(grid.second->getPosition(index, cell_position))
            {
              int tile_min_x, tile_min_y, tile_max_x, tile_max_y;
              tile->worldToMapEnforceBounds(
                cell_position.x()-grid_half_resolution,
                cell_position.y()-grid_half_resolution,
                tile_min_x, tile_min_y
              );
              tile->worldToMapEnforceBounds(
                cell_position.x()+grid_half_resolution,
                cell_position.y()+grid_half_resolution,
                tile_max_x, tile_max_y
              );
              for(int tile_y = tile_min_y; tile_y <= tile_max_y; tile_y++)
              {
                for(int tile_x = tile_min_x; tile_x <= tile_max_x; tile_x++)
                {
                  if(tile->getCost(tile_x, tile_y) == nav2_costmap_2d::NO_INFORMATION)
                  {
                    tile->setCost(tile_x, tile_y, cost);
                  }
                }
              }
            }
          }
        }

      }
      m_tiles[id].costmap = tile;
      m_tiles[id].needs_update = true;
    }
  }
}

void S57Layer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  double world_min_x, world_min_y, world_max_x, world_max_y;
  master_grid.mapToWorld(min_i, min_j, world_min_x, world_min_y);
  master_grid.mapToWorld(max_i, max_j, world_max_x, world_max_y);

  TileID start_tile = worldToTile(world_min_x, world_min_y);
  TileID end_tile = worldToTile(world_max_x, world_max_y);

  bool complete = true;

  for(int ti = start_tile.first; ti <= end_tile.first; ti++)
  {
    int tile_offset_x = -ti*m_tile_size + (master_grid.getOriginX()-m_origin_x)/m_resolution;
    int start_i = std::max(min_i, -tile_offset_x);
    int i_count = std::min(max_i, m_tile_size-tile_offset_x)-start_i;
    for(int tj = start_tile.second; tj <= end_tile.second; tj++)
    {
      int tile_offset_y = -tj*m_tile_size + (master_grid.getOriginY()-m_origin_y)/m_resolution;

      TileID tile = std::make_pair(ti,tj);
      auto current_tile = m_tiles[tile].costmap;
      complete = complete && m_tiles[tile].complete;
      for(int j = std::max(min_j, -tile_offset_y); j < max_j && j+tile_offset_y < m_tile_size; j++)
      {
        unsigned int target_index = master_grid.getIndex(start_i, j);
        if(current_tile)
        {
          unsigned int source_index = current_tile->getIndex(start_i+tile_offset_x, j+tile_offset_y);
          for(int i = 0; i < i_count; i++)
          {
            unsigned char cost = current_tile->getCharMap()[source_index+i];
            if(cost != nav2_costmap_2d::NO_INFORMATION)
              master_grid.getCharMap()[target_index+i]=cost;
          }
        }
        else
        {
          for(int i = 0; i < i_count; i++)
            master_grid.getCharMap()[target_index+i]=nav2_costmap_2d::NO_INFORMATION;

        }
      }
      m_tiles[tile].needs_update = false;
    }
  }
  current_ = complete;
}

unsigned char S57Layer::get_cost_from_grid(grid_map::GridMap &grid, const grid_map::Index &index)
{
  if(!std::isnan(grid.at("restricted", index)))
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  auto overhead = grid.at("overhead", index);
  if(!std::isnan(overhead) && overhead < m_overhead_clearance)
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  auto elevation = grid.at("elevation", index);
  if(!std::isnan(elevation))
  {
    auto depth = -elevation;
    if (depth < m_minimum_depth)
      return nav2_costmap_2d::LETHAL_OBSTACLE;
    unsigned char cost = nav2_costmap_2d::FREE_SPACE;
    if(depth < m_maximum_caution_depth)
      cost = nav2_costmap_2d::MAX_NON_OBSTACLE*(1.0-((depth-m_minimum_depth)/(m_maximum_caution_depth-m_minimum_depth)));
    if(!std::isnan(grid.at("unsurveyed", index)) || !std::isnan(grid.at("caution", index)))
      cost = std::max(cost, m_unsurveyed_cost);
    return cost;
  }
  return nav2_costmap_2d::NO_INFORMATION;
}

bool S57Layer::worldToLatLon(double x, double y, double &lat, double &lon)
{
  if(tf_->canTransform("earth", m_global_frame_id, tf2::TimePointZero))
  { 
    geometry_msgs::msg::PoseStamped world;
    world.pose.position.x = x;
    world.pose.position.y = y;
    world.header.frame_id = m_global_frame_id;
    geometry_msgs::msg::PoseStamped ecef;
    tf_->transform(world, ecef, "earth");
    return m_s57Catalog->ecefToLatLong(ecef.pose.position.x, ecef.pose.position.y, ecef.pose.position.z, lat, lon);
  }
  return false;
}

bool S57Layer::llToWorld(double lat, double lon, double &x, double &y)
{
  if(tf_->canTransform(m_global_frame_id, "earth", tf2::TimePointZero))
  {
    geometry_msgs::msg::PoseStamped ecef;
    ecef.header.frame_id = "earth";
    if(m_s57Catalog->llToECEF(lat, lon, ecef.pose.position.x, ecef.pose.position.y, ecef.pose.position.z))
    {
      geometry_msgs::msg::PoseStamped world;
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

unsigned char S57Layer::unsurveyedCost() const
{
  return m_unsurveyed_cost;
}

double S57Layer::overheadClearance() const
{
  return m_overhead_clearance;
}

} // namespace s57_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(s57_layer::S57Layer, nav2_costmap_2d::Layer)

#include "s57_layer.h"

#include "geodesy/ecef.h"
#include "geodesy/wgs84.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "s57_msgs/srv/get_datasets.hpp"
#include <cstdlib>

using namespace std::chrono_literals;

namespace s57_layer
{

S57Layer::S57Layer()
{

}

S57Layer::~S57Layer()
{
}

void S57Layer::onInitialize()
{
  auto node = node_.lock();
  current_ = false;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_+".enabled", enabled_);
  // above doesn't seem to work
  // enabled_ = true;

  declareParameter("minimum_depth", rclcpp::ParameterValue(minimum_depth_));
  node->get_parameter(name_+".minimum_depth", minimum_depth_);

  declareParameter("maximum_caution_depth", rclcpp::ParameterValue(maximum_caution_depth_));
  node->get_parameter(name_+".maximum_caution_depth", maximum_caution_depth_);

  declareParameter("overhead_clearance", rclcpp::ParameterValue(overhead_clearance_));
  node->get_parameter(name_+".overhead_clearance", overhead_clearance_);

  declareParameter("unsurveyed_cost", rclcpp::ParameterValue(unsurveyed_cost_));
  node->get_parameter(name_+".unsurveyed_cost", unsurveyed_cost_);

  declareParameter("update_timeout", rclcpp::ParameterValue(update_timeout_));
  node->get_parameter(name_+".update_timeout", update_timeout_);
  declareParameter("tile_size", rclcpp::ParameterValue(tile_size_));
  node->get_parameter(name_+".tile_size", tile_size_);

  declareParameter("allow_uncharted", rclcpp::ParameterValue(allow_uncharted_));
  node->get_parameter(name_+".allow_uncharted", allow_uncharted_);

  declareParameter("chart_datum_frame", rclcpp::ParameterValue(chart_datum_frame_));
  node->get_parameter(name_+".chart_datum_frame", chart_datum_frame_);
  if(!chart_datum_frame_.empty())
    RCLCPP_INFO_STREAM(logger_, "Tide correction enabled: chart_datum_frame = " << chart_datum_frame_);

  declareParameter("s57_grids_namespace", rclcpp::ParameterValue(s57_grids_namespace_));
  node->get_parameter(name_+".s57_grids_namespace", s57_grids_namespace_);

  if(!s57_grids_namespace_.empty() && s57_grids_namespace_.back() != '/')
  {
    s57_grids_namespace_ += '/';
  }
  RCLCPP_INFO_STREAM(logger_, "Using S57 grids namespace: " << s57_grids_namespace_);
  RCLCPP_INFO_STREAM(logger_, "allow_uncharted: " << (allow_uncharted_ ? "true" : "false"));

  std::string service_name = s57_grids_namespace_ + "get_datasets";


  declareParameter("get_datasets_service", rclcpp::ParameterValue(service_name));
  node->get_parameter(name_+".get_datasets_service", service_name);
  RCLCPP_INFO_STREAM(logger_, "Using get datasets service: " << service_name);
  get_datasets_client_ = node->create_client<s57_msgs::srv::GetDatasets>(service_name);


  global_frame_id_ = layered_costmap_->getGlobalFrameID();
  buffered_min_.header.frame_id = global_frame_id_;
  buffered_max_.header.frame_id = global_frame_id_;

  matchSize();

}

void S57Layer::reset()
{
  //pending_grids_.clear();
  //grids_.clear();
  //tiles_.clear();
  current_ = false;
  for(auto& t: tiles_)
    t.second.needs_update = true;
}

void S57Layer::matchSize()
{
  auto parent = layered_costmap_->getCostmap();

  x_origin_ = parent->getOriginX();
  y_origin_ = parent->getOriginY();
  resolution_ = parent->getResolution();
  tiles_.clear();
  current_ = false;
}


S57Layer::TileID S57Layer::worldToTile(double x, double y)
{
  int ix = (x - x_origin_)/resolution_/tile_size_;
  int iy = (y - y_origin_)/resolution_/tile_size_;
  return std::make_pair(ix, iy);
}


void S57Layer::updateBounds(double, double, double, double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  if(!chart_datum_frame_.empty())
  {
    try
    {
      auto transform = tf_->lookupTransform(global_frame_id_, chart_datum_frame_, tf2::TimePointZero);
      double new_offset = -transform.transform.translation.z;
      if(std::abs(new_offset - tide_offset_) > 0.01)
      {
        tide_offset_ = new_offset;
        RCLCPP_INFO_STREAM(logger_, "Tide offset updated: " << tide_offset_ << " m (water above chart datum)");
        for(auto& t: tiles_)
          t.second.complete = false;
      }
    }
    catch(const tf2::TransformException& e)
    {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 10000, "Cannot look up tide offset (%s → %s): %s",
        chart_datum_frame_.c_str(), global_frame_id_.c_str(), e.what());
    }
  }

  auto start_time = clock_->now();

  // Figure out the bounds of the parent costmap
  auto parent = layered_costmap_->getCostmap();

  double  world_min_x, world_min_y, world_max_x, world_max_y;
  world_min_x = parent->getOriginX();
  world_max_x = world_min_x + parent->getSizeInMetersX();
  world_min_y = parent->getOriginY();
  world_max_y = world_min_y + parent->getSizeInMetersY();

  // 5% buffer around the world bounds
  double buffer = std::max(world_max_x-world_min_x, world_max_y-world_min_y)*0.05;

  // use two layers of buffer. inner buffer to make sure data is available
  // and an outer buffer to limit the number of service calls to the get data service
  bool bounds_need_update = false;

  if( world_min_x - buffer < buffered_min_.point.x ||
      world_max_x + buffer > buffered_max_.point.x ||
      world_min_y - buffer < buffered_min_.point.y ||
      world_max_y + buffer > buffered_max_.point.y)
  {
    bounds_need_update = true;
  }

  if(pending_datasets_request_)
  {
    std::vector<int64_t> pruned_requests;
    // Prune all requests older than 5s.
    size_t n_pruned = get_datasets_client_->prune_requests_older_than(
      std::chrono::system_clock::now() - 5s,
      &pruned_requests
    );
    if (n_pruned)
    {
      RCLCPP_WARN_STREAM(
        logger_,
        "The get datasets server hasn't replied for more than 5s, " <<n_pruned << " requests were discarded, the discarded requests numbers are:");
      for (const auto & req_num : pruned_requests) {
        RCLCPP_WARN_STREAM(logger_, "\t" << req_num);
      }
      pending_datasets_request_ = false;
    }
  }

  if(bounds_need_update)
  {
    if(pending_datasets_request_)
    {
      RCLCPP_WARN_STREAM(logger_, "A request to the get datasets service is already pending and bounds already need an update. Not sending a new request.");

    }
    else
    {
      if(!get_datasets_client_->service_is_ready())
      {
        RCLCPP_WARN_STREAM(logger_, "The get datasets service is not ready, not sending a request.");
      }
      else
      {
        // update the buffered bounds adding both inner and outer buffer
        geometry_msgs::msg::PointStamped new_min, new_max;
        new_min.header.frame_id = global_frame_id_;
        new_max.header.frame_id = global_frame_id_;

        new_min.point.x = world_min_x - 2.0*buffer;
        new_min.point.y = world_min_y - 2.0*buffer;
        new_max.point.x = world_max_x + 2.0*buffer;
        new_max.point.y = world_max_y + 2.0*buffer;

        try
        {
          auto request = std::make_shared<s57_msgs::srv::GetDatasets::Request>();
          request->bounds.min_pt = worldToLatLon(new_min.point.x, new_min.point.y);
          request->bounds.max_pt = worldToLatLon(new_max.point.x, new_max.point.y);
          request->minimum_scale = 2*resolution_/0.0003125; // See S57Dataset::recommendedResolution for scale calculation

          get_datasets_client_->async_send_request(
            request,
            std::bind(&S57Layer::getDatasetsCallback, this, std::placeholders::_1)
          );
          pending_datasets_request_ = true;
          buffered_min_ = new_min;
          buffered_max_ = new_max;
        }
        catch(const std::exception& e)
        {
          RCLCPP_ERROR_STREAM(logger_, "Failed to send request to get datasets service: " << e.what());
        }
      }

    }
  }


  TileID start_tile = worldToTile(world_min_x, world_min_y);
  TileID end_tile = worldToTile(world_max_x, world_max_y);

  bool done = false;
  int tiles_needing_update = 0;
  for(int i = start_tile.first; i <= end_tile.first && !done; i++)
    for(int j = start_tile.second; j <= end_tile.second && !done; j++)
    {
      auto id = std::make_pair(i,j);
      if(!tiles_[id].complete)
        generateTile(id);
      if(tiles_[id].needs_update)
      {
        tiles_needing_update += 1;
        double tile_min_x = x_origin_+id.first*resolution_*tile_size_;
        double tile_max_x = tile_min_x + resolution_*tile_size_;
        double tile_min_y = y_origin_+id.second*resolution_*tile_size_;
        double tile_max_y = tile_min_y + resolution_*tile_size_;

        *min_x = std::min(*min_x, tile_min_x-resolution_);
        *max_x = std::max(*max_x, tile_max_x+resolution_);
        *min_y = std::min(*min_y, tile_min_y-resolution_);
        *max_y = std::max(*max_y, tile_max_y+resolution_);
      }
      if(clock_->now() - start_time > rclcpp::Duration::from_seconds(update_timeout_))
        done = true;
    }

  if(tiles_needing_update > 0)
  {
    *min_x = std::max(*min_x, world_min_x);
    *max_x = std::min(*max_x, world_max_x);
    *min_y = std::max(*min_y, world_min_y);
    *max_y = std::min(*max_y, world_max_y);
  }

}


void S57Layer::generateTile(TileID id)
{
  double world_min_x = x_origin_+id.first*resolution_*tile_size_;
  double world_max_x = world_min_x + resolution_*tile_size_;
  double world_min_y = y_origin_+id.second*resolution_*tile_size_;
  double world_max_y = world_min_y + resolution_*tile_size_;

  // No chart coverage for this area — handle as uncharted.
  if(current_charts_.empty())
  {
    tiles_[id].complete = true;
    // No tile costmap is created. In updateCosts, a nullptr tile
    // with complete=true is recognized as uncharted territory and
    // handled according to allow_uncharted_.
    return;
  }

  // This map is used to sort the grids by resolution. It allows us to use the highest
  // resolution chart that is not higher than the parent costmap's resolution.
  std::map<std::pair<double, std::string>, std::shared_ptr<grid_map::GridMap> > grids;
  bool all_charts_available = true;
  for(auto& c: current_charts_)
  {
      std::string chart = c.label;
      if(chart_bounds_.count(chart) == 0)
      {
        RCLCPP_WARN_STREAM(logger_, "Chart " << chart << " not found in chart bounds, skipping.");
        continue;
      }
      auto bounds = chart_bounds_[chart];
      if(bounds.first.x > world_max_x || bounds.second.x < world_min_x ||
         bounds.first.y > world_max_y || bounds.second.y < world_min_y)
      {
        RCLCPP_DEBUG_STREAM(logger_, "Chart " << chart << " is not in the tile bounds, skipping.");
        continue;
      }
      auto grid = grids_[chart];
      if(grid)
      {
        grids[std::make_pair(grid->getResolution(), chart)] = grid;
      }
      else
        all_charts_available = false;
  }
  tiles_[id].complete = all_charts_available;
  if(grids.size() > tiles_[id].chart_count)
  {
    auto tile = std::make_shared<nav2_costmap_2d::Costmap2D>(tile_size_, tile_size_, resolution_, world_min_x, world_min_y, nav2_costmap_2d::NO_INFORMATION);

    grid_map::Position tile_center(world_min_x + resolution_*tile_size_/2.0, world_min_y + resolution_*tile_size_/2.0);
    grid_map::Length tile_size(resolution_*tile_size_, resolution_*tile_size_);

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
    tiles_[id].costmap = tile;
    tiles_[id].needs_update = true;
    tiles_[id].chart_count = grids.size();
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
    int tile_offset_x = -ti*tile_size_ + (master_grid.getOriginX()-x_origin_)/resolution_;
    int start_i = std::max(min_i, -tile_offset_x);
    int i_count = std::min(max_i, tile_size_-tile_offset_x)-start_i;
    for(int tj = start_tile.second; tj <= end_tile.second; tj++)
    {
      int tile_offset_y = -tj*tile_size_ + (master_grid.getOriginY()-y_origin_)/resolution_;

      TileID tile = std::make_pair(ti,tj);
      auto current_tile = tiles_[tile].costmap;
      complete = complete && tiles_[tile].complete;
      if(current_tile)
      {
        // Chart data available — copy costs to master costmap.
        for(int j = std::max(min_j, -tile_offset_y); j < max_j && j+tile_offset_y < tile_size_; j++)
        {
          unsigned int target_index = master_grid.getIndex(start_i, j);
          unsigned int source_index = current_tile->getIndex(start_i+tile_offset_x, j+tile_offset_y);
          for(int i = 0; i < i_count; i++)
          {
            unsigned char cost = current_tile->getCharMap()[source_index+i];
            if(cost != nav2_costmap_2d::NO_INFORMATION)
              master_grid.getCharMap()[target_index+i]=cost;
          }
        }
      }
      else if(!allow_uncharted_ || !tiles_[tile].complete)
      {
        // Either uncharted navigation is disallowed, or chart data
        // exists but hasn't loaded yet — mark cells as unknown.
        for(int j = std::max(min_j, -tile_offset_y); j < max_j && j+tile_offset_y < tile_size_; j++)
        {
          unsigned int target_index = master_grid.getIndex(start_i, j);
          for(int i = 0; i < i_count; i++)
            master_grid.getCharMap()[target_index+i]=nav2_costmap_2d::NO_INFORMATION;
        }
      }
      // else: uncharted + allow_uncharted — leave master costmap untouched.
      tiles_[tile].needs_update = false;
    }
  }
  current_ = complete;
}

unsigned char S57Layer::get_cost_from_grid(grid_map::GridMap &grid, const grid_map::Index &index)
{
  if(!std::isnan(grid.at("restricted", index)))
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  auto overhead = grid.at("overhead", index);
  if(!std::isnan(overhead) && overhead < overhead_clearance_)
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  auto elevation = grid.at("elevation", index);
  if(!std::isnan(elevation))
  {
    auto depth = -elevation + tide_offset_;
    if (depth < minimum_depth_)
      return nav2_costmap_2d::LETHAL_OBSTACLE;
    unsigned char cost = nav2_costmap_2d::FREE_SPACE;
    if(depth < maximum_caution_depth_)
      cost = nav2_costmap_2d::MAX_NON_OBSTACLE*(1.0-((depth-minimum_depth_)/(maximum_caution_depth_-minimum_depth_)));
    if(!std::isnan(grid.at("unsurveyed", index)) || !std::isnan(grid.at("caution", index)))
      cost = std::max(cost, unsurveyed_cost_);
    return cost;
  }
  return nav2_costmap_2d::NO_INFORMATION;
}

geographic_msgs::msg::GeoPoint S57Layer::worldToLatLon(double x, double y)
{
  geometry_msgs::msg::PointStamped world;
  world.point.x = x;
  world.point.y = y;
  world.header.frame_id = global_frame_id_;
  geometry_msgs::msg::PointStamped ecef;
  tf_->transform(world, ecef, "earth");
  geodesy::ECEFPoint ecef_point(ecef.point);
  return geodesy::toMsg(ecef_point);
}

geometry_msgs::msg::Point S57Layer::llToWorld(const geographic_msgs::msg::GeoPoint& geo_point)
{
  geometry_msgs::msg::PointStamped ecef;
  ecef.header.frame_id = "earth";
  ecef.point = toGeometry(geodesy::ECEFPoint(geo_point));

  geometry_msgs::msg::PointStamped world;
  tf_->transform(ecef, world, global_frame_id_);
  return world.point;
}

void S57Layer::getDatasetsCallback(GetDatasetsClient::SharedFuture future)
{
  if(!future.valid())
  {
    RCLCPP_ERROR_STREAM(logger_, "Invalid future in getDatasetsCallback");
    return;
  }
  auto result = future.get();
  if(!result)
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to get datasets from service");
    pending_datasets_request_ = false;
    return;
  }

  if(result->datasets.empty())
  {
    RCLCPP_WARN_STREAM(logger_,
      "No chart coverage for current area (0 datasets from service). "
      "allow_uncharted is " << (allow_uncharted_ ? "true" : "false") << ".");
  }
  else
  {
    RCLCPP_DEBUG_STREAM(logger_, "Received " << result->datasets.size() << " datasets from service");
  }

  current_charts_ = result->datasets;
  chart_bounds_.clear();

  for(const auto &dataset_info: result->datasets)
  {
    if (grid_subscriptions_[dataset_info.label] == nullptr)
    {
      rclcpp::QoS latched_qos(1);
      latched_qos.transient_local();
      latched_qos.keep_last(1);

      auto grid_sub = node_.lock()->create_subscription<grid_map_msgs::msg::GridMap>(
        s57_grids_namespace_+"datasets/"+dataset_info.topic,
        latched_qos,
        [this, label=dataset_info.label](const grid_map_msgs::msg::GridMap::SharedPtr msg)
        {
          this->mapGridCallback(label, msg);
        }
      );
      grid_subscriptions_[dataset_info.label] = grid_sub;
    }
    auto min_point = llToWorld(dataset_info.bounds.min_pt);
    auto max_point = llToWorld(dataset_info.bounds.max_pt);
    chart_bounds_[dataset_info.label] = std::make_pair(min_point, max_point);
  }

  pending_datasets_request_ = false;

  // prune outdated subscriptions
  std::vector<std::string> outdated_subscriptions;
  for(const auto & sub: grid_subscriptions_)
  {
    bool found = false;
    for(const auto & dataset_info: current_charts_)
    {
      if(sub.first == dataset_info.label)
      {
        found = true;
        break;
      }
    }
    if(!found)
      outdated_subscriptions.push_back(sub.first);
  }
  for(const auto & label: outdated_subscriptions)
  {
    grid_subscriptions_.erase(label);
  }

  // prune outdated grids
  std::vector<std::string> outdated_grids;
  for (const auto & grid: grids_)
  {
    bool found = false;
    for(const auto & dataset_info: current_charts_)
    {
      if(grid.first == dataset_info.label)
      {
        found = true;
        break;
      }
    }
    if(!found)
      outdated_grids.push_back(grid.first);
  }
  for (const auto & grid: outdated_grids)
  {
    grids_.erase(grid);
  }

  // mark all tiles as needing regeneration
  for(auto& t: tiles_)
    t.second.complete = false;

}

void S57Layer::mapGridCallback(std::string grid_name, std::shared_ptr<grid_map_msgs::msg::GridMap> grid_map)
{
  RCLCPP_INFO_STREAM(logger_, "Received grid map for grid: " << grid_name);
  auto grid = std::make_shared<grid_map::GridMap>();
  if(!grid_map::GridMapRosConverter::fromMessage(*grid_map, *grid))
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to convert grid map from message for grid: " << grid_name);
    return;
  }
  if(grid->getLayers().empty())
  {
    RCLCPP_ERROR_STREAM(logger_, "Grid map for grid " << grid_name << " has no layers, skipping.");
    return;
  }
  if(grid->getResolution() <= 0.0)
  {
    RCLCPP_ERROR_STREAM(logger_, "Grid map for grid " << grid_name << " has invalid resolution, skipping.");
    return;
  }
  if(grid->getLength().x() <= 0.0 || grid->getLength().y() <= 0.0)
  {
    RCLCPP_ERROR_STREAM(logger_, "Grid map for grid " << grid_name << " has invalid size, skipping.");
    return;
  }
  grids_[grid_name] = grid;
}

} // namespace s57_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(s57_layer::S57Layer, nav2_costmap_2d::Layer)

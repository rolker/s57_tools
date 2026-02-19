#include "grid_publisher.h"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "marine_charts/s57_dataset.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <thread>

namespace s57_grids
{

GridPublisher::GridPublisher(const std::string &node_name)
  : rclcpp_lifecycle::LifecycleNode(node_name)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GridPublisher::on_configure(const rclcpp_lifecycle::State &state)
{
  std::string enc_root;
  if(!has_parameter("enc_root"))
  {
    if(const char * enc_root_env = std::getenv("ROS_S57_ENC_ROOT"))
    {
      enc_root = enc_root_env;
      RCLCPP_INFO_STREAM(get_logger(), "Initializing parameter with ENC_ROOT from environment: " << enc_root);
    }
    declare_parameter("enc_root", rclcpp::ParameterValue(enc_root));
  }
  get_parameter("enc_root", enc_root);

  catalog_ = std::make_shared<marine_charts::S57Catalog>(enc_root);

  if(!has_parameter("resolution_factor"))
  {
    declare_parameter("resolution_factor", rclcpp::ParameterValue(resolution_factor_));
  }
  resolution_factor_ = get_parameter("resolution_factor").as_double();

  if(!has_parameter("map_frame"))
  {
    declare_parameter("map_frame", rclcpp::ParameterValue(map_frame_));
  }
  map_frame_ = get_parameter("map_frame").as_string();

  if(!has_parameter("grid_republish_period"))
  {
    declare_parameter("grid_republish_period", rclcpp::ParameterValue(grid_republish_period_));
  }
  grid_republish_period_ = get_parameter("grid_republish_period").as_double();
  if(grid_republish_period_ > 0.0)
  {
    republish_grids_timer_ = create_wall_timer(
      std::chrono::milliseconds(int(1000.0*grid_republish_period_)),
      std::bind(&GridPublisher::republishGrids, this)
    );
  }

  if(!has_parameter("publish_costmaps"))
  {
    declare_parameter("publish_costmaps", rclcpp::ParameterValue(publish_costmaps_));
  }
  publish_costmaps_ = get_parameter("publish_costmaps").as_bool();

  list_service_ = create_service<s57_msgs::srv::GetDatasets>(
    "list_datasets",
    std::bind(&GridPublisher::listDatasets, this, std::placeholders::_1, std::placeholders::_2)
  );

  get_service_ = create_service<s57_msgs::srv::GetDatasets>(
    "get_datasets",
    std::bind(&GridPublisher::getDatasets, this, std::placeholders::_1, std::placeholders::_2)
  );

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  new_grids_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&GridPublisher::checkForNewGrids, this)
  );


  return LifecycleNode::on_configure(state);

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GridPublisher::on_activate(const rclcpp_lifecycle::State &state)
{
  for(auto& g: grid_publishers_)
    g.second->on_activate();

  return LifecycleNode::on_activate(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GridPublisher::on_cleanup(const rclcpp_lifecycle::State &state)
{
  abort_flag_ = true;

  list_service_.reset();
  get_service_.reset();

  new_grids_timer_.reset();

  catalog_.reset();
  for(auto& p: pending_dataset_grids_)
    if(p.second.valid())
      p.second.wait();

  tf_listener_.reset();
  tf_buffer_.reset();

  return LifecycleNode::on_cleanup(state);
}


void GridPublisher::listDatasets(
  s57_msgs::srv::GetDatasets::Request::ConstSharedPtr request,
  s57_msgs::srv::GetDatasets::Response::SharedPtr response)
{
  for(auto d: catalog_->intersectingCharts(request->bounds))
    if(d->chartScale() > request->minimum_scale)
    {
      s57_msgs::msg::DatasetInfo di;
      di.bounds = d->getBounds();
      di.label = d->label();
      di.scale = d->chartScale();
      di.resolution = d->recommendedResolution();
      di.topic = d->topic();
      response->datasets.push_back(di);
    }
}

void GridPublisher::getDatasets(
  s57_msgs::srv::GetDatasets::Request::ConstSharedPtr request,
  s57_msgs::srv::GetDatasets::Response::SharedPtr response)
{
  listDatasets(request, response);
  std::lock_guard<std::mutex> lock(requested_grids_mutex);
  for(const auto& d: response->datasets)
  {
    requested_grids_.push_back(d.label);
    requested_grids_to_publish_.push_back(d.label);
  }
}

void GridPublisher::checkForNewGrids()
{
  {
    std::lock_guard<std::mutex> lock(requested_grids_mutex);
    for(auto r: requested_grids_)
    {
      auto ds = catalog_->dataset(r);
      if(grid_publishers_.count(ds->label()) == 0 && pending_dataset_grids_.count(ds->label()) == 0)
      {
        pending_dataset_grids_[ds->label()] = std::async(
          &marine_charts::S57Dataset::getGrid,
          ds,
          marine_charts::GridCreationContext(map_frame_, *tf_buffer_, resolution_factor_, get_logger()),
          std::ref(abort_flag_)
        );
      }
    }
    requested_grids_.clear();
  }

  std::vector<std::string> done_grids;
  for(auto& pg: pending_dataset_grids_)
  {
    if(pg.second.valid())
      if(pg.second.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
      {
        auto grid = pg.second.get();
        {
          std::lock_guard<std::mutex> lock(requested_grids_mutex);
          if(std::find(requested_grids_to_publish_.begin(), requested_grids_to_publish_.end(), pg.first) != requested_grids_to_publish_.end())
          {
            auto ds = catalog_->dataset(pg.first);

            rclcpp::QoS latched_qos(1);
            latched_qos.transient_local();
            latched_qos.keep_last(1);

            grid_publishers_[pg.first] =
              create_publisher<grid_map_msgs::msg::GridMap>(
                "datasets/"+ds->topic(), latched_qos);
            grid_publishers_[pg.first]->on_activate();

            auto message = grid_map::GridMapRosConverter::toMessage(*grid);
            RCLCPP_DEBUG_STREAM(get_logger(), "Publishing grid to " << "datasets/" << pg.first);
            grid_publishers_[pg.first]->publish(*message);

            if(publish_costmaps_)
            {
              costmap_publishers_[pg.first] =
                create_publisher<nav_msgs::msg::OccupancyGrid>(
                  "datasets/occupancy_grids/"+ds->topic(), latched_qos);
              costmap_publishers_[pg.first]->on_activate();
              nav_msgs::msg::OccupancyGrid occupancy_grid;
              grid_map::GridMapRosConverter::toOccupancyGrid(*grid, "elevation", -10.0, 0.0, occupancy_grid);
              costmap_publishers_[pg.first]->publish(occupancy_grid);
            }
          }
          else
            grid_publishers_[pg.first]; // create the entry in the map so above check to see if we need to generate a grid works.
        }

        done_grids.push_back(pg.first);

        std::lock_guard<std::mutex> lock(dataset_grids_mutex_);
        dataset_grids_[pg.first] = grid;
      }
  }
  for(auto d: done_grids)
    pending_dataset_grids_.erase(d);

  done_grids.clear();

}

void GridPublisher::republishGrids()
{
  RCLCPP_INFO_STREAM(get_logger(), "Republishing grids");
  for(const auto &grid_publisher: grid_publishers_)
  {
    if(grid_publisher.second && grid_publisher.second->is_activated())
    {
      std::lock_guard<std::mutex> lock(dataset_grids_mutex_);
      const auto &it = dataset_grids_.find(grid_publisher.first);
      if(it != dataset_grids_.end())
      {
        auto grid = it->second;
        if(grid)
        {
          grid->setTimestamp(get_clock()->now().nanoseconds());
          auto message = grid_map::GridMapRosConverter::toMessage(*grid);
          grid_publisher.second->publish(*message);
          RCLCPP_INFO_STREAM(get_logger(), "Republishing grid to " << "datasets/" << grid_publisher.first);
        }
      }
    }
  }
}

} // namespace s57_grids

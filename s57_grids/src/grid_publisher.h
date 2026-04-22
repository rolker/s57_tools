#ifndef S57_GRIDS_S57_GRID_PUBLISHER_H
#define S57_GRIDS_S57_GRID_PUBLISHER_H

#include <future>


#include "grid_map_ros/grid_map_ros.hpp"
#include "marine_charts/s57_catalog.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "s57_msgs/srv/get_datasets.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace s57_grids
{

/// Generates grid_maps from S57 nautical chart data.
/// Services are made available to list or retrieve gridded
/// charts within specified areas.
///
/// Not yet implemented for ROS2:
/// Grids specified in the parameter server are generated
/// centered on the robot's position and updated as the robot
/// moves. These grids combine all available data for that area.

class GridPublisher: public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit GridPublisher(const std::string &node_name);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

private:
  // Service handler that returns a list of charts that intersect a queried area.
  void listDatasets(
    s57_msgs::srv::GetDatasets::Request::ConstSharedPtr request,
    s57_msgs::srv::GetDatasets::Response::SharedPtr response
  );

  // Service handler that returns a list of charts and also queues them to be published.
  void getDatasets(
    s57_msgs::srv::GetDatasets::Request::ConstSharedPtr request,
    s57_msgs::srv::GetDatasets::Response::SharedPtr response
  );

  // Called periodically to check for new grids that may be ready to be published or further processed.
  void checkForNewGrids();

  // Called periodically to republish existing grids with fresh timestamps.
  void republishGrids();

  // Called once after the boat's pose first becomes known via TF, queues
  // all charts within precompute_radius_ around that position so the
  // costmap layer's first get_datasets request finds them already cached
  // (or close to ready).
  void tryPrecompute();

  std::shared_ptr<marine_charts::S57Catalog> catalog_;
  rclcpp::Service<s57_msgs::srv::GetDatasets>::SharedPtr list_service_;
  rclcpp::Service<s57_msgs::srv::GetDatasets>::SharedPtr get_service_;

  // Factor applied to a chart's recommended resolution to determine a grid's output resolution.
  double resolution_factor_ = 1.0;

  std::map<std::string, rclcpp_lifecycle::LifecyclePublisher<grid_map_msgs::msg::GridMap>::SharedPtr > grid_publishers_;

  bool publish_costmaps_ = false;
  std::map<std::string, rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr > costmap_publishers_;


  std::map<std::string, std::shared_ptr<grid_map::GridMap> > dataset_grids_;
  std::mutex dataset_grids_mutex_;

  // Futures waiting for datasets being generated in separate threads
  std::map<std::string, std::future<std::shared_ptr<grid_map::GridMap> > > pending_dataset_grids_;

  // Wall-clock time each chart was first added to requested_grids_ — used
  // to log per-chart processing latency when the grid is published.
  std::map<std::string, rclcpp::Time> grid_request_start_times_;

  rclcpp::TimerBase::SharedPtr new_grids_timer_;
  // Period for new_grids_timer_ in seconds. Smaller values reap completed
  // std::async futures sooner; the previous hardcoded 1.0 s added up to
  // 1 s of latency per chart.
  double check_new_grids_period_ = 0.1;

  rclcpp::TimerBase::SharedPtr republish_grids_timer_;
  double grid_republish_period_ = 0.0; // seconds, if <= 0.0, no republishing

  std::vector<std::string> requested_grids_;
  std::vector<std::string> requested_grids_to_publish_;
  std::mutex requested_grids_mutex;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_ = "map";

  // Precompute-on-first-TF: when robot_base_frame_ is set and
  // precompute_radius_ > 0, tryPrecompute polls TF until the boat's
  // pose is known, then queues all charts within precompute_radius_
  // (meters) of that position.
  std::string robot_base_frame_;
  double precompute_radius_ = 5000.0;
  bool did_precompute_ = false;

  std::atomic<bool> abort_flag_ = false;
};

} // namespace s57_grids

#endif

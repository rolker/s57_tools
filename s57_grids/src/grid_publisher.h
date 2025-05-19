#ifndef S57_GRIDS_S57_GRID_PUBLISHER_H
#define S57_GRIDS_S57_GRID_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "s57_msgs/srv/get_datasets.hpp"
#include "marine_charts/s57_catalog.h"
#include "grid_map_ros/grid_map_ros.hpp"
#include <future>
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

  std::shared_ptr<marine_charts::S57Catalog> catalog_;
  rclcpp::Service<s57_msgs::srv::GetDatasets>::SharedPtr list_service_;
  rclcpp::Service<s57_msgs::srv::GetDatasets>::SharedPtr get_service_;

  // Factor applied to a chart's recommended resolution to determine a grid's output resolution.
  double resolution_factor_ = 1.0;  

  std::map<std::string, rclcpp_lifecycle::LifecyclePublisher<grid_map_msgs::msg::GridMap>::SharedPtr > grid_publishers_;
  
  std::map<std::string, std::shared_ptr<grid_map::GridMap> > dataset_grids_;
  std::mutex dataset_grids_mutex_;
  
  // Futures waiting for datasets being generated in separate threads
  std::map<std::string, std::future<std::shared_ptr<grid_map::GridMap> > > pending_dataset_grids_;


  rclcpp::TimerBase::SharedPtr new_grids_timer_;

  std::vector<std::string> requested_grids_;
  std::vector<std::string> requested_grids_to_publish_;
  std::mutex requested_grids_mutex;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_ = "map";

  // struct GridOutput
  // {
  //   std::string name; // name used for output topic
  //   double resolution; // meters
  //   double length; // length of the grid sides in meters
  //   double period; // time between checks if grid needs updating (seconds)

  //   ros::Publisher publisher;
  //   ros::Publisher costmap_publisher; // Used for debugging in rviz, which struggles to display grid_maps.
  //   ros::Time last_publish_time;
  //   std::thread thread;
  // };

  /// Continuously updates a grid based on the robot's position.
  /// This is meant to be run in its own thread and will only return
  /// when the abort flag is set.
  // void updateGrid(GridOutput& output_grid);

  // std::map<std::string, GridOutput> output_grids_;

  // struct RobotSpecs
  // {
  //   std::string frame_id = "base_link";
  //   double minimum_depth = 1.0;
  //   double maximum_caution_depth = 3.0;
  //   double overhead_clearance = 10.0;

  //   double minimum_speed = 0.0;
  //   double maximum_speed = 1.0;
  // };

  // RobotSpecs robot_;

  std::atomic<bool> abort_flag_ = false;
};

} // namespace s57_grids

#endif

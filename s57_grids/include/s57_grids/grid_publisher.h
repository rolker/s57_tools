#ifndef S57_GRIDS_S57_GRID_PUBLISHER_H
#define S57_GRIDS_S57_GRID_PUBLISHER_H

#include "ros/ros.h"
#include "s57_msgs/GetDatasets.h"
#include "s57_grids/s57_catalog.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <future>
#include <tf2_ros/transform_listener.h>

namespace s57_grids
{

// Generates grid_maps from S57 nautical chart data.
// Grids specified in the parameter server are generated centered on the robot's position
// and updated as the robot moves. These grids combine all avaiable data for that area.
// Services are also made avaiable to list or retrieve gridded charts within specified areas.
class GridPublisher
{
public:
  GridPublisher();
  ~GridPublisher();

private:
  // Service handler that returns a list of charts that intersect a queried area.
  bool listDatasets(s57_msgs::GetDatasets::Request &req, s57_msgs::GetDatasets::Response &res);

  // Service handler that returns a list of charts and also queues them to be published.
  bool getDatasets(s57_msgs::GetDatasets::Request &req, s57_msgs::GetDatasets::Response &res);

  // Called periodically to check for new grids that may be ready to be published or further processed.
  void checkForNewGrids(const ros::TimerEvent& event);

  // Buffers the grid to provide a margin of safety
  std::shared_ptr<grid_map::GridMap> bufferGrid(std::shared_ptr<grid_map::GridMap> grid);

  std::shared_ptr<s57_grids::S57Catalog> catalog_;
  ros::ServiceServer list_service_;
  ros::ServiceServer get_service_;

  // Factor applied to a chart's recommended resolution to determine a grid's output resolution.
  double resolution_factor_ = 1.0;  

  // Distance in meters to speeds around lower speed cells.
  double buffer_radius_ = 50.0;

  std::map<std::string, ros::Publisher> grid_publishers_;
  
  std::map<std::string, std::shared_ptr<grid_map::GridMap> > dataset_grids_;
  std::mutex dataset_grids_mutex_;
  
  // Futures waiting for datasets being generated in separate threads
  std::map<std::string, std::future<std::shared_ptr<grid_map::GridMap> > > pending_dataset_grids_;

  // Futures waiting for datasets being buffered in separate threads.
  std::map<std::string, std::future<std::shared_ptr<grid_map::GridMap> > > pending_buffered_grids_;

  ros::Timer new_grids_timer_;

  std::vector<std::string> requested_grids_;
  std::vector<std::string> requested_grids_to_publish_;
  std::mutex requested_grids_mutex;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_ = "map";

  struct GridOutput
  {
    std::string name; // name used for output topic
    double resolution; // meters
    double length; // length of the grid sides in meters
    double period; // time between checks if grid needs updating (seconds)

    ros::Publisher publisher;
    ros::Time last_publish_time;
    std::thread thread;
  };

  // Continoulsy updates a grid based on the robot's position.
  // This is meant to be run in its own thread and will only return
  // when the abort flag is set.
  void updateGrid(GridOutput& output_grid);

  std::map<std::string, GridOutput> output_grids_;

  struct RobotSpecs
  {
    std::string frame_id = "base_link";
    double minimum_depth = 1.0;
    double maximum_caution_depth = 3.0;
    double overhead_clearance = 10.0;

    double minimum_speed = 0.0;
    double maximum_speed = 1.0;
  };

  RobotSpecs robot_;

  bool abort_flag_=false;
  std::mutex abort_flag_mutex_;


};

} // namespace s57_grids

#endif

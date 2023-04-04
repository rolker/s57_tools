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

class GridPublisher
{
public:
  GridPublisher();
  ~GridPublisher();

private:
  bool listDatasets(s57_msgs::GetDatasets::Request &req, s57_msgs::GetDatasets::Response &res);
  bool getDatasets(s57_msgs::GetDatasets::Request &req, s57_msgs::GetDatasets::Response &res);
  void checkForNewGrids(const ros::TimerEvent& event);

  std::shared_ptr<s57_grids::S57Catalog> catalog_;
  ros::ServiceServer list_service_;
  ros::ServiceServer get_service_;

  double resolution_factor_ = 1.0;  

  std::map<std::string, ros::Publisher> grid_publishers_;
  std::map<std::string, std::shared_ptr<grid_map::GridMap> > dataset_grids_;
  std::map<std::string, std::future<std::shared_ptr<grid_map::GridMap> > > pending_dataset_grids_;

  ros::Timer new_grids_timer_;

  std::vector<std::string> requested_grids_;
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

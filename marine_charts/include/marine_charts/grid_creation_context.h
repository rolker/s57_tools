#ifndef MARINE_CHARTS_GRID_CREATION_CONTEXT_H
#define MARINE_CHARTS_GRID_CREATION_CONTEXT_H

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "grid_map_core/GridMap.hpp"
#include "tf2_ros/buffer.h"


class OGRGeometry;
class OGRCoordinateTransformation;

namespace marine_charts
{

class GridCreationContext
{
public:
  GridCreationContext(std::string map_frame, tf2_ros::Buffer &tf_buffer, double resolution_factor, rclcpp::Logger logger);
  bool ecefToMap(double x, double y, double z, double &mx, double &my);
  bool llToMap(double lat, double lon, double &x, double &y);
  void rasterize(grid_map::GridMap& grid_map, OGRGeometry* geometry, double value, std::string layer, bool lower=false);

  // compares existing value and only updates if greater when lower is false or lower when lower is true
  void updateCost(grid_map::GridMap& grid_map, const grid_map::Index& index,  double value, std::string layer, bool lower=false);

  double resolution_factor() const;
  const geometry_msgs::msg::TransformStamped& earth_to_map() const;
private:
  double resolution_factor_;
  geometry_msgs::msg::TransformStamped earth_to_map_;
  std::shared_ptr<OGRCoordinateTransformation> ll_to_earth_;
  rclcpp::Logger logger_;
};


} // namespace marine_charts

#endif // MARINE_CHARTS_GRID_CREATION_CONTEXT_H

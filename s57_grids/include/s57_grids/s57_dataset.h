#ifndef S57_GRIDS_S57_DATASET_H
#define S57_GRIDS_S57_DATASET_H

#include <string>
#include <memory>
#include <geographic_msgs/BoundingBox.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

class GDALDataset;
class OGRCoordinateTransformation;
class OGRGeometry;

namespace s57_grids
{

struct GridCreationContext
{
  double resolution_factor;
  geometry_msgs::TransformStamped earth_to_map;
  std::shared_ptr<OGRCoordinateTransformation> ll_to_earth;
  
  GridCreationContext(std::string map_frame, tf2_ros::Buffer &tf_buffer, double res_factor);
  bool ecefToMap(double x, double y, double z, double &mx, double &my);
  bool llToMap(double lat, double lon, double &x, double &y);
  void rasterize(grid_map::GridMap& grid_map, OGRGeometry* geometry, double value, std::string layer, bool lower=false);

  // compares existing value and only updates if greater when lower is false or lower when lower is true
  void updateCost(grid_map::GridMap& grid_map, const grid_map::Index& index,  double value, std::string layer, bool lower=false);
};

class S57Dataset
{
public:
  S57Dataset(std::string path);
  ~S57Dataset();

  void setBounds(double minLat, double minLon, double maxLat, double maxLon);
  const geographic_msgs::BoundingBox& getBounds();
  bool hasValidBounds() const;
  bool hasValidBoundsTryOpen();
  bool intersects(double minLat, double minLon, double  maxLat, double maxLon) const;
  std::string const &filePath() const;
  std::string const &label() const;
  std::string topic() const;
  
  std::shared_ptr<grid_map::GridMap> getGrid(GridCreationContext context);

  double chartScale();
  double recommendedResolution();
private:
  std::shared_ptr<GDALDataset> open();
  
private:
  std::string file_path_;
  std::string label_;
  geographic_msgs::BoundingBox bounds_;
  double chart_scale_ = 0.0;

  bool abort_flag_=false;
  std::mutex abort_flag_mutex_;

};

} // namespace s57_grids

#endif

#ifndef MARINE_CHARTS_S57_DATASET_H
#define MARINE_CHARTS_S57_DATASET_H

#include <string>
#include <memory>
#include "geographic_msgs/msg/bounding_box.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "grid_map_core/GridMap.hpp"
#include "marine_charts/grid_creation_context.h"

class GDALDataset;
class OGRCoordinateTransformation;
class OGRGeometry;

namespace marine_charts
{


class S57Dataset
{
public:
  S57Dataset(std::string path);

  void setBounds(double minLat, double minLon, double maxLat, double maxLon);
  const geographic_msgs::msg::BoundingBox& getBounds();
  bool hasValidBounds() const;
  bool hasValidBoundsTryOpen();
  bool intersects(double minLat, double minLon, double  maxLat, double maxLon) const;
  std::string const &filePath() const;
  std::string const &label() const;
  std::string topic() const;

  std::shared_ptr<grid_map::GridMap> getGrid(GridCreationContext context, std::atomic<bool>& abort_flag);

  double chartScale();
  double recommendedResolution();
private:
  std::shared_ptr<GDALDataset> open();

private:
  std::string file_path_;
  std::string label_;
  geographic_msgs::msg::BoundingBox bounds_;
  double chart_scale_ = 0.0;
};

} // namespace marine_charts

#endif

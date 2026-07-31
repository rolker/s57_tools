#ifndef MARINE_CHARTS_S57_DATASET_H
#define MARINE_CHARTS_S57_DATASET_H

#include <string>
#include <memory>
#include <vector>
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

// One M_QUAL (OBJL 308) quality-of-data zone: its coverage polygon as WKB plus
// the S-57 CATZOC code (1=A1, 2=A2, 3=B, 4=C, 5=D, 6=U; 0 when unset). WKB keeps
// OGR types out of this header — callers reconstruct the geometry with
// OGRGeometryFactory::createFromWkb.
struct CatzocZone
{
  std::vector<unsigned char> wkb;
  int catzoc = 0;
};

// Read the M_QUAL (OBJL 308) quality zones from an open S-57 vector dataset.
// This is the CATZOC data that getGrid()'s case 308 deliberately ignores (the
// costmap path does not consume it); s57_to_geotiff uses it for the chart-layer
// σ floor (ADR-0010 D7). Returns an empty vector when the dataset has no M_QUAL
// features.
std::vector<CatzocZone> readCatzocZones(GDALDataset* dataset);

// Convenience overload: open the S-57 cell at `path` and read its M_QUAL zones.
// Returns an empty vector when the file cannot be opened.
std::vector<CatzocZone> readCatzocZones(const std::string& path);

} // namespace marine_charts

#endif

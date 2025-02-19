#ifndef MARINE_CHARTS_S57_CATALOG_H
#define MARINE_CHARTS_S57_CATALOG_H

#include <map>
#include <memory>
#include <string>
#include <vector>
#include "geographic_msgs/msg/bounding_box.hpp"

// TODO EPSG:4978 for lat/lon to ECEF
class OGRCoordinateTransformation;

namespace marine_charts
{

class S57Dataset;

class S57Catalog
{
public:
  S57Catalog(std::string enc_root);
  std::vector<std::shared_ptr<S57Dataset> > intersectingCharts(const geographic_msgs::msg::BoundingBox &bounds);
  std::vector<std::shared_ptr<S57Dataset> > intersectingCharts(double minLat, double minLon, double maxLat, double maxLon);
  bool ecefToLatLong(double x, double y, double z, double &lat, double &lon);
  bool llToECEF(double lat, double lon, double &x, double &y, double &z);
  std::shared_ptr<S57Dataset> dataset(std::string label) const;

private:
  std::map<std::string, std::shared_ptr<S57Dataset> > datasets_;
  std::shared_ptr<OGRCoordinateTransformation> m_WGS84ToECEF_transformation;
  std::shared_ptr<OGRCoordinateTransformation> m_ECEFToWGS84_transformation;
};

} // namespace marine_charts

#endif

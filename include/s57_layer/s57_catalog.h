#ifndef S57_LAYER_S57_CATALOG_H
#define S57_LAYER_S57_CATALOG_H

#include <string>
#include <vector>
#include <memory>

// TODO EPSG:4978 for lat/lon to ECEF
class OGRCoordinateTransformation;

namespace s57_layer
{

class S57Dataset;

class S57Catalog
{
public:
  S57Catalog(std::string enc_root);
  std::vector<std::shared_ptr<S57Dataset> > intersectingCharts(double minLat, double minLon, double maxLat, double maxLon);
  std::pair<double, double> ecefToLatLong(double x, double y, double z);

private:
  std::vector<std::shared_ptr<S57Dataset> > m_datasets;
  std::shared_ptr<OGRCoordinateTransformation> m_WGS84ToECEF_transformation;
  std::shared_ptr<OGRCoordinateTransformation> m_ECEFToWGS84_transformation;
};

} // namespace s57_layer

#endif

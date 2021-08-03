#ifndef S57_LAYER_S57_DATASET_H
#define S57_LAYER_S57_DATASET_H

#include <string>
#include <memory>

class GDALDataset;
class OGREnvelope;

namespace costmap_2d
{
  class Costmap2D;
}

namespace s57_layer
{

class S57Layer;

void GDALDeleter(GDALDataset * ds);

class S57Dataset
{
public:
  S57Dataset(std::string path);
  void setEnvelope(double minLat, double minLon, double maxLat, double maxLon);
  std::shared_ptr<OGREnvelope> getEnvelope();
  std::string const &filePath() const;
  std::string const &label() const;
  std::shared_ptr<costmap_2d::Costmap2D> getCosts(double minLat, double minLon, double maxLat, double maxLon, S57Layer &layer);
private:
  void open();

private:
  std::string m_file_path;
  std::string m_label;
  std::shared_ptr<GDALDataset> m_dataset;
  std::shared_ptr<OGREnvelope> m_envelope;
  double m_minimum_pixel_size = 0.0;
};

} // namespace s57_layer

#endif

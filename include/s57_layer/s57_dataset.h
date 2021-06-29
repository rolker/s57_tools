#ifndef S57_LAYER_S57_DATASET_H
#define S57_LAYER_S57_DATASET_H

#include <string>
#include <memory>

class GDALDataset;
class OGREnvelope;

namespace s57_layer
{

void GDALDeleter(GDALDataset * ds);

class S57Dataset
{
public:
  S57Dataset(std::string path);
  void setEnvelope(double minLat, double minLon, double maxLat, double maxLon);
  std::shared_ptr<OGREnvelope> getEnvelope();
  std::string const &filePath() const;
private:
  void open();

private:
  std::string m_file_path;
  std::shared_ptr<GDALDataset> m_dataset;
  std::shared_ptr<OGREnvelope> m_envelope;
};

} // namespace s57_layer

#endif

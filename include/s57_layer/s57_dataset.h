#ifndef S57_LAYER_S57_DATASET_H
#define S57_LAYER_S57_DATASET_H

#include <string>
#include <memory>

class GDALDataset;

namespace s57_layer
{

void GDALDeleter(GDALDataset * ds);

class S57Dataset
{
public:
  S57Dataset(std::string path);
  bool valid() const;
private:
  std::shared_ptr<GDALDataset> m_dataset;
};

} // namespace s57_layer

#endif

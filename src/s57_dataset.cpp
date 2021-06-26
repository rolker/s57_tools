#include "s57_layer/s57_dataset.h"

#include "ogrsf_frmts.h"
#include <iostream>

namespace s57_layer
{

S57Dataset::S57Dataset(std::string path)
{
  std::cerr << path << std::endl;
  m_dataset = std::shared_ptr<GDALDataset>(reinterpret_cast<GDALDataset*>(GDALOpenEx(path.c_str(), GDAL_OF_VECTOR,  nullptr, nullptr, nullptr)),GDALDeleter);
  if(m_dataset)
  {
    OGRLayer* coverage = m_dataset->GetLayerByName("M_COVR");
    if(coverage)
    {
      OGREnvelope extent;
      if(coverage->GetExtent(&extent) == OGRERR_NONE)
      {
        std::cerr << extent.MinX << ", " << extent.MinY << "  " << extent.MaxX << ", " << extent.MaxY << std::endl;
      }
    }
    else
      m_dataset.reset();
  }
}

void GDALDeleter(GDALDataset* ds)
{
  GDALClose(ds);
}

bool S57Dataset::valid() const
{
  return bool(m_dataset);
}

} // namespace s57_layer

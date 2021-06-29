#include "s57_layer/s57_dataset.h"

#include "ogrsf_frmts.h"
#include <iostream>

namespace s57_layer
{

S57Dataset::S57Dataset(std::string path):m_file_path(path)
{
}

void S57Dataset::setEnvelope(double minLat, double minLon, double maxLat, double maxLon)
{
  m_envelope = std::shared_ptr<OGREnvelope>(new OGREnvelope);
  m_envelope->Merge(minLon, minLat);
  m_envelope->Merge(maxLon, maxLat);
}

std::shared_ptr<OGREnvelope> S57Dataset::getEnvelope()
{
  if(!m_envelope || (!m_envelope->IsInit() && !m_dataset))
    open();
  return m_envelope;
}

void S57Dataset::open()
{
  m_dataset = std::shared_ptr<GDALDataset>(reinterpret_cast<GDALDataset*>(GDALOpenEx(m_file_path.c_str(), GDAL_OF_VECTOR,  nullptr, nullptr, nullptr)),GDALDeleter);
  
  if(m_dataset)
  {
    OGRLayer* coverage = m_dataset->GetLayerByName("M_COVR");
    if(coverage)
    {
      OGREnvelope envelope;
      if(coverage->GetExtent(&envelope) == OGRERR_NONE)
      {
        m_envelope = std::shared_ptr<OGREnvelope>(new OGREnvelope(envelope));
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

std::string const &S57Dataset::filePath() const
{
  return m_file_path;
}

} // namespace s57_layer

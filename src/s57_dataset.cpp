#include "s57_layer/s57_dataset.h"

#include "ogrsf_frmts.h"
#include <iostream>
#include <costmap_2d/costmap_2d.h>
#include "s57_layer/s57_layer.h"

namespace s57_layer
{

S57Dataset::S57Dataset(std::string path):m_file_path(path)
{
  auto slash = path.rfind("/");
  if(slash != path.npos)
    m_label = path.substr(slash+1);
  else
    m_label = path;
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
      // figure out scale and calc potential grid size
      // From S52:
      // minimum effective size of the area for chart display: 270 x 270 mm.
      // minimum lines per mm (L) given by L=864/s, where s is the smaller
      // dimension of the chart display area.
      // 864/270 = 3.2 lines/mm
      // pixel size = 1/3.2 = 0.3125 mm -> 0.0003125 meters
      // symbols should be at least 12 pixels: 3.75 mm or 0.00375 meters
      int minSCAMIN = -1;
      for(auto l: m_dataset->GetLayers())
      {
        l->ResetReading();
        OGRFeature* feature;
        while(feature = l->GetNextFeature())
        {
          int i = feature->GetFieldIndex("SCAMIN");
          if(i>-1) 
          {
            int scamin = feature->GetFieldAsInteger(i);
            if(scamin != 0)
            {
              if(minSCAMIN == -1)
                minSCAMIN = scamin;
              else
                minSCAMIN = std::min(minSCAMIN, scamin);
            }
          }
        }
      }
      m_minimum_pixel_size = 0.0003125*minSCAMIN;
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

std::string const &S57Dataset::label() const
{
  return m_label;
}

std::shared_ptr<costmap_2d::Costmap2D> S57Dataset::getCosts(double minLat, double minLon, double maxLat, double maxLon, S57Layer &layer)
{
  std::shared_ptr<costmap_2d::Costmap2D> ret;
  if(!m_dataset)
    open();
  if(m_dataset && m_minimum_pixel_size > 0.0)
  {
    OGREnvelope envelope;
    envelope.Merge(minLon, minLat);
    envelope.Merge(maxLon, maxLat);
    envelope.Intersect(*m_envelope);

    double minX, minY, maxX, maxY;
    if(layer.llToWorld(envelope.MinY, envelope.MinX, minX, minY) && layer.llToWorld(envelope.MaxY, envelope.MaxX, maxX, maxY))
    {
      http://www.cs.cmu.edu/~quake/triangle.html
      std::cerr << " " << m_label << std::endl;
      std::cerr << "  world: " << minX << ", " << minY << " to " << maxX << ", " << maxY << std::endl;
      ret = std::shared_ptr<costmap_2d::Costmap2D>(new costmap_2d::Costmap2D(std::ceil((maxX-minX)/m_minimum_pixel_size), std::ceil((maxY-minY)/m_minimum_pixel_size), m_minimum_pixel_size, minX, minY, costmap_2d::NO_INFORMATION));
      std::cerr << "  cell size: " << m_minimum_pixel_size << std::endl;
      std::cerr << "  map size: " << ret->getSizeInCellsX() << ", " << ret->getSizeInCellsY() << std::endl;
      std::map<std::pair<int, int>, std::shared_ptr<OGRPoint> > latlonMap;
      for(int i = 0; i < ret->getSizeInCellsX(); i++)
        for(int j = 0; j < ret->getSizeInCellsY(); j++)
        {
          double wx, wy;
          ret->mapToWorld(i, j, wx, wy);
          double cellLat, cellLon;
          if (layer.worldToLatLon(wx, wy, cellLat, cellLon))
            latlonMap[std::make_pair(i,j)] = std::shared_ptr<OGRPoint>(new OGRPoint(cellLon, cellLat));
        }
      
      OGRLayer* land_area = m_dataset->GetLayerByName("LNDARE");
      if(land_area)
      {
        land_area->SetSpatialFilterRect(minLon, minLat, maxLon, maxLat);
        std::cerr << "  land area with " << land_area->GetFeatureCount() << " features" << std::endl;
        land_area->ResetReading();
        OGRFeature* feature;
        while(feature = land_area->GetNextFeature())
        {
          auto geometry = feature->GetGeometryRef();
          std::cerr << "   geom type: " << geometry->getGeometryType() << std::endl;
          if(geometry->getGeometryType() == wkbPolygon)
          {
            OGRPolygon* polygon = geometry->toPolygon();
            OGRPoint p;
            polygon->getExteriorRing()->getPoint(0, &p);
            std::cerr << "    first point of exterior ring: " << p.getX() << ", " << p.getY() << std::endl;
            std::cerr << "    first cell of ret costmap: " << latlonMap.begin()->second->getX() << ", " << latlonMap.begin()->second->getY() << std::endl;
          }
          for(auto cell: latlonMap)
            if(geometry->Contains( cell.second.get()))
              ret->setCost(cell.first.first, cell.first.second, costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
  return ret;
}

} // namespace s57_layer

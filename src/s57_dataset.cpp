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

double S57Dataset::minimumPixelSize()
{
  if(!m_dataset)
    open();
  return m_minimum_pixel_size;
}

double S57Dataset::distanceTo(double x, double y, S57Layer &layer) const
{
  double minX, minY, maxX, maxY;
  if(m_envelope && layer.llToWorld(m_envelope->MinY, m_envelope->MinX, minX, minY) && layer.llToWorld(m_envelope->MaxY, m_envelope->MaxX, maxX, maxY))
  {
    double dx, dy;
    if(x >= minX && x <= maxX)
      dx = 0.0;
    else
      if (x < minX)
        dx = minX - x;
      else
        dx = x - maxX;
    if(y >= minY && y <= maxY)
      dy = 0.0;
    else
      if (y < minY)
        dy = minY - y;
      else
        dy = y - maxY;
    if(dx == 0.0 && dy == 0.0)
      return 0.0;
    return sqrt(dx*dx+dy*dy);
  }
  return std::nan("");
}

std::shared_ptr<costmap_2d::Costmap2D> S57Dataset::getCosts(double minLat, double minLon, double maxLat, double maxLon, S57Layer &layer)
{
  std::shared_ptr<costmap_2d::Costmap2D> ret;
  if(!m_dataset)
    open();
  if(m_dataset && m_minimum_pixel_size > 0.0)
  {
    // OGREnvelope envelope;
    // envelope.Merge(minLon, minLat);
    // envelope.Merge(maxLon, maxLat);
    // envelope.Intersect(*m_envelope);
    OGREnvelope envelope(*m_envelope);

    double minX, minY, maxX, maxY;
    if(layer.llToWorld(envelope.MinY, envelope.MinX, minX, minY) && layer.llToWorld(envelope.MaxY, envelope.MaxX, maxX, maxY))
    {
      // http://www.cs.cmu.edu/~quake/triangle.html
      std::cerr << " " << m_label << std::endl;
      std::cerr << "  world: " << minX << ", " << minY << " to " << maxX << ", " << maxY << std::endl;
      ret = std::shared_ptr<costmap_2d::Costmap2D>(new costmap_2d::Costmap2D(std::ceil((maxX-minX)/m_minimum_pixel_size), std::ceil((maxY-minY)/m_minimum_pixel_size), m_minimum_pixel_size, minX, minY,  costmap_2d::NO_INFORMATION));
      std::cerr << "  cell size: " << m_minimum_pixel_size << std::endl;
      std::cerr << "  map size: " << ret->getSizeInCellsX() << ", " << ret->getSizeInCellsY() << std::endl;

      // std::map<std::pair<int, int>, std::shared_ptr<OGRPoint> > latlonMap;
      // for(int i = 0; i < ret->getSizeInCellsX(); i++)
      //   for(int j = 0; j < ret->getSizeInCellsY(); j++)
      //   {
      //     double wx, wy;
      //     ret->mapToWorld(i, j, wx, wy);
      //     double cellLat, cellLon;
      //     if (layer.worldToLatLon(wx, wy, cellLat, cellLon))
      //       latlonMap[std::make_pair(i,j)] = std::shared_ptr<OGRPoint>(new OGRPoint(cellLon, cellLat));
      //   }
      
      OGRLayer* sea_area = m_dataset->GetLayerByName("SEAARE");
      if(sea_area)
      {
        //sea_area->SetSpatialFilterRect(minLon, minLat, maxLon, maxLat);
        std::cerr << "  sea area with " << sea_area->GetFeatureCount() << " features" << std::endl;
        sea_area->ResetReading();
        OGRFeature* feature;
        while(feature = sea_area->GetNextFeature())
        {
          auto geometry = feature->GetGeometryRef();
          rasterize(*ret, *geometry, layer, costmap_2d::FREE_SPACE);
        }
      }


      OGRLayer* land_area = m_dataset->GetLayerByName("LNDARE");
      if(land_area)
      {
        //land_area->SetSpatialFilterRect(minLon, minLat, maxLon, maxLat);
        std::cerr << "  land area with " << land_area->GetFeatureCount() << " features" << std::endl;
        land_area->ResetReading();
        OGRFeature* feature;
        while(feature = land_area->GetNextFeature())
        {
          auto geometry = feature->GetGeometryRef();
          rasterize(*ret, *geometry, layer, costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
  return ret;
}

void S57Dataset::rasterize(costmap_2d::Costmap2D& map, OGRGeometry& geometry, S57Layer &layer, unsigned char value)
{
  // http://alienryderflex.com/polygon_fill/
  if(geometry.getGeometryType() == wkbPolygon)
  {
    OGRPolygon* polygon = geometry.toPolygon();
    struct WorldPoint
    {
      double x,y;
    };

    std::vector<std::vector<WorldPoint> > rings;
    rings.push_back(std::vector<WorldPoint>());

    for(auto p: polygon->getExteriorRing())
    {
      WorldPoint wp;
      if(layer.llToWorld(p.getY(), p.getX(), wp.x, wp.y))
        rings.back().push_back(wp);
    }
    if(!polygon->getExteriorRing()->get_IsClosed())
      rings.back().push_back(rings.back().front()); // close the ring if necessary
    for(int i = 0; i < polygon->getNumInteriorRings(); i++)
    {
      rings.push_back(std::vector<WorldPoint>());
      for(auto p: polygon->getInteriorRing(i))
      {
        WorldPoint wp;
        if(layer.llToWorld(p.getX(), p.getY(), wp.x, wp.y))
          rings.back().push_back(wp);
      }
      if(!polygon->getInteriorRing(i)->get_IsClosed())
        rings.back().push_back(rings.back().front()); // close the ring if necessary

    }

    // std::cerr << "      polygon with " << rings.size() << " rings" << std::endl;
    // for(auto ring: rings)
    // {
    //   std::cerr << "        ring with " << ring.size() << " verticies" << std::endl;
    //   for(auto p: ring)
    //     std::cerr << "          " << p.x << ", " << p.y << std::endl;
    // }

    for(int row = 0; row < map.getSizeInCellsY(); row++)
    {
      double wx, wy;
      map.mapToWorld(0, row, wx, wy);


      std::set<double> nodes;

      for(auto ring: rings)
      {
        for(int i = 0; i < ring.size()-1; i++)
        {
          if(   ring[i].y < wy && ring[i+1].y >= wy 
             || ring[i+1].y < wy && ring[i].y >= wy)
          {
            // (int) (polyX[i]+(pixelY-polyY[i])/(polyY[j]-polyY[i]) *(polyX[j]-polyX[i]))
            nodes.insert(ring[i].x+(wy-ring[i].y)/(ring[i+1].y-ring[i].y)*(ring[i+1].x-ring[i].x));
          }
        }
      }

      // if(!nodes.empty())
      // {
      //   std::cerr << "        wy: " << wy << std::endl;
      //   std::cerr << "        " << nodes.size() << " nodes" << std::endl;
      // }


      auto node = nodes.begin();
      while(node != nodes.end())
      {
        auto next_node = node;
        next_node++;
        if(next_node == nodes.end())
          break;

        if(*node > map.getOriginX() + map.getSizeInMetersX())
          break;

        if(*next_node > map.getOriginX())
        {
          unsigned int y, x1, x2;
          if(*node < map.getOriginX())
            x1 = 0;
          else
            map.worldToMap(*node, wy, x1, y);
          if(!map.worldToMap(*next_node, wy, x2, y))
            x2 = map.getSizeInCellsX()-1;

          // std::cerr << "          " << *node << " to " << *next_node << " map: " << x1 << " to " << x2 << std::endl;
          for(unsigned x = x1; x <= x2; x++)
            map.setCost(x, row, value);
        }
        node = next_node;
        node++;
      }
    }
  }

}

} // namespace s57_layer

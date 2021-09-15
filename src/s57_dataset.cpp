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

      ret = std::shared_ptr<costmap_2d::Costmap2D>(new costmap_2d::Costmap2D(std::ceil((maxX-minX)/m_minimum_pixel_size), std::ceil((maxY-minY)/m_minimum_pixel_size), m_minimum_pixel_size, minX, minY,  costmap_2d::NO_INFORMATION));

      ROS_DEBUG_STREAM(m_label);
      ROS_DEBUG_STREAM("world: " << minX << ", " << minY << " to " << maxX << ", " << maxY);
      ROS_DEBUG_STREAM("cell size: " << m_minimum_pixel_size);
      ROS_DEBUG_STREAM("map size: " << ret->getSizeInCellsX() << ", " << ret->getSizeInCellsY());

      for(auto&& featurePair: m_dataset->GetFeatures())
      {
        int i = featurePair.feature->GetFieldIndex("OBJL");
        if(i == -1) // no field index found
          continue;
        int objl = featurePair.feature->GetFieldAsInteger(i);
        
        switch(objl)
        {
          // Group 1 (skin of the earth)
          // with min depth (DRVAL1)
          case 42:  // DEPARE Depth area
          case 46:  // DRGARE Dredged area
          case 57:  // FLODOC Floating dock
          {
            int i = featurePair.feature->GetFieldIndex("DRVAL1");
            if(i>0)
            {
              double min_depth = featurePair.feature->GetFieldAsDouble(i);
              double min_caution_depth = layer.minimumDepth();
              if(min_depth <= min_caution_depth)
                rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, layer.maximumCautionCost());
              else
              {
                double max_caution_depth = layer.maximumCautionDepth();
                if(min_depth >= max_caution_depth)
                  rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, costmap_2d::FREE_SPACE);
                else
                {
                  unsigned char cost = layer.maximumCautionCost()*(1.0 - ((min_depth-min_caution_depth)/(max_caution_depth-min_caution_depth)));
                  rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, cost);
                }
              }
            }
            break;
          }

          // unknown depth
          case 154: // UNSARE
            rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, layer.unsurveyedCost());
            break;

          // no min depth values, so lethal
          case 65:  // HULKES Hulk
          case 71:  // LNDARE Land area
          case 95:  // PONTON Pontoon
            rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, costmap_2d::LETHAL_OBSTACLE);
            break;

          // Group 2 (Everything else not in Group 1)
          // Lethal
          case 26:  // CAUSWY Causeway
          case 30:  // COALNE Coastline
          case 38:  // DAMCON Dam
          case 49:  // DYKCON Dyke
          case 55:  // FSHFAC Fishing facility
          case 61:  // GATCON Gate
          case 86:  // OBSTRN Obstruction   * TODO, check if submerged and safe
          case 90:  // PILPNT Pile
          case 98:  // PYLONS Pylon/bridge support
          case 122: // SLCONS Shoreline construction
            rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, costmap_2d::LETHAL_OBSTACLE);
            break;

          // Overhead obstructions
          case 11:  // BRIDGE Bridge
          case 21:  // CBLOHD Cable, overhead
          case 34:  // CONVYR Conveyor
          {
            double clearance = -1;
            int i = featurePair.feature->GetFieldIndex("VERCLR");
            if(i>0)
              if(featurePair.feature->IsFieldSetAndNotNull(i))
                clearance = featurePair.feature->GetFieldAsDouble(i);
            i = featurePair.feature->GetFieldIndex("VERCSA");
            if(i>0)
              if(featurePair.feature->IsFieldSetAndNotNull(i))
                clearance = featurePair.feature->GetFieldAsDouble(i);
            i = featurePair.feature->GetFieldIndex("VERCLL");
            if(i>0)
              if(featurePair.feature->IsFieldSetAndNotNull(i))
                clearance = featurePair.feature->GetFieldAsDouble(i);
            if(clearance <= layer.overheadClearance())
              rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, costmap_2d::LETHAL_OBSTACLE);
            break;
          }

          // Caution
          case 27:  // CTNARE Caution area
          case 82:  // MARCUL Marine farm/culture
          case 83:  // MIPARE Military practice area
          case 96:  // PRCARE Precautionary area
          case 158: // WEDKLP Weed/Kelp
            // TODO make caution cost  a parameter
            rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, 50);
            break;

          // Restricted area
          case 112: // RESARE Restricted area
          {
            int i = featurePair.feature->GetFieldIndex("RESTRN");
            if(i>0)
              if(featurePair.feature->IsFieldSetAndNotNull(i))
              {
                int restriction_type = featurePair.feature->GetFieldAsInteger(i);
                if(restriction_type == 7 || restriction_type == 8)
                  rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, costmap_2d::LETHAL_OBSTACLE);
                if(restriction_type == 14) // area to be avoided
                  rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, layer.maximumCautionCost());
              }

            break;
          }

          // features with a depth value
          case 94:  // PIPSOL Pipeline, submarine/on land
          {
            int i = featurePair.feature->GetFieldIndex("DRVAL1");
            if(i>0)
            {
              double min_depth = featurePair.feature->GetFieldAsDouble(i);
              double min_caution_depth = layer.minimumDepth();
              if(min_depth <= min_caution_depth)
                rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, layer.maximumCautionCost());
              else
              {
                double max_caution_depth = layer.maximumCautionDepth();
                if(min_depth >= max_caution_depth)
                  rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, costmap_2d::FREE_SPACE);
                else
                {
                  unsigned char cost = layer.maximumCautionCost()*(1.0 - ((min_depth-min_caution_depth)/(max_caution_depth-min_caution_depth)));
                  rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, cost);
                }
              }
            }
            break;
          }


          // Underwater obstructions
          case 153: // UWTROC Underwater/awash rock
          case 159: // WRECKS Wreck
          {
            int i = featurePair.feature->GetFieldIndex("VALSOU");
            if(i>0)
              if(featurePair.feature->IsFieldSetAndNotNull(i))
              {
                double sounding = featurePair.feature->GetFieldAsDouble(i);
                double min_caution_depth = layer.minimumDepth();
                if(sounding <= min_caution_depth)
                  rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, layer.maximumCautionCost());
                else
                {
                  double max_caution_depth = layer.maximumCautionDepth();
                  if(sounding >= max_caution_depth)
                    rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, costmap_2d::FREE_SPACE);
                  else
                  {
                    unsigned char cost = layer.maximumCautionCost()*(1.0 - ((sounding-min_caution_depth)/(max_caution_depth-min_caution_depth)));
                    rasterize(*ret, *(featurePair.feature->GetGeometryRef()), layer, cost);
                  }
                }
              }
            break;
          }

          // Ignored layers
          case 1:   // ADMARE
          case 2:   // AIRARE
          case 3:   // ACHBRT Anchor berth      *should this incure a cost?
          case 4:   // ACHARE Anchorage area    *should this incure a cost?
          case 5:   // BCNCAR Beacon, cardinal
          case 6:   // BCNISD Beacon, isolated danger  *is the danger encoded otherwise?
          case 7:   // BCNLAT Beacon, lateral
          case 8:   // BCNSAW Beacon, safe water
          case 9:   // BCNSPP Beacon, special purpose/general
          case 10:  // BERTHS Berth
          case 12:  // BUISGL Building, single
          case 13:  // BUAARE Built-up area
          case 14:  // BOYCAR Buoy, cardinal
          case 15:  // BOYINB Buoy, installation
          case 16:  // BOYISD Buoy, isolated danger   *is the danger encoded otherwise?
          case 17:  // BOYLAT Buoy, lateral
          case 18:  // BOYSAW Buoy, safe water
          case 19:  // BOYSPP Buoy, special purpose/general
          case 20:  // CBLARE Cable area
          case 22:  // CBLSUB Cable, submarine
          case 23:  // CANALS Canal                    *is this already represented?
          case 25:  // CTSARE Cargo transhipment area
          case 28:  // CHKPNT Checkpoint
          case 29:  // CGUSTA Coastguard station
          case 31:  // CONZNE Contiguous zone
          case 32:  // COSARE Continental shelf area
          case 33:  // CTRPNY Control point
          case 35:  // CRANES Crane                    *should we worry about this one?
          case 36:  // CURENT Current - non-gravitational
          case 37:  // CUSZNE Custom zone
          case 39:  // DAYMAR Daymark
          case 40:  // DWRRCL Deep water route centerline
          case 41:  // DWRTPT Deep water route part
          case 43:  // DEPCNT Depth contour
          case 44:  // DISMAR Distance mark
          case 45:  // DOCARE Dock area
          case 47:  // DRYDOC Dry dock              *should we explicitly avoid these?
          case 48:  // DMPGRD Dumping ground
          case 50:  // EXEZNE Exclusive economic zone
          case 51:  // FAIRWY Fairway
          case 58:  // FOGSIG Fog signal
          case 59:  // FORSTC Fortified structure
          case 64:  // HRBFAC Harbour facility
          case 69:  // LAKARE Lake
          case 72:  // LNDELV Land elevation
          case 73:  // LNDRGN Land region
          case 74:  // LNDMRK Landmark
          case 75:  // LIGHTS Light
          case 81:  // MAGVAR Magnetic Variation
          case 84:  // MORFAC Mooring/Warping facility
          case 85:  // NAVLNE Navigation Line
          case 91:  // PILBOP Pilot boarding place
          case 92:  // PIPARE Pipeline area
          case 103: // RTPBCM Radar transponder beacon
          case 105: // RDOSTA Radio station
          case 109: // RECTRC Recommended track     *should we use this to improve costs?
          case 114: // RIVERS River
          case 117: // RUNWAY Runway
          case 119: // SEAARE Sea area/named water area
          case 121: // SBDARE Seabed area
          case 125: // SILTNK Silo/tank
          case 126: // SLOTOP Slope topline
          case 127: // SLOGRD Sloping ground
          case 128: // SMCFAC Small craft facility
          case 129: // SOUNDG Sounding              *should we look at individual soundings?
          case 144: // TOPMAR Topmark
          case 146: // TSSBND Traffic separation scheme boundary
          case 148: // TSSLPT Traffic separation scheme lane part
          case 150: // TSEZNE Traffice separation zone
          case 152: // TWRTPT Two-way route part
          case 156: // WATTUR Water trubulence
          case 301: // M_CSCL Compilation scale of data
          case 302: // M_COVR Coverage
          case 305: // M_NPUB Nautical publication information
          case 306: // M_NSYS Navigational system of marks
          case 308: // M_QUAL Quality of data
          case 400: // C_AGGR Aggregation
          case 401: // C_ASSO Association
            break; 

          default:
            ROS_INFO_STREAM("Not handled: objl: " << objl << " name " << featurePair.layer->GetName());
        }
      }

    }
  }
  return ret;
}

void S57Dataset::updateCost(costmap_2d::Costmap2D& map, unsigned int mx, unsigned int my,  unsigned char value)
{
  unsigned char existing_cost = map.getCost(mx, my);
  if(existing_cost == costmap_2d::NO_INFORMATION
     || (value > existing_cost && value != costmap_2d::NO_INFORMATION))
    map.setCost(mx, my, value);
}

void S57Dataset::rasterize(costmap_2d::Costmap2D& map, OGRGeometry& geometry, S57Layer &layer, unsigned char value)
{
  struct WorldPoint
  {
    double x,y;
  };

  switch(geometry.getGeometryType())
  {
    case wkbPoint:
    {
      OGRPoint* point = geometry.toPoint();
      WorldPoint wp;
      unsigned int mx, my;
      if(layer.llToWorld(point->getY(), point->getX(), wp.x, wp.y))
        if(map.worldToMap(wp.x, wp.y, mx, my))
          updateCost(map, mx, my, value);
      break;
    }
    case wkbLineString:
    {
      OGRLineString* lineString = geometry.toLineString();
      std::vector<WorldPoint> lines;
      for(auto p: *lineString)
      {
        WorldPoint wp;
        if(layer.llToWorld(p.getY(), p.getX(), wp.x, wp.y))
          lines.push_back(wp);
      }
      if(lines.size() >1)
      {
        auto p1 = lines.begin();
        auto p2 = p1;
        p2++;
        while(p2 != lines.end())
        {
          double dx = p2->x - p1->x;
          double dy = p2->y - p1->y;
          if(dx != 0.0 || dy != 0.0)
          {
            double stepx, stepy;
            int stepCount;
            if (fabs(dx) > fabs(dy))
            {
              stepx = map.getResolution();
              stepy = stepx * dy/dx;
              stepCount = abs(dx/stepx);
            }
            else
            {
              stepy = map.getResolution();
              stepx = stepy * dx/dy;
              stepCount = abs(dy/stepy);
            }
            for(int i = 0; i <= stepCount; i++)
            {
              unsigned int mx, my;
              if(map.worldToMap(p1->x+i*stepx, p1->y+i*stepy, mx, my))
                updateCost(map, mx, my, value);
            }
          }
          p1 = p2;
          p2++;
        }
      }
      break;
    }
    case wkbPolygon:
    // http://alienryderflex.com/polygon_fill/
    {
      OGRPolygon* polygon = geometry.toPolygon();

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
              updateCost(map, x, row, value);
          }
          node = next_node;
          node++;
        }
      }
      break;
    }
    default:
      ROS_WARN_STREAM("geometry type not handled: " << geometry.getGeometryType());
  }

}

} // namespace s57_layer

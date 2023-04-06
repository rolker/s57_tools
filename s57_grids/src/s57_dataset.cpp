#include "s57_grids/s57_dataset.h"

#include "ogrsf_frmts.h"
#include <iostream>
#include <costmap_2d/costmap_2d.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace s57_grids
{

void GDALDeleter(GDALDataset* ds)
{
  GDALClose(ds);
}

S57Dataset::S57Dataset(std::string path):file_path_(path)
{
  auto slash = path.rfind("/");
  if(slash != path.npos)
    label_ = path.substr(slash+1);
  else
    label_ = path;
}

S57Dataset::~S57Dataset()
{
  std::lock_guard<std::mutex> lock(abort_flag_mutex_);
  abort_flag_ = true; 
}

void S57Dataset::setBounds(double minLat, double minLon, double maxLat, double maxLon)
{
  bounds_.max_pt.latitude = maxLat;
  bounds_.max_pt.longitude = maxLon;
  bounds_.min_pt.latitude = minLat;
  bounds_.min_pt.longitude = minLon;
}

bool S57Dataset::intersects(double minLat, double minLon, double  maxLat, double maxLon) const
{
  if(hasValidBounds())
   return minLat <= bounds_.max_pt.latitude &&
          maxLat >= bounds_.min_pt.latitude &&
          minLon <= bounds_.max_pt.longitude &&
          maxLon >= bounds_.min_pt.longitude;
  return false;
}

const geographic_msgs::BoundingBox& S57Dataset::getBounds()
{
  if(!hasValidBounds())  
    open();
  return bounds_;
}

bool S57Dataset::hasValidBounds() const
{
  return bounds_.max_pt.latitude > bounds_.min_pt.latitude && bounds_.max_pt.longitude > bounds_.min_pt.longitude;
}

bool S57Dataset::hasValidBoundsTryOpen()
{
  if(!hasValidBounds())
    open();
  return hasValidBounds();
}

std::shared_ptr<GDALDataset> S57Dataset::open()
{
  auto dataset = std::shared_ptr<GDALDataset>(reinterpret_cast<GDALDataset*>(GDALOpenEx(file_path_.c_str(), GDAL_OF_VECTOR,  nullptr, nullptr, nullptr)),GDALDeleter);
  
  if(dataset)
  {
    OGRLayer* coverage = dataset->GetLayerByName("M_COVR");
    if(coverage)
    {
      OGREnvelope envelope;
      if(coverage->GetExtent(&envelope) == OGRERR_NONE)
      {
        bounds_.max_pt.latitude = envelope.MaxY;
        bounds_.max_pt.longitude = envelope.MaxX;
        bounds_.min_pt.latitude = envelope.MinY;
        bounds_.min_pt.longitude = envelope.MinX;
      }
      // figure out scale and calc potential grid size
      // From S52:
      // minimum effective size of the area for chart display: 270 x 270 mm.
      // minimum lines per mm (L) given by L=864/s, where s is the smaller
      // dimension of the chart display area.
      // 864/270 = 3.2 lines/mm
      // pixel size = 1/3.2 = 0.3125 mm -> 0.0003125 meters
      // symbols should be at least 12 pixels: 3.75 mm or 0.00375 meters
      
      OGRLayer* dsid = dataset->GetLayerByName("DSID");
      if(dsid)
      {
        dsid->ResetReading();
        OGRFeature* feature = dsid->GetNextFeature();
        if(feature)
        {
          int i = feature->GetFieldIndex("DSPM_CSCL");
          if(feature->IsFieldSetAndNotNull(i))
          {
            chart_scale_ = feature->GetFieldAsDouble(i);
            //m_minimum_pixel_size = 0.0003125*chart_scale;
          }
          OGRFeature::DestroyFeature(feature);
        }

      }

    }
    else
      dataset.reset();
  }
  return dataset;
}


std::string const &S57Dataset::filePath() const
{
  return file_path_;
}

std::string const &S57Dataset::label() const
{
  return label_;
}

std::string S57Dataset::topic() const
{
  std::string ret = label_;
  while(ret.find_first_of(".") != std::string::npos)
    ret.replace(ret.find_first_of("."),1,"_");
  return ret;
}

double S57Dataset::chartScale()
{
  if(chart_scale_ == 0.0)
    open();
  return chart_scale_;
}

double S57Dataset::recommendedResolution()
{
  // From S52:
  // minimum effective size of the area for chart display: 270 x 270 mm.
  // minimum lines per mm (L) given by L=864/s, where s is the smaller
  // dimension of the chart display area.
  // 864/270 = 3.2 lines/mm
  // pixel size = 1/3.2 = 0.3125 mm -> 0.0003125 meters
  return chartScale()*0.0003125;
}

std::shared_ptr<grid_map::GridMap> S57Dataset::getGrid(GridCreationContext context)
{
  ros::Time start_time = ros::Time::now();
  ROS_INFO_STREAM("Generating grid for " << topic());
  std::shared_ptr<grid_map::GridMap> ret;
  auto dataset = open();
  if(dataset)
  {
    double minX, minY, maxX, maxY;

    if(context.llToMap(bounds_.max_pt.latitude, bounds_.max_pt.longitude, maxX, maxY) &&
       context.llToMap(bounds_.min_pt.latitude, bounds_.min_pt.longitude, minX, minY))
    {
      ret = std::make_shared<grid_map::GridMap>();
      ret->setGeometry(grid_map::Length(maxX-minX, maxY-minY), recommendedResolution()*context.resolution_factor);
      auto s = ret->getSize();
      ROS_INFO_STREAM(topic() << " grid size: " << s[0] << " x " << s[1]);
      ret->setPosition(grid_map::Position(minX+(maxX-minX)/2.0,minY+(maxY-minY)/2.0));
      ret->setFrameId(context.earth_to_map.header.frame_id);
      ret->add("elevation");
      ret->add("overhead");
      ret->add("unsurveyed");
      ret->add("caution");
      ret->add("restricted");
      ret->setTimestamp(context.earth_to_map.header.stamp.toNSec());

      for(auto&& featurePair: dataset->GetFeatures())
      {
        {
          // Check if we need to quit
          std::lock_guard<std::mutex> abort_lock(abort_flag_mutex_);
          if(abort_flag_)
            return std::shared_ptr<grid_map::GridMap>();
        }

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
              context.rasterize(*ret, featurePair.feature->GetGeometryRef(), -min_depth, "elevation");
            }
            break;
          }

          // unknown depth
          case 154: // UNSARE
        
            context.rasterize(*ret, featurePair.feature->GetGeometryRef(), 0.0, "unsurveyed");
            break;

          // no min depth values, so lethal
          case 65:  // HULKES Hulk
          case 71:  // LNDARE Land area
          case 95:  // PONTON Pontoon
            context.rasterize(*ret, featurePair.feature->GetGeometryRef(), 1.0, "elevation");
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
            context.rasterize(*ret, featurePair.feature->GetGeometryRef(), 1.0, "elevation");
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
            
            context.rasterize(*ret, featurePair.feature->GetGeometryRef(), clearance, "overhead", true);
            break;
          }

          // Caution
          case 27:  // CTNARE Caution area
          case 82:  // MARCUL Marine farm/culture
          case 83:  // MIPARE Military practice area
          case 96:  // PRCARE Precautionary area
          case 158: // WEDKLP Weed/Kelp
            context.rasterize(*ret, featurePair.feature->GetGeometryRef(), 0, "caution");
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
                  context.rasterize(*ret, featurePair.feature->GetGeometryRef(), 0.0, "restricted");
                if(restriction_type == 14) // area to be avoided
                  context.rasterize(*ret, featurePair.feature->GetGeometryRef(), 0.0, "restricted");
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
              context.rasterize(*ret, featurePair.feature->GetGeometryRef(), -min_depth, "elevation");
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
                context.rasterize(*ret, featurePair.feature->GetGeometryRef(), -sounding, "elevation");
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
          case 301: // M_CSCL Compilation scale of data    *if the scale differs from base chart scale, we we generate different grids
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
  auto elapsed = ros::Time::now() - start_time;
  ROS_INFO_STREAM(topic() << " done. Elapsed time: " << elapsed);
  return ret;
}

GridCreationContext::GridCreationContext(std::string map_frame, tf2_ros::Buffer &tf_buffer, double res_factor): resolution_factor(res_factor)
{
  earth_to_map = tf_buffer.lookupTransform(map_frame, "earth", ros::Time());
}

bool GridCreationContext::ecefToMap(double x, double y, double z, double &mx, double &my)
{
  geometry_msgs::Point ecef;
  ecef.x = x;
  ecef.y = y;
  ecef.z = z;
  geometry_msgs::Point map;
  tf2::doTransform(ecef, map, earth_to_map);
  mx = map.x;
  my = map.y;
  return true;
}

bool GridCreationContext::llToMap(double lat, double lon, double &x, double &y)
{
  // delay creation to make sure it gets created in thread where it
  // will be used
  if(!ll_to_earth)
  {
    OGRSpatialReference wgs84, ecef;
    wgs84.SetWellKnownGeogCS("WGS84");
    ecef.importFromEPSG(4978);
    ll_to_earth = std::shared_ptr<OGRCoordinateTransformation>(OGRCreateCoordinateTransformation(&wgs84, &ecef), OCTDestroyCoordinateTransformation);
  }
  double alt = 0.0;
  if(ll_to_earth->Transform(1, &lat, &lon, &alt))
  {
    return ecefToMap(lat, lon, alt, x, y);
  }
  return false;
}

struct MapPoint
{
  double x,y;
};

void GridCreationContext::rasterize(grid_map::GridMap& grid_map, OGRGeometry* geometry, double value, std::string layer, bool lower)
{
  if(!geometry)
  {
    ROS_WARN_STREAM("null geometry in rasterize");
    return;
  }

  switch(geometry->getGeometryType())
  {
    case wkbPoint:
    {
      OGRPoint* point = geometry->toPoint();
      MapPoint mp;
      if(llToMap(point->getY(), point->getX(), mp.x, mp.y))
      {
        grid_map::Index i;
        if(grid_map.getIndex(grid_map::Position(mp.x, mp.y), i))
          updateCost(grid_map, i, value, layer, lower);
      }
      break;
    }
    case wkbLineString:
    {
      OGRLineString* lineString = geometry->toLineString();
      std::vector<MapPoint> lines;
      for(auto p: *lineString)
      {
        MapPoint mp;
        if(llToMap(p.getY(), p.getX(), mp.x, mp.y))
        {
          lines.push_back(mp);
        }
      }
      if(lines.size() >1)
      {
        auto p1 = lines.begin();
        auto p2 = p1;
        p2++;
        while(p2 != lines.end())
        {
          double half_res = grid_map.getResolution()/2.0;
          for(int xtweak = -1; xtweak <= 1; ++xtweak)
            for(int ytweak = -1; ytweak <= 1; ++ytweak)
            {
              grid_map::Index start, end;
              if(grid_map.getIndex(grid_map::Position(p1->x+xtweak*half_res, p1->y+ytweak*half_res), start) &&
                grid_map.getIndex(grid_map::Position(p2->x+xtweak*half_res, p2->y+ytweak*half_res), end))
                for (grid_map::LineIterator iterator(grid_map, start, end); !iterator.isPastEnd(); ++iterator)
                  updateCost(grid_map, *iterator, value, layer, lower);
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
      OGRPolygon* polygon = geometry->toPolygon();

      std::vector<std::vector<MapPoint> > rings;
      rings.push_back(std::vector<MapPoint>());

      auto er = polygon->getExteriorRing();
      if(!er)
        ROS_WARN_STREAM("null exterior ring");
      else
      {
        //grid_map.add("temp");
        //grid_map::Polygon gpolygon;
        for(auto p: er)
        {
          MapPoint wp;
          if(llToMap(p.getY(), p.getX(), wp.x, wp.y))
            rings.back().push_back(wp);
            //gpolygon.addVertex(grid_map::Position(wp.x, wp.y));
        }
        if(!er->get_IsClosed())
          rings.back().push_back(rings.back().front()); // close the ring if necessary
          //gpolygon.addVertex(gpolygon.getVertex(0));

        // for(grid_map::PolygonIterator iterator(grid_map, gpolygon); !iterator.isPastEnd(); ++iterator)
        //   grid_map.at("temp", *iterator) = 1.0;


        for(int i = 0; i < polygon->getNumInteriorRings(); i++)
        {
          rings.push_back(std::vector<MapPoint>());
          //gpolygon = grid_map::Polygon();
          for(auto p: polygon->getInteriorRing(i))
          {
            MapPoint wp;
            if(llToMap(p.getX(), p.getY(), wp.x, wp.y))
              rings.back().push_back(wp);
              //gpolygon.addVertex(grid_map::Position(wp.x, wp.y));
          }
          if(!polygon->getInteriorRing(i)->get_IsClosed())
            rings.back().push_back(rings.back().front()); // close the ring if necessary
            //gpolygon.addVertex(gpolygon.getVertex(0));
          //for(grid_map::PolygonIterator iterator(grid_map, gpolygon); !iterator.isPastEnd(); ++iterator)
            //grid_map.at("temp", *iterator) = -1.0;
        }

        // for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
        //   if(grid_map.at("temp", *iterator) == 1.0)
        //     updateCost(grid_map, *iterator, value, layer, lower);
        // grid_map.erase("temp");
      }

      for(int row = 0; row < grid_map.getSize()[1]; row++)
      {
        double wy;
        wy = grid_map.getPosition().y()-grid_map.getLength().y()/2.0;
        wy += grid_map.getResolution()*row;

        std::set<double> nodes;

        for(auto ring: rings)
        {
          for(int i = 0; i < ring.size()-1; i++)
          {
            if(   ring[i].y < wy && ring[i+1].y >= wy 
              || ring[i+1].y < wy && ring[i].y >= wy)
            {
              nodes.insert(ring[i].x+(wy-ring[i].y)/(ring[i+1].y-ring[i].y)*(ring[i+1].x-ring[i].x));
            }
          }
        }

        auto node = nodes.begin();
        while(node != nodes.end())
        {
          auto next_node = node;
          next_node++;
          if(next_node == nodes.end())
            break;

          if(*node > grid_map.getPosition().x() + grid_map.getLength().x()/2.0)
            break;

          if(*next_node > grid_map.getPosition().x() - grid_map.getLength().x()/2.0)
          {
            for(auto x = *node; x <= *next_node; x += grid_map.getResolution())
            {
              grid_map::Index i;
              if(grid_map.getIndex(grid_map::Position(x, wy),i))
                updateCost(grid_map, i, value, layer, lower);
            }
          }
          node = next_node;
          node++;
        }
      }

      break;
    }
    default:
      ROS_WARN_STREAM("geometry type not handled: " << geometry->getGeometryType());
  }

}

void GridCreationContext::updateCost(grid_map::GridMap& grid_map, const grid_map::Index& index,  double value, std::string layer, bool lower)
{
  auto existing_cost = grid_map.at(layer, index);
  if(isnan(existing_cost) || ( (!lower && value > existing_cost) || (lower && value < existing_cost)))
    grid_map.at(layer, index) = value;
}

} // namespace s57_grids

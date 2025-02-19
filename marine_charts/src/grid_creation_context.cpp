#include "marine_charts/grid_creation_context.h"
#include "geometry_msgs/msg/point.hpp"
#include "grid_map_core/iterators/LineIterator.hpp"
#include "ogrsf_frmts.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace marine_charts
{

GridCreationContext::GridCreationContext(std::string map_frame, tf2_ros::Buffer &tf_buffer, double resolution_factor, rclcpp::Logger logger):
  resolution_factor_(resolution_factor),
  logger_(logger)
{
  earth_to_map_ = tf_buffer.lookupTransform(map_frame, "earth", tf2::TimePointZero);
}

double GridCreationContext::resolution_factor() const
{
  return resolution_factor_;
}

const geometry_msgs::msg::TransformStamped& GridCreationContext::earth_to_map() const
{
  return earth_to_map_;
}

bool GridCreationContext::ecefToMap(double x, double y, double z, double &mx, double &my)
{
  geometry_msgs::msg::Point ecef;
  ecef.x = x;
  ecef.y = y;
  ecef.z = z;
  geometry_msgs::msg::Point map;
  tf2::doTransform(ecef, map, earth_to_map_);
  mx = map.x;
  my = map.y;
  return true;
}

bool GridCreationContext::llToMap(double lat, double lon, double &x, double &y)
{
  // delay creation to make sure it gets created in thread where it
  // will be used
  if(!ll_to_earth_)
  {
    OGRSpatialReference wgs84, ecef;
    wgs84.SetWellKnownGeogCS("WGS84");
    ecef.importFromEPSG(4978);
    ll_to_earth_ = std::shared_ptr<OGRCoordinateTransformation>(OGRCreateCoordinateTransformation(&wgs84, &ecef), OCTDestroyCoordinateTransformation);
  }
  double alt = 0.0;
  if(ll_to_earth_->Transform(1, &lat, &lon, &alt))
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
    RCLCPP_DEBUG_STREAM(logger_, "null geometry in rasterize");
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
      for(auto& p: *lineString)
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
    {
      OGRPolygon* polygon = geometry->toPolygon();

      std::vector<std::vector<MapPoint> > rings;
      rings.push_back(std::vector<MapPoint>());

      auto er = polygon->getExteriorRing();
      if(!er)
        RCLCPP_DEBUG_STREAM(logger_, "null exterior ring");
      else
      {
        for(auto& p: er)
        {
          MapPoint wp;
          if(llToMap(p.getY(), p.getX(), wp.x, wp.y))
            rings.back().push_back(wp);
        }
        if(!er->get_IsClosed())
          rings.back().push_back(rings.back().front()); // close the ring if necessary

        for(int i = 0; i < polygon->getNumInteriorRings(); i++)
        {
          rings.push_back(std::vector<MapPoint>());
          for(auto& p: polygon->getInteriorRing(i))
          {
            MapPoint wp;
            if(llToMap(p.getX(), p.getY(), wp.x, wp.y))
              rings.back().push_back(wp);
          }
          if(!polygon->getInteriorRing(i)->get_IsClosed())
            rings.back().push_back(rings.back().front()); // close the ring if necessary
        }
      }

      // Efficient Polygon Fill Algorithm
      // http://alienryderflex.com/polygon_fill/
      // for each raster row...
      for(int row = 0; row < grid_map.getSize()[1]; row++)
      {
        double wy;
        wy = grid_map.getPosition().y()-grid_map.getLength().y()/2.0;
        wy += grid_map.getResolution()*row;

        std::set<double> nodes;

        // find all the intersections with polygon edges
        // the ordered intersections wil be where we enter
        // then exit the polygon, which can happen multiple times
        for(auto ring: rings)
        {
          for(std::size_t i = 0; i+1 < ring.size(); i++)
          {
            if(  (ring[i].y < wy && ring[i+1].y >= wy) 
              || (ring[i+1].y < wy && ring[i].y >= wy))
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
          // increment twice essentially, to skip over a section outside the polygon
          node = next_node;
          node++;
        }
      }

      break;
    }
    default:
      RCLCPP_DEBUG_STREAM(logger_, "geometry type not handled: " << geometry->getGeometryType());
  }

}

void GridCreationContext::updateCost(grid_map::GridMap& grid_map, const grid_map::Index& index,  double value, std::string layer, bool lower)
{
  auto existing_cost = grid_map.at(layer, index);
  if(std::isnan(existing_cost) || ( (!lower && value > existing_cost) || (lower && value < existing_cost)))
    grid_map.at(layer, index) = value;
}

} // namespace marine_charts

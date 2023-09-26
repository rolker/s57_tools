#include "s57_grids/grid_publisher.h"
#include <ros/package.h>
#include <s57_grids/s57_dataset.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ogrsf_frmts.h"

namespace s57_grids
{

GridPublisher::GridPublisher()
{
  std::string enc_root = ros::package::getPath("s57_grids")+"/data/ENC_ROOT";
  enc_root = ros::param::param("~enc_root",enc_root);
  catalog_ = std::make_shared<s57_grids::S57Catalog>(enc_root);

  resolution_factor_ = ros::param::param("~resolution_factor", resolution_factor_);
  map_frame_ = ros::param::param("~map_frame", map_frame_);
  buffer_radius_ = ros::param::param("~buffer_radius", buffer_radius_);

  robot_.frame_id = ros::param::param("~robot/frame_id", robot_.frame_id);
  robot_.minimum_depth = ros::param::param("~robot/minimum_depth", robot_.minimum_depth);
  robot_.maximum_caution_depth = ros::param::param("~robot/maximum_caution_depth", robot_.maximum_caution_depth);
  robot_.overhead_clearance = ros::param::param("~robot/overhead_clearance", robot_.overhead_clearance);
  robot_.minimum_speed = ros::param::param("~robot/minimum_speed", robot_.minimum_speed);
  robot_.maximum_speed = ros::param::param("~robot/maximum_speed", robot_.maximum_speed);

  if (ros::param::has("~grids"))
  {
    XmlRpc::XmlRpcValue grid_list;
    if(ros::param::get("~grids", grid_list))
    {
      if(grid_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
        ROS_ERROR_STREAM("grids parameter is not an array");
      else
      {
        for(int32_t i = 0; i < grid_list.size(); ++i)
        {
          std::string name;
          try
          {
            name = static_cast<std::string>(grid_list[i]["name"]);
          }
          catch (XmlRpc::XmlRpcException e)
          {
            ROS_ERROR_STREAM("Error reading grid number " << i << " name: " << e.getMessage());
            continue;
          }
          output_grids_[name].name = name;
          try
          {
            if(grid_list[i]["resolution"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
              output_grids_[name].resolution = static_cast<double>(grid_list[i]["resolution"]);
            else
              output_grids_[name].resolution = static_cast<int>(grid_list[i]["resolution"]);

            if(grid_list[i]["length"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
              output_grids_[name].length = static_cast<double>(grid_list[i]["length"]);
            else
              output_grids_[name].length = static_cast<int>(grid_list[i]["length"]);
            if(grid_list[i]["period"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
              output_grids_[name].period = static_cast<double>(grid_list[i]["period"]);
            else
              output_grids_[name].period = static_cast<int>(grid_list[i]["period"]);
          }
          catch (XmlRpc::XmlRpcException e)
          {
            ROS_ERROR_STREAM("Error reading grid parameters for " << name << ": " << e.getMessage());
            continue;
          }
        }
      }
    }
  }

  ros::NodeHandle n("~");

  list_service_ = n.advertiseService("list_datasets", &GridPublisher::listDatasets, this);
  get_service_ = n.advertiseService("get_datasets", &GridPublisher::getDatasets, this);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

  for(auto& g: output_grids_)
  {
    g.second.publisher = n.advertise<grid_map_msgs::GridMap>("grids/"+g.first, 1, true);
    g.second.thread = std::thread(&GridPublisher::updateGrid, this, std::ref(g.second));
  }

  new_grids_timer_ = ros::NodeHandle().createTimer(ros::Duration(1.0), &GridPublisher::checkForNewGrids, this);    
}

GridPublisher::~GridPublisher()
{
  catalog_.reset();
  {
    std::lock_guard<std::mutex> lock(abort_flag_mutex_);
    abort_flag_ = true;
  }
  for(auto& g: output_grids_)
    g.second.thread.join();
  for(auto& p: pending_dataset_grids_)
    if(p.second.valid())
      p.second.wait();
}


bool GridPublisher::listDatasets(s57_msgs::GetDatasets::Request &req, s57_msgs::GetDatasets::Response &res)
{
  for(auto d: catalog_->intersectingCharts(req.bounds))
    if(d->chartScale() > req.minimum_scale)
    {
      s57_msgs::DatasetInfo di;
      di.bounds = d->getBounds();
      di.label = d->label();
      di.scale = d->chartScale();
      di.resolution = d->recommendedResolution();
      di.topic = d->topic();
      res.datasets.push_back(di);
    }
  return true;
}

bool GridPublisher::getDatasets(s57_msgs::GetDatasets::Request &req, s57_msgs::GetDatasets::Response &res)
{
  bool ret = listDatasets(req, res);
  if(ret)
  {
    std::lock_guard<std::mutex> lock(requested_grids_mutex);
    for(auto d: res.datasets)
    {
      requested_grids_.push_back(d.label);
      requested_grids_to_publish_.push_back(d.label);
    }
  }
  return ret;
}

void GridPublisher::checkForNewGrids(const ros::TimerEvent& event)
{
  {
    std::lock_guard<std::mutex> lock(requested_grids_mutex);
    for(auto r: requested_grids_)
    {
      auto ds = catalog_->dataset(r);
      if(grid_publishers_.count(ds->label()) == 0 && pending_dataset_grids_.count(ds->label()) == 0 && pending_buffered_grids_.count(ds->label()) == 0)
      {
        pending_dataset_grids_[ds->label()] = std::async(&S57Dataset::getGrid, ds, GridCreationContext(map_frame_, tf_buffer_, resolution_factor_) );
      }
    }
    requested_grids_.clear();
  }

  std::vector<std::string> done_grids;
  ros::NodeHandle n("~");
  for(auto& pg: pending_dataset_grids_)
  {
    if(pg.second.valid())
      if(pg.second.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
      {
        auto grid = pg.second.get();
        pending_buffered_grids_[pg.first] = std::async(&GridPublisher::bufferGrid, this, grid);
        done_grids.push_back(pg.first);
      }
  }
  for(auto d: done_grids)
    pending_dataset_grids_.erase(d);

  done_grids.clear();

  for(auto& pg: pending_buffered_grids_)
  {
    if(pg.second.valid())
      if(pg.second.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
      {
        auto grid = pg.second.get();

        {
          std::lock_guard<std::mutex> lock(requested_grids_mutex);
          if(std::find(requested_grids_to_publish_.begin(), requested_grids_to_publish_.end(), pg.first) != requested_grids_to_publish_.end())
          {
            auto ds = catalog_->dataset(pg.first);
            grid_publishers_[pg.first] = n.advertise<grid_map_msgs::GridMap>("datasets/"+ds->topic(), 1, true);
            grid_map_msgs::GridMap message;
            grid_map::GridMapRosConverter::toMessage(*grid, message);
            ROS_DEBUG_STREAM("Publishing grid to " << "datasets/" << pg.first);
            grid_publishers_[pg.first].publish(message);
          }
          else
            grid_publishers_[pg.first]; // create the entry in the map so above check to see if we need to generate a grid works.
        }

        done_grids.push_back(pg.first);

        std::lock_guard<std::mutex> lock(dataset_grids_mutex_);
        dataset_grids_[pg.first] = grid;
      }
  }
  for(auto d: done_grids)
    pending_buffered_grids_.erase(d);
}

std::shared_ptr<grid_map::GridMap> GridPublisher::bufferGrid(std::shared_ptr<grid_map::GridMap> grid)
{
  grid->add("speed");
  for(grid_map::GridMapIterator i(*grid); !i.isPastEnd(); ++i)
  {
    {
      // Check if we need to quit
      std::lock_guard<std::mutex> abort_lock(abort_flag_mutex_);
      if(abort_flag_)
        return std::shared_ptr<grid_map::GridMap>();
    }

    auto speed = std::nan("");
    if(!isnan(grid->at("restricted", *i)))
      speed = -1.0;
    else
    {
      auto overhead = grid->at("overhead", *i);
      if(!isnan(overhead) && overhead <= robot_.overhead_clearance)
      {
        speed = -1.0;
      }
      else
      {
        auto elevation = grid->at("elevation", *i);
        if(!isnan(elevation))
        {
          auto depth = -elevation;
          if (depth < 0.0)
            speed = -1.0;
          else if (depth < robot_.minimum_depth)
            speed = 0.0;
          else if(depth > robot_.maximum_caution_depth)
            speed = robot_.maximum_speed;
          else
            speed = robot_.minimum_speed+(robot_.maximum_speed-robot_.minimum_speed)*(depth-robot_.minimum_depth)/(robot_.maximum_caution_depth-robot_.minimum_depth);
          if(!isnan(grid->at("unsurveyed", *i)) || !isnan(grid->at("caution", *i)))
            if(isnan(speed))
              speed = robot_.minimum_speed;
            else
              speed = std::max(robot_.minimum_speed, 0.5*speed);
        }
      }
    } 
    if(!isnan(speed))
      grid->at("speed", *i) = speed;
  }

  return grid;

  grid->add("buffered_speed", grid->get("speed"));
  for(grid_map::GridMapIterator i(*grid); !i.isPastEnd(); ++i)
  {
    {
      // Check if we need to quit
      std::lock_guard<std::mutex> abort_lock(abort_flag_mutex_);
      if(abort_flag_)
        return std::shared_ptr<grid_map::GridMap>();
    }
    auto speed = grid->at("speed", *i);
    if(isnan(speed))
    {
      grid_map::Position center;
      if (grid->getPosition(*i, center))
      {
        bool need_buffering = false;
        for(grid_map::CircleIterator ci(*grid, center, grid->getResolution()); !ci.isPastEnd(); ++ci)
          if(!isnan(grid->at("speed", *ci)))
          {
            need_buffering = true;
            break;
          }
        if(need_buffering)
          for(grid_map::CircleIterator ci(*grid, center, buffer_radius_); !ci.isPastEnd(); ++ci)
          {
            if(!isnan(grid->at("speed", *ci)))
            {
              grid_map::Position position;
              if(grid->getPosition(*ci, position))
              {
                auto distance_factor = (position-center).norm()/buffer_radius_;
                grid->at("buffered_speed", *ci) = std::min(double(grid->at("buffered_speed", *ci)), distance_factor*robot_.maximum_speed);
              }
            }
          }
      }
    }    
  }
  return grid;
}

void GridPublisher::updateGrid(GridOutput& output_grid)
{
  OGRSpatialReference wgs84, ecef;
  wgs84.SetWellKnownGeogCS("WGS84");
  ecef.importFromEPSG(4978);
  auto earth_to_ll = std::shared_ptr<OGRCoordinateTransformation>(OGRCreateCoordinateTransformation(&ecef, &wgs84), OCTDestroyCoordinateTransformation);

  ros::Time next_publish_time = ros::Time::now();

  grid_map::GridMap grid;
  grid.setGeometry(grid_map::Length(output_grid.length, output_grid.length), output_grid.resolution);

  grid.add("speed");
  grid.add("source_resolution");

  grid.setFrameId(map_frame_);

  while(true)
  {
    {
      // Check if we need to quit
      std::lock_guard<std::mutex> abort_lock(abort_flag_mutex_);
      if(abort_flag_)
        break;
    }

    auto retry = std::chrono::seconds(1);

    std::vector<std::string> dataset_labels;

    geometry_msgs::PointStamped center;
    center.header.frame_id = robot_.frame_id;
    center.header.stamp = ros::Time::now();
    try
    {
      auto center_map = tf_buffer_.transform(center, map_frame_, ros::Duration(1.0));
      auto lower_left = center_map;
      double half_length = output_grid.length/2.0;
      lower_left.point.x -= half_length;
      lower_left.point.y -= half_length;
      auto upper_right = center_map;
      upper_right.point.x += half_length;
      upper_right.point.y += half_length;
      
      auto ll_earth = tf_buffer_.transform(lower_left, "earth", ros::Duration(1.0));
      auto ur_earth = tf_buffer_.transform(upper_right, "earth", ros::Duration(1.0));
      if(earth_to_ll->Transform(1, &ll_earth.point.x, &ll_earth.point.y, &ll_earth.point.z) &&
         earth_to_ll->Transform(1, &ur_earth.point.x, &ur_earth.point.y, &ur_earth.point.z))
      {
        geographic_msgs::BoundingBox bounds;
        bounds.min_pt.latitude = ll_earth.point.x;
        bounds.min_pt.longitude = ll_earth.point.y;
        bounds.max_pt.latitude = ur_earth.point.x;
        bounds.max_pt.longitude = ur_earth.point.y;
        
        grid.move(grid_map::Position(center_map.point.x, center_map.point.y));

        auto datasets = catalog_->intersectingCharts(bounds);
        for(auto d: datasets)
        {
          if(d->recommendedResolution()*resolution_factor_ >= output_grid.resolution)
            dataset_labels.push_back(d->label());
        }

        std::lock_guard<std::mutex> lock(requested_grids_mutex);
        for(auto dl: dataset_labels)
        {
          requested_grids_.push_back(dl);
          //else
          //  ROS_INFO_STREAM(output_grid.name << " at res: " << output_grid.resolution << " skipping " << d->label() << " scale " << d->chartScale() << " rec. res. " << d->recommendedResolution());
        }
      }

      std::map<double, std::vector<std::shared_ptr<grid_map::GridMap> > > grids_by_resolution;
      
      for(auto dl: dataset_labels)
      {
        std::lock_guard<std::mutex> lock(dataset_grids_mutex_);
        if (dataset_grids_.count(dl))
          grids_by_resolution[dataset_grids_[dl]->getResolution()] .push_back(dataset_grids_[dl]);
      }

      for(grid_map::GridMapIterator i(grid); !i.isPastEnd(); ++i)
      {
        grid_map::Position position;
        if(grid.getPosition(*i, position))
        {
          auto source_resolution = grid.at("source_resolution", *i);
          for(auto& gr: grids_by_resolution)
          {
            if(!isnan(source_resolution) && gr.first >= source_resolution)
              break;
            for(auto g: gr.second)
            {
              grid_map::Index from_index;
              if(g->getIndex(position, from_index))
              {
                auto speed = g->at("speed", from_index);
                if(!isnan(speed))
                {
                  grid.at("speed", *i) = speed;
                  grid.at("source_resolution", *i) = gr.first;
                  source_resolution = gr.first;
                }

              }
            }
          }
        }
      }

      if(ros::Time::now() >= next_publish_time)
      {
        grid.setTimestamp(ros::Time::now().toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(grid, message);
        output_grid.publisher.publish(message);
        next_publish_time += ros::Duration(output_grid.period);
      }
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
    std::this_thread::sleep_for(retry);
    
  }
}

} // namespace s57_grids

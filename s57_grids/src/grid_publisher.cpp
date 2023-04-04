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
      requested_grids_.push_back(d.label);
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
      if(grid_publishers_.count(ds->label()) == 0 && pending_dataset_grids_.count(ds->label()) == 0)
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
        auto ds = catalog_->dataset(pg.first);
        grid_publishers_[pg.first] = n.advertise<grid_map_msgs::GridMap>("datasets/"+ds->topic(), 1, true);
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(*grid, message);
        ROS_INFO_STREAM("Publishing grid to " << "datasets/" << pg.first);
        grid_publishers_[pg.first].publish(message);
        done_grids.push_back(pg.first);

        dataset_grids_[pg.first] = grid;
      }
  }
  for(auto d: done_grids)
    pending_dataset_grids_.erase(d);
}


void GridPublisher::updateGrid(GridOutput& output_grid)
{
  OGRSpatialReference wgs84, ecef;
  wgs84.SetWellKnownGeogCS("WGS84");
  ecef.importFromEPSG(4978);
  auto earth_to_ll = std::shared_ptr<OGRCoordinateTransformation>(OGRCreateCoordinateTransformation(&ecef, &wgs84), OCTDestroyCoordinateTransformation);

  // double alt = 0.0;
  // if(ll_to_earth->Tr Transform(1, &lat, &lon, &alt))
  // {
  //   return ecefToMap(lat, lon, alt, x, y);
  // }

  grid_map::GridMap grid;
  grid.setGeometry(grid_map::Length(output_grid.length, output_grid.length), output_grid.resolution);

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
      auto center_map = tf_buffer_.transform(center, map_frame_);
      auto lower_left = center_map;
      double half_length = output_grid.length/2.0;
      lower_left.point.x -= half_length;
      lower_left.point.y -= half_length;
      auto upper_right = center_map;
      upper_right.point.x += half_length;
      upper_right.point.y += half_length;
      
      auto ll_earth = tf_buffer_.transform(lower_left, "earth");
      auto ur_earth = tf_buffer_.transform(upper_right, "earth");
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

      std::vector<std::shared_ptr<grid_map::GridMap> > grids;
      


    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
    std::this_thread::sleep_for(retry);
    
  }
}

} // namespace s57_grids

#include "grid_publisher.h"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "marine_charts/s57_dataset.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cmath>
#include <thread>
#include <unordered_set>

namespace s57_grids
{

GridPublisher::GridPublisher(const std::string &node_name)
  : rclcpp_lifecycle::LifecycleNode(node_name)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GridPublisher::on_configure(const rclcpp_lifecycle::State &state)
{
  std::string enc_root;
  if(!has_parameter("enc_root"))
  {
    if(const char * enc_root_env = std::getenv("ROS_S57_ENC_ROOT"))
    {
      enc_root = enc_root_env;
      RCLCPP_INFO_STREAM(get_logger(), "Initializing parameter with ENC_ROOT from environment: " << enc_root);
    }
    declare_parameter("enc_root", rclcpp::ParameterValue(enc_root));
  }
  get_parameter("enc_root", enc_root);

  catalog_ = std::make_shared<marine_charts::S57Catalog>(enc_root);

  if(!has_parameter("resolution_factor"))
  {
    declare_parameter("resolution_factor", rclcpp::ParameterValue(resolution_factor_));
  }
  resolution_factor_ = get_parameter("resolution_factor").as_double();

  if(!has_parameter("map_frame"))
  {
    declare_parameter("map_frame", rclcpp::ParameterValue(map_frame_));
  }
  map_frame_ = get_parameter("map_frame").as_string();

  if(!has_parameter("grid_republish_period"))
  {
    declare_parameter("grid_republish_period", rclcpp::ParameterValue(grid_republish_period_));
  }
  grid_republish_period_ = get_parameter("grid_republish_period").as_double();
  if(grid_republish_period_ > 0.0)
  {
    republish_grids_timer_ = create_wall_timer(
      std::chrono::milliseconds(int(1000.0*grid_republish_period_)),
      std::bind(&GridPublisher::republishGrids, this)
    );
  }

  if(!has_parameter("publish_costmaps"))
  {
    declare_parameter("publish_costmaps", rclcpp::ParameterValue(publish_costmaps_));
  }
  publish_costmaps_ = get_parameter("publish_costmaps").as_bool();

  const double default_check_new_grids_period = check_new_grids_period_;
  if(!has_parameter("check_new_grids_period"))
  {
    declare_parameter("check_new_grids_period", rclcpp::ParameterValue(check_new_grids_period_));
  }
  check_new_grids_period_ = get_parameter("check_new_grids_period").as_double();
  if(!std::isfinite(check_new_grids_period_) || check_new_grids_period_ <= 0.0)
  {
    RCLCPP_WARN_STREAM(get_logger(),
      "Invalid check_new_grids_period value " << check_new_grids_period_
      << " s; using " << default_check_new_grids_period << " s instead.");
    check_new_grids_period_ = default_check_new_grids_period;
  }

  if(!has_parameter("robot_base_frame"))
  {
    declare_parameter("robot_base_frame", rclcpp::ParameterValue(robot_base_frame_));
  }
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();

  const double default_precompute_radius = precompute_radius_;
  if(!has_parameter("precompute_radius"))
  {
    declare_parameter("precompute_radius", rclcpp::ParameterValue(precompute_radius_));
  }
  precompute_radius_ = get_parameter("precompute_radius").as_double();
  if(!std::isfinite(precompute_radius_) || precompute_radius_ < 0.0)
  {
    RCLCPP_WARN_STREAM(get_logger(),
      "Invalid precompute_radius value " << precompute_radius_
      << " m; using " << default_precompute_radius << " m instead.");
    precompute_radius_ = default_precompute_radius;
  }

  list_service_ = create_service<s57_msgs::srv::GetDatasets>(
    "list_datasets",
    std::bind(&GridPublisher::listDatasets, this, std::placeholders::_1, std::placeholders::_2)
  );

  get_service_ = create_service<s57_msgs::srv::GetDatasets>(
    "get_datasets",
    std::bind(&GridPublisher::getDatasets, this, std::placeholders::_1, std::placeholders::_2)
  );

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Clamp to >= 1 ms — a zero-duration wall timer is undefined behaviour
  // and would also pin a CPU.
  const int new_grids_timer_ms = std::max(1, int(1000.0 * check_new_grids_period_));
  new_grids_timer_ = create_wall_timer(
    std::chrono::milliseconds(new_grids_timer_ms),
    std::bind(&GridPublisher::checkForNewGrids, this)
  );

  if(!robot_base_frame_.empty() && precompute_radius_ > 0.0)
  {
    RCLCPP_INFO_STREAM(get_logger(),
      "Precompute enabled: will queue charts within " << precompute_radius_
      << " m of " << robot_base_frame_ << " when TF first becomes available.");
  }

  return LifecycleNode::on_configure(state);

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GridPublisher::on_activate(const rclcpp_lifecycle::State &state)
{
  for(auto& g: grid_publishers_)
    g.second->on_activate();

  return LifecycleNode::on_activate(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GridPublisher::on_cleanup(const rclcpp_lifecycle::State &state)
{
  abort_flag_ = true;

  list_service_.reset();
  get_service_.reset();

  new_grids_timer_.reset();

  catalog_.reset();
  for(auto& p: pending_dataset_grids_)
    if(p.second.valid())
      p.second.wait();

  tf_listener_.reset();
  tf_buffer_.reset();

  return LifecycleNode::on_cleanup(state);
}


void GridPublisher::listDatasets(
  s57_msgs::srv::GetDatasets::Request::ConstSharedPtr request,
  s57_msgs::srv::GetDatasets::Response::SharedPtr response)
{
  for(auto d: catalog_->intersectingCharts(request->bounds))
    if(d->chartScale() > request->minimum_scale)
    {
      s57_msgs::msg::DatasetInfo di;
      di.bounds = d->getBounds();
      di.label = d->label();
      di.scale = d->chartScale();
      di.resolution = d->recommendedResolution();
      di.topic = d->topic();
      response->datasets.push_back(di);
    }
}

void GridPublisher::getDatasets(
  s57_msgs::srv::GetDatasets::Request::ConstSharedPtr request,
  s57_msgs::srv::GetDatasets::Response::SharedPtr response)
{
  listDatasets(request, response);
  std::lock_guard<std::mutex> lock(requested_grids_mutex);
  auto now = get_clock()->now();
  for(const auto& d: response->datasets)
  {
    requested_grids_.push_back(d.label);
    requested_grids_to_publish_.insert(d.label);
    // Only record start time on the first request for this label, so
    // re-requests don't reset the measurement.
    grid_request_start_times_.emplace(d.label, now);
  }
}

void GridPublisher::checkForNewGrids()
{
  // Try a one-shot eager-precompute on the first tick where the boat's
  // pose is known. Cheap when disabled (empty robot_base_frame_) or
  // already done.
  if(!did_precompute_ && !robot_base_frame_.empty() && precompute_radius_ > 0.0)
  {
    tryPrecompute();
  }

  {
    std::lock_guard<std::mutex> lock(requested_grids_mutex);
    for(auto r: requested_grids_)
    {
      auto ds = catalog_->dataset(r);
      if(grid_publishers_.count(ds->label()) == 0 && pending_dataset_grids_.count(ds->label()) == 0)
      {
        pending_dataset_grids_[ds->label()] = std::async(
          &marine_charts::S57Dataset::getGrid,
          ds,
          marine_charts::GridCreationContext(map_frame_, *tf_buffer_, resolution_factor_, get_logger()),
          std::ref(abort_flag_)
        );
      }
    }
    requested_grids_.clear();
  }

  std::vector<std::string> done_grids;
  for(auto& pg: pending_dataset_grids_)
  {
    if(pg.second.valid())
      if(pg.second.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
      {
        auto grid = pg.second.get();
        {
          std::lock_guard<std::mutex> lock(requested_grids_mutex);
          if(requested_grids_to_publish_.count(pg.first) > 0)
          {
            auto ds = catalog_->dataset(pg.first);

            rclcpp::QoS latched_qos(1);
            latched_qos.transient_local();
            latched_qos.keep_last(1);

            grid_publishers_[pg.first] =
              create_publisher<grid_map_msgs::msg::GridMap>(
                "datasets/"+ds->topic(), latched_qos);
            grid_publishers_[pg.first]->on_activate();

            auto message = grid_map::GridMapRosConverter::toMessage(*grid);
            // Report time from first request to publish for this chart.
            // Useful for diagnosing cold-cache behaviour without recompiling.
            auto start_it = grid_request_start_times_.find(pg.first);
            if(start_it != grid_request_start_times_.end())
            {
              auto elapsed = (get_clock()->now() - start_it->second).seconds();
              RCLCPP_INFO_STREAM(get_logger(),
                "Grid ready: " << pg.first << " (" << elapsed << " s from request to publish)");
              grid_request_start_times_.erase(start_it);
            }
            else
            {
              RCLCPP_INFO_STREAM(get_logger(), "Grid ready: " << pg.first);
            }
            grid_publishers_[pg.first]->publish(*message);

            if(publish_costmaps_)
            {
              costmap_publishers_[pg.first] =
                create_publisher<nav_msgs::msg::OccupancyGrid>(
                  "datasets/occupancy_grids/"+ds->topic(), latched_qos);
              costmap_publishers_[pg.first]->on_activate();
              nav_msgs::msg::OccupancyGrid occupancy_grid;
              grid_map::GridMapRosConverter::toOccupancyGrid(*grid, "elevation", -10.0, 0.0, occupancy_grid);
              costmap_publishers_[pg.first]->publish(occupancy_grid);
            }

            requested_grids_to_publish_.erase(pg.first);
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
    pending_dataset_grids_.erase(d);

  done_grids.clear();

}

void GridPublisher::tryPrecompute()
{
  // Look up the boat's current pose in the map frame. If not yet
  // available (TF tree still bootstrapping), silently retry next tick.
  geometry_msgs::msg::TransformStamped pose_in_map;
  try
  {
    pose_in_map = tf_buffer_->lookupTransform(
      map_frame_, robot_base_frame_, tf2::TimePointZero);
  }
  catch(const tf2::TransformException&)
  {
    return;
  }

  // Convert (x, y, z) from map frame to ECEF, then to lat/lon. The
  // catalog needs lat/lon to find intersecting charts.
  geometry_msgs::msg::PointStamped map_pt;
  map_pt.header = pose_in_map.header;
  map_pt.point.x = pose_in_map.transform.translation.x;
  map_pt.point.y = pose_in_map.transform.translation.y;
  map_pt.point.z = pose_in_map.transform.translation.z;
  geometry_msgs::msg::PointStamped ecef_pt;
  try
  {
    tf_buffer_->transform(map_pt, ecef_pt, "earth");
  }
  catch(const tf2::TransformException&)
  {
    return;
  }
  double lat = 0.0, lon = 0.0;
  if(!catalog_->ecefToLatLong(ecef_pt.point.x, ecef_pt.point.y, ecef_pt.point.z, lat, lon))
    return;

  // Equirectangular approximation: 1° lat ≈ 111 000 m everywhere;
  // 1° lon ≈ 111 000 m × cos(lat). Good to <1% for radii up to ~50 km
  // outside polar regions.
  const double meters_per_deg_lat = 111000.0;
  const double meters_per_deg_lon = 111000.0 * std::cos(lat * M_PI / 180.0);
  const double lat_delta = precompute_radius_ / meters_per_deg_lat;

  // Latitude is bounded; clamp to the physically meaningful range.
  const double lat_min = std::max(-90.0, lat - lat_delta);
  const double lat_max = std::min( 90.0, lat + lat_delta);

  // Longitude wraps. Build one or two query boxes:
  //   - Polar region (cos(lat) ≈ 0): meters_per_deg_lon → 0 makes the
  //     equirectangular lon span effectively global; use [-180, 180].
  //   - Radius spans the globe (lon_delta >= 180): same.
  //   - Crosses the dateline: split into two boxes, since
  //     S57Dataset::intersects assumes minLon <= maxLon and does not wrap.
  std::vector<geographic_msgs::msg::BoundingBox> query_bounds;
  constexpr double kMinMetersPerDegLon = 1.0;  // ~89.99999° lat
  if(std::abs(meters_per_deg_lon) <= kMinMetersPerDegLon)
  {
    geographic_msgs::msg::BoundingBox b;
    b.min_pt.latitude = lat_min;
    b.max_pt.latitude = lat_max;
    b.min_pt.longitude = -180.0;
    b.max_pt.longitude =  180.0;
    query_bounds.push_back(b);
  }
  else
  {
    const double lon_delta = precompute_radius_ / meters_per_deg_lon;
    if(lon_delta >= 180.0)
    {
      geographic_msgs::msg::BoundingBox b;
      b.min_pt.latitude = lat_min;
      b.max_pt.latitude = lat_max;
      b.min_pt.longitude = -180.0;
      b.max_pt.longitude =  180.0;
      query_bounds.push_back(b);
    }
    else
    {
      const double lon_min = lon - lon_delta;
      const double lon_max = lon + lon_delta;
      if(lon_min < -180.0)
      {
        geographic_msgs::msg::BoundingBox west, east;
        west.min_pt.latitude = lat_min; west.max_pt.latitude = lat_max;
        east.min_pt.latitude = lat_min; east.max_pt.latitude = lat_max;
        west.min_pt.longitude = lon_min + 360.0;
        west.max_pt.longitude =  180.0;
        east.min_pt.longitude = -180.0;
        east.max_pt.longitude = lon_max;
        query_bounds.push_back(west);
        query_bounds.push_back(east);
      }
      else if(lon_max > 180.0)
      {
        geographic_msgs::msg::BoundingBox west, east;
        west.min_pt.latitude = lat_min; west.max_pt.latitude = lat_max;
        east.min_pt.latitude = lat_min; east.max_pt.latitude = lat_max;
        west.min_pt.longitude = lon_min;
        west.max_pt.longitude =  180.0;
        east.min_pt.longitude = -180.0;
        east.max_pt.longitude = lon_max - 360.0;
        query_bounds.push_back(west);
        query_bounds.push_back(east);
      }
      else
      {
        geographic_msgs::msg::BoundingBox b;
        b.min_pt.latitude = lat_min;
        b.max_pt.latitude = lat_max;
        b.min_pt.longitude = lon_min;
        b.max_pt.longitude = lon_max;
        query_bounds.push_back(b);
      }
    }
  }

  std::vector<std::shared_ptr<marine_charts::S57Dataset>> charts;
  std::unordered_set<std::string> seen_labels;
  for(const auto& qb: query_bounds)
  {
    for(const auto& chart: catalog_->intersectingCharts(qb))
    {
      if(seen_labels.insert(chart->label()).second)
        charts.push_back(chart);
    }
  }
  if(charts.empty())
  {
    RCLCPP_WARN_STREAM(get_logger(),
      "Precompute: no charts intersect " << precompute_radius_ << " m radius around ("
      << lat << ", " << lon << ").");
    did_precompute_ = true;
    return;
  }

  // Queue the chart labels via the same path getDatasets uses (both
  // requested_grids_ to trigger generation and requested_grids_to_publish_
  // so the latched topic message gets published when ready — late-arriving
  // S57Layer subscribers see the cached message thanks to transient_local
  // QoS). Labels already in flight (tracked via grid_request_start_times_)
  // are skipped to avoid double-queuing.
  std::lock_guard<std::mutex> lock(requested_grids_mutex);
  auto now = get_clock()->now();
  size_t queued = 0;
  for(const auto& chart: charts)
  {
    const std::string& label = chart->label();
    if(grid_request_start_times_.count(label) == 0)
    {
      requested_grids_.push_back(label);
      requested_grids_to_publish_.insert(label);
      grid_request_start_times_.emplace(label, now);
      ++queued;
    }
  }
  RCLCPP_INFO_STREAM(get_logger(),
    "Precompute: queued " << queued << " of " << charts.size()
    << " charts within " << precompute_radius_ << " m of ("
    << lat << ", " << lon << ").");
  did_precompute_ = true;
}

void GridPublisher::republishGrids()
{
  RCLCPP_INFO_STREAM(get_logger(), "Republishing grids");
  for(const auto &grid_publisher: grid_publishers_)
  {
    if(grid_publisher.second && grid_publisher.second->is_activated())
    {
      std::lock_guard<std::mutex> lock(dataset_grids_mutex_);
      const auto &it = dataset_grids_.find(grid_publisher.first);
      if(it != dataset_grids_.end())
      {
        auto grid = it->second;
        if(grid)
        {
          grid->setTimestamp(get_clock()->now().nanoseconds());
          auto message = grid_map::GridMapRosConverter::toMessage(*grid);
          grid_publisher.second->publish(*message);
          RCLCPP_INFO_STREAM(get_logger(), "Republishing grid to " << "datasets/" << grid_publisher.first);
        }
      }
    }
  }
}

} // namespace s57_grids

#include "s57_layer.h"

#include <cmath>

#include "geodesy/ecef.h"
#include "geodesy/wgs84.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "s57_msgs/srv/get_datasets.hpp"
#include <cstdlib>

using namespace std::chrono_literals;

namespace s57_layer
{

S57Layer::S57Layer()
{

}

S57Layer::~S57Layer()
{
}

void S57Layer::onInitialize()
{
  auto node = node_.lock();
  current_ = false;

  // Declare "<layer name>.<param>" on the owning node unless it already is.
  // Plain rclcpp: Nav2 on Rolling removed Layer::declareParameter, and this
  // does what Jazzy's version did, so the same code builds on both.
  auto declareParameter =
    [&node, this](const std::string & param, const rclcpp::ParameterValue & value) {
      const std::string full_name = getFullName(param);
      if (!node->has_parameter(full_name)) {
        node->declare_parameter(full_name, value);
      }
    };

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_+".enabled", enabled_);
  // above doesn't seem to work
  // enabled_ = true;

  declareParameter("minimum_depth", rclcpp::ParameterValue(minimum_depth_));
  node->get_parameter(name_+".minimum_depth", minimum_depth_);

  declareParameter("maximum_caution_depth", rclcpp::ParameterValue(maximum_caution_depth_));
  node->get_parameter(name_+".maximum_caution_depth", maximum_caution_depth_);

  declareParameter("overhead_clearance", rclcpp::ParameterValue(overhead_clearance_));
  node->get_parameter(name_+".overhead_clearance", overhead_clearance_);

  declareParameter("unsurveyed_cost", rclcpp::ParameterValue(unsurveyed_cost_));
  node->get_parameter(name_+".unsurveyed_cost", unsurveyed_cost_);

  // ADR-0010 D10: false suppresses the depth ramp (bathymetry_layer is the
  // depth authority); this layer then paints only land/restricted/overhead/
  // caution/unsurveyed and charted point hazards. Depth-related parameters
  // (minimum_depth, maximum_caution_depth, chart_datum_frame,
  // sea_surface_frame, tide_invalidate_threshold) stay declared but inert.
  declareParameter("depth_costs", rclcpp::ParameterValue(depth_costs_));
  node->get_parameter(name_+".depth_costs", depth_costs_);

  declareParameter("update_timeout", rclcpp::ParameterValue(update_timeout_));
  node->get_parameter(name_+".update_timeout", update_timeout_);
  declareParameter("tile_size", rclcpp::ParameterValue(tile_size_));
  node->get_parameter(name_+".tile_size", tile_size_);

  const double default_buffer_fraction = buffer_fraction_;
  declareParameter("buffer_fraction", rclcpp::ParameterValue(buffer_fraction_));
  node->get_parameter(name_+".buffer_fraction", buffer_fraction_);
  if(!std::isfinite(buffer_fraction_) || buffer_fraction_ < 0.0)
  {
    RCLCPP_WARN_STREAM(logger_,
      "Invalid buffer_fraction value " << buffer_fraction_
      << "; using " << default_buffer_fraction << " instead.");
    buffer_fraction_ = default_buffer_fraction;
  }

  declareParameter("allow_uncharted", rclcpp::ParameterValue(allow_uncharted_));
  node->get_parameter(name_+".allow_uncharted", allow_uncharted_);

  declareParameter("chart_datum_frame", rclcpp::ParameterValue(chart_datum_frame_));
  node->get_parameter(name_+".chart_datum_frame", chart_datum_frame_);

  declareParameter("sea_surface_frame", rclcpp::ParameterValue(sea_surface_frame_));
  node->get_parameter(name_+".sea_surface_frame", sea_surface_frame_);

  const double default_tide_invalidate_threshold = tide_invalidate_threshold_;
  declareParameter("tide_invalidate_threshold", rclcpp::ParameterValue(tide_invalidate_threshold_));
  node->get_parameter(name_+".tide_invalidate_threshold", tide_invalidate_threshold_);
  // ROS 2 parameters are external input — validate. Negative would make
  // every tide change exceed the threshold (continuous invalidation =
  // costmap stalls); non-finite would silently disable invalidation.
  if(!std::isfinite(tide_invalidate_threshold_) || tide_invalidate_threshold_ < 0.0)
  {
    RCLCPP_WARN_STREAM(logger_,
      "Invalid tide_invalidate_threshold value " << tide_invalidate_threshold_
      << " m; using default " << default_tide_invalidate_threshold << " m instead.");
    tide_invalidate_threshold_ = default_tide_invalidate_threshold;
  }

  if(!depth_costs_)
    RCLCPP_INFO_STREAM(logger_,
      "Depth costs suppressed (ADR-0010 D10 mode): bathymetry_layer is the "
      "depth authority; tide correction inactive, no chart_datum TF required.");
  else if(!chart_datum_frame_.empty() && !sea_surface_frame_.empty())
    RCLCPP_INFO_STREAM(logger_, "Tide correction enabled: sea surface height in chart datum frame ("
      << sea_surface_frame_ << " expressed in " << chart_datum_frame_
      << "), invalidate threshold " << tide_invalidate_threshold_ << " m");

  declareParameter("s57_grids_namespace", rclcpp::ParameterValue(s57_grids_namespace_));
  node->get_parameter(name_+".s57_grids_namespace", s57_grids_namespace_);

  if(!s57_grids_namespace_.empty() && s57_grids_namespace_.back() != '/')
  {
    s57_grids_namespace_ += '/';
  }
  RCLCPP_INFO_STREAM(logger_, "Using S57 grids namespace: " << s57_grids_namespace_);
  RCLCPP_INFO_STREAM(logger_, "allow_uncharted: " << (allow_uncharted_ ? "true" : "false"));

  std::string service_name = s57_grids_namespace_ + "get_datasets";


  declareParameter("get_datasets_service", rclcpp::ParameterValue(service_name));
  node->get_parameter(name_+".get_datasets_service", service_name);
  RCLCPP_INFO_STREAM(logger_, "Using get datasets service: " << service_name);
  // rclcpp's free function, not the node's member: Nav2 on Rolling wraps the
  // member to return nav2::ServiceClient. The free function gives the same
  // rclcpp::Client, QoS and callback group as Jazzy's member.
  get_datasets_client_ = rclcpp::create_client<s57_msgs::srv::GetDatasets>(
    node->get_node_base_interface(), node->get_node_graph_interface(),
    node->get_node_services_interface(), service_name);


  global_frame_id_ = layered_costmap_->getGlobalFrameID();
  buffered_min_.header.frame_id = global_frame_id_;
  buffered_max_.header.frame_id = global_frame_id_;

  matchSize();

}

void S57Layer::reset()
{
  //pending_grids_.clear();
  //grids_.clear();
  //tiles_.clear();
  current_ = false;
  // Costmap2DROS::resetLayers resets the whole window before this, so the
  // next cycle reports all of it, including any tile not generated yet.
  // Defensive: the loop below flags every tile already generated, and every
  // tile in the window has been by the time a reset can come, so the flags
  // alone repaint the window and no test fails without this line
  // (ResetWhileMovingEveryCellEveryCycle passes). It covers a tile that
  // update_timeout left ungenerated.
  whole_pending_ = true;
  for(auto& t: tiles_)
    t.second.needs_update = true;
}

void S57Layer::matchSize()
{
  auto parent = layered_costmap_->getCostmap();

  x_origin_ = parent->getOriginX();
  y_origin_ = parent->getOriginY();
  resolution_ = parent->getResolution();
  tiles_.clear();
  current_ = false;
  // The window's size or resolution may have changed, so last_window_ no
  // longer describes it. Defensive as far as the tests go: their only
  // resizeMap comes before the first cycle, when whole_pending_ is true
  // already, and the cleared tiles are regenerated and reported anyway. A
  // resize while running (Costmap2DROS's width, height or resolution
  // parameters) reaches it; no test drives one.
  whole_pending_ = true;
}


S57Layer::TileID S57Layer::worldToTile(double x, double y)
{
  int ix = static_cast<int>(std::floor((x - x_origin_)/resolution_/tile_size_));
  int iy = static_cast<int>(std::floor((y - y_origin_)/resolution_/tile_size_));
  return std::make_pair(ix, iy);
}


void S57Layer::updateBounds(double, double, double, double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
  {
    // Defensive: the window moves on while this layer is disabled, so
    // last_window_ goes stale; repaint the whole window once it is enabled
    // again. This line runs every cycle on a layer configured disabled, but
    // its effect cannot be observed today: enabled_ is set only in
    // onInitialize and no parameter callback changes it, so nothing
    // re-enables the layer, and no test can see the repaint.
    whole_pending_ = true;
    return;
  }

  // Tide only matters for the depth ramp; in suppressed mode (D10) skip the
  // lookup entirely so a missing chart_datum TF never warns or invalidates.
  if(depth_costs_ && !chart_datum_frame_.empty() && !sea_surface_frame_.empty())
  {
    try
    {
      auto transform = tf_->lookupTransform(chart_datum_frame_, sea_surface_frame_, tf2::TimePointZero);
      double new_offset = transform.transform.translation.z;
      if(std::abs(new_offset - tide_offset_) > tide_invalidate_threshold_)
      {
        tide_offset_ = new_offset;
        RCLCPP_INFO_STREAM(logger_, "Tide offset updated: " << tide_offset_ << " m (water above chart datum)");
        // Mark tiles for regeneration but keep the cached costmap as a
        // fallback so the controller has a usable costmap during regen.
        // generateTile rebuilds the costmap from grids when chart_count=0,
        // and the empty-charts early-return clears the pointer for safety.
        for(auto& t: tiles_)
        {
          t.second.complete = false;
          t.second.chart_count = 0;
          t.second.needs_update = true;
        }
        current_ = false;
      }
    }
    catch(const tf2::TransformException& e)
    {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 10000,
        "Cannot look up tide offset (%s expressed in %s): %s",
        sea_surface_frame_.c_str(), chart_datum_frame_.c_str(), e.what());
    }
  }

  auto start_time = clock_->now();

  // Figure out the bounds of the parent costmap
  auto parent = layered_costmap_->getCostmap();

  double  world_min_x, world_min_y, world_max_x, world_max_y;
  world_min_x = parent->getOriginX();
  world_max_x = world_min_x + parent->getSizeInMetersX();
  world_min_y = parent->getOriginY();
  world_max_y = world_min_y + parent->getSizeInMetersY();

  // Buffer around the world bounds, controlled by buffer_fraction_
  // (default 0.05 = 5%). Larger values pre-load more chart area.
  double buffer = std::max(world_max_x-world_min_x, world_max_y-world_min_y)*buffer_fraction_;

  // use two layers of buffer. inner buffer to make sure data is available
  // and an outer buffer to limit the number of service calls to the get data service
  bool bounds_need_update = false;

  if( world_min_x - buffer < buffered_min_.point.x ||
      world_max_x + buffer > buffered_max_.point.x ||
      world_min_y - buffer < buffered_min_.point.y ||
      world_max_y + buffer > buffered_max_.point.y)
  {
    bounds_need_update = true;
  }

  if(pending_datasets_request_)
  {
    std::vector<int64_t> pruned_requests;
    // Prune all requests older than 5s.
    size_t n_pruned = get_datasets_client_->prune_requests_older_than(
      std::chrono::system_clock::now() - 5s,
      &pruned_requests
    );
    if (n_pruned)
    {
      RCLCPP_WARN_STREAM(
        logger_,
        "The get datasets server hasn't replied for more than 5s, " <<n_pruned << " requests were discarded, the discarded requests numbers are:");
      for (const auto & req_num : pruned_requests) {
        RCLCPP_WARN_STREAM(logger_, "\t" << req_num);
      }
      pending_datasets_request_ = false;
    }
  }

  if(bounds_need_update)
  {
    if(pending_datasets_request_)
    {
      RCLCPP_WARN_STREAM(logger_, "A request to the get datasets service is already pending and bounds already need an update. Not sending a new request.");

    }
    else
    {
      if(!get_datasets_client_->service_is_ready())
      {
        RCLCPP_WARN_STREAM(logger_, "The get datasets service is not ready, not sending a request.");
      }
      else
      {
        // update the buffered bounds adding both inner and outer buffer
        geometry_msgs::msg::PointStamped new_min, new_max;
        new_min.header.frame_id = global_frame_id_;
        new_max.header.frame_id = global_frame_id_;

        new_min.point.x = world_min_x - 2.0*buffer;
        new_min.point.y = world_min_y - 2.0*buffer;
        new_max.point.x = world_max_x + 2.0*buffer;
        new_max.point.y = world_max_y + 2.0*buffer;

        try
        {
          auto request = std::make_shared<s57_msgs::srv::GetDatasets::Request>();
          request->bounds.min_pt = worldToLatLon(new_min.point.x, new_min.point.y);
          request->bounds.max_pt = worldToLatLon(new_max.point.x, new_max.point.y);
          request->minimum_scale = 2*resolution_/0.0003125; // See S57Dataset::recommendedResolution for scale calculation

          get_datasets_client_->async_send_request(
            request,
            std::bind(&S57Layer::getDatasetsCallback, this, std::placeholders::_1)
          );
          pending_datasets_request_ = true;
          buffered_min_ = new_min;
          buffered_max_ = new_max;
        }
        catch(const std::exception& e)
        {
          RCLCPP_ERROR_STREAM(logger_, "Failed to send request to get datasets service: " << e.what());
        }
      }

    }
  }


  // The window by its cell edges. world_max_x and world_max_y above come
  // from getSizeInMetersX/Y, which is half a cell short on Jazzy; they only
  // choose the tiles and the chart request.
  const double half_cell = parent->getResolution()/2.0;
  Rect window;
  window.include(world_min_x, world_min_y,
    world_min_x + parent->getSizeInCellsX()*parent->getResolution(),
    world_min_y + parent->getSizeInCellsY()*parent->getResolution());

  // Phase 1, rolling window: record the columns and rows this cycle's move
  // uncovered, or find that the whole window must be reported.
  const bool rolling = layered_costmap_->isRolling();
  const bool report_whole = rolling && !recordUncovered(window, half_cell);

  // Phase 2: generate the tiles in the window, and collect the ones that
  // need repainting into this layer's own rectangle.
  TileID start_tile = worldToTile(world_min_x, world_min_y);
  TileID end_tile = worldToTile(world_max_x, world_max_y);

  Rect own;
  bool done = false;
  for(int i = start_tile.first; i <= end_tile.first && !done; i++)
    for(int j = start_tile.second; j <= end_tile.second && !done; j++)
    {
      auto id = std::make_pair(i,j);
      if(!tiles_[id].complete)
        generateTile(id);
      double tile_min_x = x_origin_+id.first*resolution_*tile_size_;
      double tile_max_x = tile_min_x + resolution_*tile_size_;
      double tile_min_y = y_origin_+id.second*resolution_*tile_size_;
      double tile_max_y = tile_min_y + resolution_*tile_size_;
      // A tile wholly outside last cycle's window has every cell it has in
      // this window in the spans phase 1 just recorded, so it is painted
      // with them. Reporting it as well would add a tile column or row to
      // this cycle's bounds, and with the next cycle's strip on the other
      // axis the whole window (see chooseStrip).
      if(rolling && !report_whole &&
         (tile_max_x <= last_window_.min_x + half_cell ||
          tile_min_x >= last_window_.max_x - half_cell ||
          tile_max_y <= last_window_.min_y + half_cell ||
          tile_min_y >= last_window_.max_y - half_cell))
        tiles_[id].needs_update = false;
      if(tiles_[id].needs_update)
        own.include(tile_min_x-resolution_, tile_min_y-resolution_,
          tile_max_x+resolution_, tile_max_y+resolution_);
      if(clock_->now() - start_time > rclcpp::Duration::from_seconds(update_timeout_))
        done = true;
    }

  // Phase 3, rolling window: add this cycle's strip, if any, to the
  // layer's own rectangle, and remember the window.
  if(rolling)
  {
    const Strip strip = report_whole ? Strip::whole : chooseStrip();
    if(strip == Strip::whole)
    {
      own.include(window.min_x, window.min_y, window.max_x, window.max_y);
      for(auto & span: pending_)
        span = Span();
    }
    else if(strip != Strip::none)
    {
      // A strip's edges are cell edges. Nav2's conversion (the minimum
      // truncated, the maximum truncated plus one) covers them whichever way
      // rounding goes; the half-cell margin keeps that from depending on it.
      // Defensive: no test fails without the margin, nor any harness run,
      // at 1 m or 0.1 m cells, because that conversion already covers them.
      const Span & span = pending_[strip];
      if(strip == Strip::x_low || strip == Strip::x_high)
        own.include(span.lo - half_cell, window.min_y, span.hi + half_cell, window.max_y);
      else
        own.include(window.min_x, span.lo - half_cell, window.max_x, span.hi + half_cell);
      pending_[strip] = Span();
    }
    last_reported_ = strip;
    last_window_ = window;
    whole_pending_ = false;
  }

  // Phase 4: clip this layer's own rectangle to the window, then widen the
  // bounds the layers before it reported. Never narrow them: they are what
  // those layers need repainted (Nav2's layer contract;
  // LayeredCostmap::updateMap warns "Illegal bounds change" when a layer
  // does).
  own.clip(window.min_x, window.min_y, window.max_x, window.max_y);
  if(!own.empty())
  {
    *min_x = std::min(*min_x, own.min_x);
    *min_y = std::min(*min_y, own.min_y);
    *max_x = std::max(*max_x, own.max_x);
    *max_y = std::max(*max_y, own.max_y);
  }
}

bool S57Layer::recordUncovered(const Rect & window, double half_cell)
{
  if(whole_pending_ ||
     window.min_x >= last_window_.max_x - half_cell || window.max_x <= last_window_.min_x + half_cell ||
     window.min_y >= last_window_.max_y - half_cell || window.max_y <= last_window_.min_y + half_cell)
    return false;

  // A jump: a move of more than a tile on either axis in one cycle, as when
  // localisation jumps. Its uncovered cells can reach the robot itself, and
  // as strips they would wait their turn (up to 3 cycles, see chooseStrip),
  // so the whole window is reported at once. Either axis, not both: a jump
  // along one axis would wait a cycle behind a strip on the other. Travel
  // never moves this far: BEN's global costmap moves about 5 cells per
  // cycle at 5 m/s and its local one 2, against 100-cell tiles. A move past
  // the window (the test above) is such a jump too, unless a tile is as wide
  // as the window; that test is kept for that case. On a window no wider
  // than a tile plus a cell (a 60-cell local costmap with the default
  // 100-cell tiles) the test above always fires first, so this rule never
  // decides there.
  const double tile = tile_size_*resolution_;
  if(std::abs(window.min_x - last_window_.min_x) > tile + half_cell ||
     std::abs(window.min_y - last_window_.min_y) > tile + half_cell)
    return false;

  // What has scrolled out of the window needs no repainting. A span that
  // has wholly scrolled out is no longer pending, so it holds back no other
  // strip (chooseStrip). Only a pending span ages: an empty one's age is
  // reset when cells are next added to it, and aging it every cycle would
  // overflow the int (undefined behaviour) after 2^31 cycles with nothing
  // pending on that side, about 13.6 years at 5 Hz.
  for(int s = Strip::x_low; s <= Strip::y_high; s++)
  {
    const bool x = s == Strip::x_low || s == Strip::x_high;
    pending_[s].clip(x ? window.min_x : window.min_y, x ? window.max_x : window.max_y, half_cell);
    if(pending_[s].valid)
      pending_[s].age++;
  }

  // A cell is uncovered when it lies outside last cycle's window on either
  // axis: in a column beyond one of its x edges, or a row beyond one of its
  // y edges.
  pending_[Strip::x_low].add(window.min_x, last_window_.min_x, half_cell);
  pending_[Strip::x_high].add(last_window_.max_x, window.max_x, half_cell);
  pending_[Strip::y_low].add(window.min_y, last_window_.min_y, half_cell);
  pending_[Strip::y_high].add(last_window_.max_y, window.max_y, half_cell);
  return true;
}

// Why two strips are never reported in consecutive cycles. The inflation
// layer reports the bounds it was given this cycle merged with last
// cycle's, padded by its radius (InflationLayer::updateBounds, the same on
// Jazzy and Rolling), and LayeredCostmap::updateMap resets and repaints
// that one rectangle. Any two different strips together span the window:
// an x strip (the window's full height) and a y strip (its full width), or
// an x strip at the west edge and one at the east, as when the window
// reverses or jitters while station keeping. That would repaint and
// re-inflate the whole window, which on BEN's 5 km global costmap took
// 0.3-0.4 s per cycle on a desktop (#57). So a strip follows last cycle's
// only when it is the same strip (steady travel along one axis) and no
// other is pending; otherwise this layer first reports no strip for a
// cycle (a gap), in which the inflation layer re-runs only last cycle's.
//
// After a gap, the strip whose oldest cell has waited longest goes next;
// the next chance for another is two cycles later. A cell waiting 2 cycles
// or more would then wait 4 or more, so if two strips have such cells the
// whole window is reported instead, which keeps the bound: every uncovered
// cell is painted at most 3 cycles after the cycle that uncovered it. On
// travel along one axis that is the same cycle, or the next after a single
// reversal (repeated reversals of 2 cells or more a cycle can make a cell
// wait 2); on steady diagonal travel the strips go x, gap, y, gap, and no
// whole window is reported at any heading. Two strips with cells waiting 2
// cycles happen when station keeping jitters widely enough to put the window
// on 3 or more origins per axis, on a sharp turn from oblique travel, and
// when travel reverses on both axes. The whole window is never reported
// while the window stays on 2 origins per axis, which holds when the
// position stays within one cell of a point. Wider jitter can report it
// (measured: uniform noise of 1.5 cells a cycle reported it in 6 to 9 of
// 600 cycles, 2 cells in about 2 to 3%) (#57). The whole window
// is also reported on the first cycle, after reset() or matchSize(), on a
// jump of more than a tile, and whenever tiles are flagged for every cell
// of it (a tide step over the threshold, a change of the chart set).
S57Layer::Strip S57Layer::chooseStrip() const
{
  bool other = false;
  for(int s = Strip::x_low; s <= Strip::y_high; s++)
    other = other || (pending_[s].valid && s != last_reported_);
  if(last_reported_ <= Strip::y_high)
  {
    if(other || !pending_[last_reported_].valid)
      return Strip::none;
    return last_reported_;
  }

  Strip oldest = Strip::none;
  int waiting = 0;
  for(int s = Strip::x_low; s <= Strip::y_high; s++)
  {
    if(!pending_[s].valid)
      continue;
    if(pending_[s].age >= 2)
      waiting++;
    if(oldest == Strip::none || pending_[s].age > pending_[oldest].age)
      oldest = static_cast<Strip>(s);
  }
  return waiting >= 2 ? Strip::whole : oldest;
}


void S57Layer::generateTile(TileID id)
{
  double world_min_x = x_origin_+id.first*resolution_*tile_size_;
  double world_max_x = world_min_x + resolution_*tile_size_;
  double world_min_y = y_origin_+id.second*resolution_*tile_size_;
  double world_max_y = world_min_y + resolution_*tile_size_;

  // No chart coverage for this area — handle as uncharted.
  if(current_charts_.empty())
  {
    tiles_[id].complete = true;
    // Clear any stale costmap from a prior era when this tile had
    // chart coverage. updateCosts treats nullptr+complete as uncharted
    // and handles it according to allow_uncharted_.
    tiles_[id].costmap = nullptr;
    // Report the tile, as the branch below does for a tile no chart covers:
    // with allow_uncharted false it must be painted unknown, and nothing
    // else would ask for it on a window that does not move.
    tiles_[id].needs_update = true;
    return;
  }

  // This map is used to sort the grids by resolution. It allows us to use the highest
  // resolution chart that is not higher than the parent costmap's resolution.
  std::map<std::pair<double, std::string>, std::shared_ptr<grid_map::GridMap> > grids;
  bool all_charts_available = true;
  for(auto& c: current_charts_)
  {
      std::string chart = c.label;
      if(chart_bounds_.count(chart) == 0)
      {
        RCLCPP_WARN_STREAM(logger_, "Chart " << chart << " not found in chart bounds, skipping.");
        continue;
      }
      auto bounds = chart_bounds_[chart];
      if(bounds.first.x > world_max_x || bounds.second.x < world_min_x ||
         bounds.first.y > world_max_y || bounds.second.y < world_min_y)
      {
        RCLCPP_DEBUG_STREAM(logger_, "Chart " << chart << " is not in the tile bounds, skipping.");
        continue;
      }
      auto grid = grids_[chart];
      if(grid)
      {
        grids[std::make_pair(grid->getResolution(), chart)] = grid;
      }
      else
        all_charts_available = false;
  }

  // No chart from current_charts_ overlaps this tile. Treat as uncharted —
  // same handling as the empty-current_charts_ early return — and clear
  // any costmap left over from a prior era when this tile had coverage.
  if(grids.empty() && all_charts_available)
  {
    tiles_[id].complete = true;
    tiles_[id].costmap = nullptr;
    tiles_[id].chart_count = 0;
    tiles_[id].needs_update = true;
    return;
  }

  tiles_[id].complete = all_charts_available;
  if(grids.size() > tiles_[id].chart_count)
  {
    auto tile = std::make_shared<nav2_costmap_2d::Costmap2D>(tile_size_, tile_size_, resolution_, world_min_x, world_min_y, nav2_costmap_2d::NO_INFORMATION);

    grid_map::Position tile_center(world_min_x + resolution_*tile_size_/2.0, world_min_y + resolution_*tile_size_/2.0);
    grid_map::Length tile_size(resolution_*tile_size_, resolution_*tile_size_);

    for(auto grid: grids)
    {
      auto grid_half_resolution = grid.second->getResolution()/2.0;

      // Ask grid_map for this chart's cells under the tile with the tile
      // clipped, per axis, to the grid's cell centres (rolker/s57_tools#59).
      // grid_map rejects a request whose centre is outside the grid (a chart
      // edge or a seam between the tile's centre and part of the tile, or a
      // chart smaller than a tile and off its centre). When it pulls a corner
      // on or beyond the grid's south-west edge back in, it adds an epsilon
      // that rounding loses in a band around the map's x = 0 and y = 0 lines
      // (wider for a longer chart), and rejects that request too. Either way
      // the chart was dropped from the whole tile. A request whose corners
      // lie within the grid's cell centres is never pulled in and its centre
      // is inside the grid, so it cannot be rejected. Its submap holds
      // every chart cell that overlaps the tile with positive area (up to
      // floating-point rounding of an edge), plus at most a row or column
      // that only touches the tile, which the loop below paints nothing from.
      // The grid's own geometry is used, not chart_bounds_: setGeometry
      // rounds the grid to whole cells. Where nothing is clipped, the request
      // is the tile's own centre and size, as before this change.
      const grid_map::Position grid_position = grid.second->getPosition();
      const grid_map::Length grid_length = grid.second->getLength();
      const double tile_lo[2] = {world_min_x, world_min_y};
      const double tile_hi[2] = {world_max_x, world_max_y};
      grid_map::Position request_center = tile_center;
      grid_map::Length request_length = tile_size;
      bool overlaps = true;
      for(int a = 0; a < 2; a++)
      {
        const double grid_lo = grid_position[a] - grid_length[a]/2.0;
        const double grid_hi = grid_position[a] + grid_length[a]/2.0;
        // No overlap with positive extent: nothing to paint.
        if(grid_lo >= tile_hi[a] || grid_hi <= tile_lo[a])
        {
          overlaps = false;
          break;
        }
        const double lo = std::clamp(tile_lo[a], grid_lo + grid_half_resolution, grid_hi - grid_half_resolution);
        const double hi = std::clamp(tile_hi[a], grid_lo + grid_half_resolution, grid_hi - grid_half_resolution);
        if(lo != tile_lo[a] || hi != tile_hi[a])
        {
          request_center[a] = lo + (hi - lo)/2.0;
          request_length[a] = hi - lo;
        }
      }
      if(!overlaps)
        continue;

      // By the argument above the request cannot fail; a failure is still
      // handled as before (the chart paints nothing in this tile).
      bool submap_ok = false;
      grid_map::SubmapGeometry overlap(*grid.second, request_center, request_length, submap_ok);
      if(submap_ok)
      {
        for(grid_map::SubmapIterator smi(overlap); !smi.isPastEnd(); ++smi)
        {
          grid_map::Index index = *smi;
          auto cost = get_cost_from_grid(*grid.second, index);
          if(cost == nav2_costmap_2d::NO_INFORMATION)
            continue;
          grid_map::Position cell_position;
          if(grid.second->getPosition(index, cell_position))
          {
            // A chart cell paints the tile cells it overlaps with positive
            // area: half-open ranges, floor for the near index and ceil - 1
            // for the far one, so a chart cell that only touches a tile
            // cell's edge, or lies outside the tile, paints nothing (near >
            // far). Each index is clamped on both sides in double before the
            // cast, so the cast is defined for any quotient: the near index
            // to [0, tile_size_] and the far one to [-1, tile_size_ - 1],
            // where tile_size_ and -1 occur only for a chart cell wholly
            // east/north or west/south of the tile and give an empty range.
            // A NaN quotient also gives an empty range, so it paints nothing:
            // the inner std::min/std::max compares false with NaN and returns
            // its first argument, the bound beyond the tile.
            // The rule holds in exact arithmetic. Where a chart cell edge
            // lies within floating-point rounding of a tile-cell edge, the
            // quotient lands a hair either side of the integer, so either
            // neighbour may be taken, as before this change. Charts placed
            // from latitude and longitude (S57Dataset::getGrid) align with
            // tile cells only by such an accident.
            const double near_x = std::floor((cell_position.x()-grid_half_resolution-world_min_x)/resolution_);
            const double near_y = std::floor((cell_position.y()-grid_half_resolution-world_min_y)/resolution_);
            const double far_x = std::ceil((cell_position.x()+grid_half_resolution-world_min_x)/resolution_) - 1.0;
            const double far_y = std::ceil((cell_position.y()+grid_half_resolution-world_min_y)/resolution_) - 1.0;
            const int tile_min_x = static_cast<int>(std::max(0.0, std::min(double(tile_size_), near_x)));
            const int tile_min_y = static_cast<int>(std::max(0.0, std::min(double(tile_size_), near_y)));
            const int tile_max_x = static_cast<int>(std::min(tile_size_ - 1.0, std::max(-1.0, far_x)));
            const int tile_max_y = static_cast<int>(std::min(tile_size_ - 1.0, std::max(-1.0, far_y)));
            for(int tile_y = tile_min_y; tile_y <= tile_max_y; tile_y++)
            {
              for(int tile_x = tile_min_x; tile_x <= tile_max_x; tile_x++)
              {
                if(tile->getCost(tile_x, tile_y) == nav2_costmap_2d::NO_INFORMATION)
                {
                  tile->setCost(tile_x, tile_y, cost);
                }
              }
            }
          }
        }
      }

    }
    tiles_[id].costmap = tile;
    tiles_[id].needs_update = true;
    tiles_[id].chart_count = grids.size();
  }
}

void S57Layer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  double world_min_x, world_min_y, world_max_x, world_max_y;
  master_grid.mapToWorld(min_i, min_j, world_min_x, world_min_y);
  master_grid.mapToWorld(max_i, max_j, world_max_x, world_max_y);

  TileID start_tile = worldToTile(world_min_x, world_min_y);
  TileID end_tile = worldToTile(world_max_x, world_max_y);

  bool complete = true;

  for(int ti = start_tile.first; ti <= end_tile.first; ti++)
  {
    int tile_offset_x = -ti*tile_size_ + static_cast<int>(std::lround((master_grid.getOriginX()-x_origin_)/resolution_));
    int start_i = std::max(min_i, -tile_offset_x);
    int i_count = std::min(max_i, tile_size_-tile_offset_x)-start_i;
    for(int tj = start_tile.second; tj <= end_tile.second; tj++)
    {
      int tile_offset_y = -tj*tile_size_ + static_cast<int>(std::lround((master_grid.getOriginY()-y_origin_)/resolution_));

      TileID tile = std::make_pair(ti,tj);
      auto current_tile = tiles_[tile].costmap;
      complete = complete && tiles_[tile].complete;
      if(current_tile)
      {
        // Chart data available — copy costs to master costmap.
        for(int j = std::max(min_j, -tile_offset_y); j < max_j && j+tile_offset_y < tile_size_; j++)
        {
          unsigned int target_index = master_grid.getIndex(start_i, j);
          unsigned int source_index = current_tile->getIndex(start_i+tile_offset_x, j+tile_offset_y);
          for(int i = 0; i < i_count; i++)
          {
            unsigned char cost = current_tile->getCharMap()[source_index+i];
            if(cost != nav2_costmap_2d::NO_INFORMATION)
              master_grid.getCharMap()[target_index+i]=cost;
          }
        }
      }
      else if(!allow_uncharted_ || !tiles_[tile].complete)
      {
        // Either uncharted navigation is disallowed, or chart data
        // exists but hasn't loaded yet — mark cells as unknown.
        for(int j = std::max(min_j, -tile_offset_y); j < max_j && j+tile_offset_y < tile_size_; j++)
        {
          unsigned int target_index = master_grid.getIndex(start_i, j);
          for(int i = 0; i < i_count; i++)
            master_grid.getCharMap()[target_index+i]=nav2_costmap_2d::NO_INFORMATION;
        }
      }
      // else: uncharted + allow_uncharted — leave master costmap untouched.
      tiles_[tile].needs_update = false;
    }
  }
  current_ = complete;
}

unsigned char S57Layer::get_cost_from_grid(grid_map::GridMap &grid, const grid_map::Index &index)
{
  if(!std::isnan(grid.at("restricted", index)))
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  auto overhead = grid.at("overhead", index);
  if(!std::isnan(overhead) && overhead < overhead_clearance_)
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  auto elevation = grid.at("elevation", index);
  if(!depth_costs_)
  {
    // ADR-0010 D10 suppressed mode: no depth ramp — bathymetry_layer owns
    // depth costs. Retained here: discrete charted hazards (UWTROC/WRECKS/
    // PIPSOL via the "hazard" channel — in "elevation" they are
    // indistinguishable from a DEPARE band, so without this they would
    // vanish wherever bathymetry_layer has no coverage), land, and the
    // unsurveyed/caution floor. Soundingless UWTROC/WRECKS are rasterized
    // as awash in "hazard" only; a PIPSOL without DRVAL1 is not rasterized
    // (see README safety caveat). The exists() guard tolerates grids from
    // an s57_grids build predating the channel.
    if(grid.exists("hazard") && !std::isnan(grid.at("hazard", index)))
      return nav2_costmap_2d::LETHAL_OBSTACLE;
    if(!std::isnan(elevation))
    {
      if(elevation > 0.0)  // shoreline / intertidal / built feature
        return nav2_costmap_2d::LETHAL_OBSTACLE;
      if(!std::isnan(grid.at("unsurveyed", index)) || !std::isnan(grid.at("caution", index)))
        return unsurveyed_cost_;
      // Known-navigable water. MUST return a real cost, not NO_INFORMATION.
      // generateTile composites overlapping charts finest-resolution-first and
      // writes only where the tile cell is still NO_INFORMATION (first writer
      // wins), which is what makes the finest chart authoritative. Returning
      // NO_INFORMATION here would leave a water cell UNCLAIMED by the fine
      // harbour chart, so the next-coarser chart in the same tile gets to write
      // it — and on an overview cell (e.g. 1:2,000,000 US2ATLPC) a harbour basin
      // is far below the resolvable feature size and falls inside the LNDARE
      // polygon, i.e. elevation > 0 => LETHAL. That painted whole navigable
      // basins lethal at Lewes, DE on 2026-08-05, including the cell the vehicle
      // was floating in. FREE_SPACE claims the cell without asserting any depth
      // opinion: bathymetry_layer runs after this layer and max-combines
      // (raise-only), so it remains the sole depth authority per ADR-0010 D10.
      return nav2_costmap_2d::FREE_SPACE;
    }
    // Elevation is NaN at this cell (the channel itself is read
    // unconditionally above — a chart without it never reaches here): the
    // chart genuinely says nothing about the cell; leave it unclaimed so a
    // coarser chart can supply data.
    return nav2_costmap_2d::NO_INFORMATION;
  }
  if(!std::isnan(elevation))
  {
    // Anything above chart datum is shoreline / intertidal / built feature
    // (LNDARE, COALNE, CAUSWY, HULKES, PONTON, SLCONS, … rasterized as
    // elevation = 1.0 in marine_charts/s57_dataset.cpp; see "no min depth
    // values, so lethal" comment there).  Without this short-circuit the
    // tide-offset math below would convert positive elevations into
    // "navigable depth" once tide_offset_ exceeds elevation, silently
    // turning every coastline navigable at any tide above ~1 m above MLLW.
    if (elevation > 0.0)
      return nav2_costmap_2d::LETHAL_OBSTACLE;
    auto depth = -elevation + tide_offset_;
    if (depth < minimum_depth_)
      return nav2_costmap_2d::LETHAL_OBSTACLE;
    unsigned char cost = nav2_costmap_2d::FREE_SPACE;
    if(depth < maximum_caution_depth_)
      cost = nav2_costmap_2d::MAX_NON_OBSTACLE*(1.0-((depth-minimum_depth_)/(maximum_caution_depth_-minimum_depth_)));
    if(!std::isnan(grid.at("unsurveyed", index)) || !std::isnan(grid.at("caution", index)))
      cost = std::max(cost, unsurveyed_cost_);
    return cost;
  }
  return nav2_costmap_2d::NO_INFORMATION;
}

geographic_msgs::msg::GeoPoint S57Layer::worldToLatLon(double x, double y)
{
  geometry_msgs::msg::PointStamped world;
  world.point.x = x;
  world.point.y = y;
  world.header.frame_id = global_frame_id_;
  geometry_msgs::msg::PointStamped ecef;
  tf_->transform(world, ecef, "earth");
  geodesy::ECEFPoint ecef_point(ecef.point);
  return geodesy::toMsg(ecef_point);
}

geometry_msgs::msg::Point S57Layer::llToWorld(const geographic_msgs::msg::GeoPoint& geo_point)
{
  geometry_msgs::msg::PointStamped ecef;
  ecef.header.frame_id = "earth";
  ecef.point = toGeometry(geodesy::ECEFPoint(geo_point));

  geometry_msgs::msg::PointStamped world;
  tf_->transform(ecef, world, global_frame_id_);
  return world.point;
}

void S57Layer::getDatasetsCallback(GetDatasetsClient::SharedFuture future)
{
  if(!future.valid())
  {
    RCLCPP_ERROR_STREAM(logger_, "Invalid future in getDatasetsCallback");
    return;
  }
  auto result = future.get();
  if(!result)
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to get datasets from service");
    pending_datasets_request_ = false;
    return;
  }

  if(result->datasets.empty())
  {
    RCLCPP_WARN_STREAM(logger_,
      "No chart coverage for current area (0 datasets from service). "
      "allow_uncharted is " << (allow_uncharted_ ? "true" : "false") << ".");
  }
  else
  {
    RCLCPP_DEBUG_STREAM(logger_, "Received " << result->datasets.size() << " datasets from service");
  }

  // Compare new dataset labels against current_charts_ before overwriting.
  // bounds_need_update fires every ~5% of window movement, so this callback
  // runs frequently while the boat moves; in steady-state survey areas the
  // label set rarely changes. If unchanged, skip the per-tile invalidation
  // at the end of this function so the cache survives.
  std::unordered_set<std::string> new_labels;
  for(const auto& d: result->datasets) new_labels.insert(d.label);
  std::unordered_set<std::string> old_labels;
  for(const auto& c: current_charts_) old_labels.insert(c.label);
  bool chart_list_unchanged = (new_labels == old_labels);

  current_charts_ = result->datasets;
  chart_bounds_.clear();

  for(const auto &dataset_info: result->datasets)
  {
    if (grid_subscriptions_[dataset_info.label] == nullptr)
    {
      rclcpp::QoS latched_qos(1);
      latched_qos.transient_local();
      latched_qos.keep_last(1);

      // rclcpp's free function, not the node's member: Nav2 on Rolling wraps
      // the member with a (topic, callback, qos) signature that Jazzy does not
      // have. It takes the node as an lvalue, hence the named pointer.
      auto node = node_.lock();
      auto grid_sub = rclcpp::create_subscription<grid_map_msgs::msg::GridMap>(
        node,
        s57_grids_namespace_+"datasets/"+dataset_info.topic,
        latched_qos,
        [this, label=dataset_info.label](const grid_map_msgs::msg::GridMap::SharedPtr msg)
        {
          this->mapGridCallback(label, msg);
        }
      );
      grid_subscriptions_[dataset_info.label] = grid_sub;
    }
    auto min_point = llToWorld(dataset_info.bounds.min_pt);
    auto max_point = llToWorld(dataset_info.bounds.max_pt);
    chart_bounds_[dataset_info.label] = std::make_pair(min_point, max_point);
  }

  pending_datasets_request_ = false;

  // prune outdated subscriptions
  std::vector<std::string> outdated_subscriptions;
  for(const auto & sub: grid_subscriptions_)
  {
    bool found = false;
    for(const auto & dataset_info: current_charts_)
    {
      if(sub.first == dataset_info.label)
      {
        found = true;
        break;
      }
    }
    if(!found)
      outdated_subscriptions.push_back(sub.first);
  }
  for(const auto & label: outdated_subscriptions)
  {
    grid_subscriptions_.erase(label);
  }

  // prune outdated grids
  std::vector<std::string> outdated_grids;
  for (const auto & grid: grids_)
  {
    bool found = false;
    for(const auto & dataset_info: current_charts_)
    {
      if(grid.first == dataset_info.label)
      {
        found = true;
        break;
      }
    }
    if(!found)
      outdated_grids.push_back(grid.first);
  }
  for (const auto & grid: outdated_grids)
  {
    grids_.erase(grid);
  }

  // Mark tiles as needing regeneration only when the chart set actually
  // changed — unchanged callbacks are common while the boat moves and
  // would otherwise force a full re-evaluation of every cached tile.
  // Also reset chart_count so the rebuild branch in generateTile fires
  // unconditionally; otherwise a same-count chart swap (one chart removed,
  // one added) would leave stale tile costmaps because grids.size() would
  // equal the previous chart_count.
  if(!chart_list_unchanged)
  {
    for(auto& t: tiles_)
    {
      t.second.complete = false;
      t.second.chart_count = 0;
      t.second.needs_update = true;
    }
  }

}

void S57Layer::mapGridCallback(std::string grid_name, std::shared_ptr<grid_map_msgs::msg::GridMap> grid_map)
{
  RCLCPP_INFO_STREAM(logger_, "Received grid map for grid: " << grid_name);
  auto grid = std::make_shared<grid_map::GridMap>();
  if(!grid_map::GridMapRosConverter::fromMessage(*grid_map, *grid))
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to convert grid map from message for grid: " << grid_name);
    return;
  }
  if(grid->getLayers().empty())
  {
    RCLCPP_ERROR_STREAM(logger_, "Grid map for grid " << grid_name << " has no layers, skipping.");
    return;
  }
  if(grid->getResolution() <= 0.0)
  {
    RCLCPP_ERROR_STREAM(logger_, "Grid map for grid " << grid_name << " has invalid resolution, skipping.");
    return;
  }
  if(grid->getLength().x() <= 0.0 || grid->getLength().y() <= 0.0)
  {
    RCLCPP_ERROR_STREAM(logger_, "Grid map for grid " << grid_name << " has invalid size, skipping.");
    return;
  }
  grids_[grid_name] = grid;
}

} // namespace s57_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(s57_layer::S57Layer, nav2_costmap_2d::Layer)

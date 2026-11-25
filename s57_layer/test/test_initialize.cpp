// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

// Runs S57Layer::initialize() on a real lifecycle node that carries
// parameter overrides, under the layer name "charts", which no other test
// uses, and reads each override back through what the layer does with it:
// the lethal decisions of get_cost_from_grid on a crafted grid, the
// uncharted handling of updateCosts, the tide offset of updateBounds, and
// the get_datasets client's name as the ROS graph lists it. A wrong name
// there means the layer never receives a chart.
//
// The last test serves get_datasets and a latched chart grid in-process, as
// s57_grids does, and follows a chart from the layer's request to the
// costmap.

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "s57_msgs/srv/get_datasets.hpp"
#include "tf2_ros/buffer.hpp"

#include "s57_layer.h"

// Nav2's lifecycle node moved from nav2_util (Jazzy) to nav2_ros_common
// (Rolling); Layer::initialize takes a weak pointer to whichever one exists.
#if __has_include("nav2_ros_common/lifecycle_node.hpp")
#include "nav2_ros_common/lifecycle_node.hpp"
using TestLifecycleNode = nav2::LifecycleNode;
#else
#include "nav2_util/lifecycle_node.hpp"
using TestLifecycleNode = nav2_util::LifecycleNode;
#endif

using namespace std::chrono_literals;

namespace
{

constexpr char kLayerName[] = "charts";

// Every parameter S57Layer::onInitialize declares.
const std::vector<std::string> kParameters = {
  "enabled", "minimum_depth", "maximum_caution_depth", "overhead_clearance",
  "unsurveyed_cost", "depth_costs", "update_timeout", "tile_size",
  "buffer_fraction", "allow_uncharted", "chart_datum_frame",
  "sea_surface_frame", "tide_invalidate_threshold", "s57_grids_namespace",
  "get_datasets_service"};

constexpr char kChartDatumFrame[] = "test_chart_datum";
constexpr char kSeaSurfaceFrame[] = "test_sea_surface";

std::string pid() {return std::to_string(getpid());}

std::string full(const std::string & param) {return std::string(kLayerName) + "." + param;}

// Each value differs from the member's starting value and from the default
// the layer declares (enabled: Nav2's Layer starts with enabled_ false and
// the layer declares true, so true here differs from the starting value;
// the defaults test and the enabled=false test cover the rest).
std::vector<rclcpp::Parameter> allOverrides(
  const std::string & service, bool enabled = true)
{
  return {
    rclcpp::Parameter(full("enabled"), enabled),
    rclcpp::Parameter(full("minimum_depth"), 2.5),            // default 0
    rclcpp::Parameter(full("maximum_caution_depth"), 7.5),    // default 5
    rclcpp::Parameter(full("overhead_clearance"), 20.0),      // default 10
    rclcpp::Parameter(full("unsurveyed_cost"), 77),           // default 100
    rclcpp::Parameter(full("depth_costs"), false),            // default true
    rclcpp::Parameter(full("update_timeout"), 0.25),          // default 0.5
    rclcpp::Parameter(full("tile_size"), 7),                  // default 100
    rclcpp::Parameter(full("buffer_fraction"), 0.2),          // default 0.05
    rclcpp::Parameter(full("allow_uncharted"), false),        // default true
    rclcpp::Parameter(full("chart_datum_frame"), std::string(kChartDatumFrame)),  // ""
    rclcpp::Parameter(full("sea_surface_frame"), std::string(kSeaSurfaceFrame)),  // map_tide
    rclcpp::Parameter(full("tide_invalidate_threshold"), 0.3),  // default 0.01
    rclcpp::Parameter(full("s57_grids_namespace"), "unused_ns_" + pid()),  // ""
    rclcpp::Parameter(full("get_datasets_service"), service),  // <ns>get_datasets
  };
}

/// Exposes what the tests read back.
class InitTestLayer : public s57_layer::S57Layer
{
public:
  using S57Layer::depth_costs_;
  using S57Layer::tide_offset_;

  unsigned char cost(grid_map::GridMap & grid)
  {
    return get_cost_from_grid(grid, grid_map::Index(0, 0));
  }
};

/// A 1x1 grid with every channel the layer reads, all NaN.
grid_map::GridMap makeCell()
{
  grid_map::GridMap grid({"elevation", "overhead", "restricted",
      "unsurveyed", "caution", "hazard"});
  grid.setGeometry(grid_map::Length(1.0, 1.0), 1.0, grid_map::Position(0.0, 0.0));
  for (const auto & name : grid.getLayers()) {
    grid[name].setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  return grid;
}

/// Polls pred until it holds or the timeout passes; returns its last value.
bool waitFor(const std::function<bool()> & pred, std::chrono::milliseconds timeout = 10s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!pred()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return pred();
    }
    std::this_thread::sleep_for(20ms);
  }
  return true;
}

class InitializeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    layered_costmap_.resizeMap(10, 10, 1.0, 0.0, 0.0);
    tf_ = std::make_unique<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>());
  }

  /// A lifecycle node carrying the given overrides. The test's name and the
  /// process id keep the node's name unique in a shared ROS domain.
  static std::shared_ptr<TestLifecycleNode> makeNode(
    const std::vector<rclcpp::Parameter> & overrides)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    const std::string test =
      ::testing::UnitTest::GetInstance()->current_test_info()->name();
    return std::make_shared<TestLifecycleNode>(
      "s57_layer_init_" + test + "_" + pid(), "", options);
  }

  /// Calls initialize() as Nav2's costmap does, with no callback group.
  void initialize(InitTestLayer & layer, const std::shared_ptr<TestLifecycleNode> & node)
  {
    layer.initialize(&layered_costmap_, kLayerName, tf_.get(), node, nullptr);
  }

  /// Sea surface height above chart datum, as a static transform between
  /// the two overridden frames only.
  void setTide(double z)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = kChartDatumFrame;
    t.child_frame_id = kSeaSurfaceFrame;
    t.transform.translation.z = z;
    t.transform.rotation.w = 1.0;
    tf_->setTransform(t, "test_initialize", true);
  }

  void updateBounds(InitTestLayer & layer)
  {
    double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
    layer.updateBounds(5.0, 5.0, 0.0, &minx, &miny, &maxx, &maxy);
  }

  /// The GetDatasets clients the graph lists on this node, by service name.
  /// Empty until the graph knows the node (discovery is asynchronous, even
  /// within one process).
  static std::vector<std::string> getDatasetsClients(TestLifecycleNode & node)
  {
    std::vector<std::string> names;
    std::map<std::string, std::vector<std::string>> clients;
    try {
      clients = node.get_node_graph_interface()->get_client_names_and_types_by_node(
        node.get_name(), node.get_namespace());
    } catch (const std::exception &) {
      return names;
    }
    for (const auto & client : clients) {
      for (const auto & type : client.second) {
        if (type == "s57_msgs/srv/GetDatasets") {
          names.push_back(client.first);
        }
      }
    }
    return names;
  }

  nav2_costmap_2d::LayeredCostmap layered_costmap_{"map", false, false};
  std::unique_ptr<tf2_ros::Buffer> tf_;
};

}  // namespace

// Every parameter is declared under "charts.", and none under the name the
// other tests use or with no prefix. Nothing is declared beforehand.
TEST_F(InitializeTest, DeclaresEveryParameterUnderTheLayersName)
{
  auto node = makeNode(allOverrides("/s57_layer_test_" + pid() + "/svc"));
  for (const auto & p : kParameters) {
    ASSERT_FALSE(node->has_parameter(full(p))) << p << " declared before initialize()";
  }
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));

  ASSERT_EQ(15u, kParameters.size());
  for (const auto & p : kParameters) {
    EXPECT_TRUE(node->has_parameter(full(p))) << full(p);
    EXPECT_FALSE(node->has_parameter("chart_layer." + p)) << "chart_layer." << p;
    EXPECT_FALSE(node->has_parameter(p)) << p;
  }
  // The declared values are the overrides.
  EXPECT_DOUBLE_EQ(2.5, node->get_parameter(full("minimum_depth")).as_double());
  EXPECT_EQ(77, node->get_parameter(full("unsurveyed_cost")).as_int());
  EXPECT_EQ(7, node->get_parameter(full("tile_size")).as_int());
}

// The overrides that decide what is lethal, read back through
// get_cost_from_grid on a crafted cell.
TEST_F(InitializeTest, ReadsLethalDecisionOverrides)
{
  auto node = makeNode(allOverrides("/s57_layer_test_" + pid() + "/svc"));
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));

  EXPECT_TRUE(layer.isEnabled());

  // depth_costs = false: the depth ramp is suppressed. A cell 0.5 m deep,
  // shallower than minimum_depth, is not lethal; a charted hazard is.
  EXPECT_FALSE(layer.depth_costs_);
  {
    auto grid = makeCell();
    grid.at("elevation", grid_map::Index(0, 0)) = -0.5;
    EXPECT_EQ(nav2_costmap_2d::FREE_SPACE, layer.cost(grid)) << "depth ignored";
    grid.at("hazard", grid_map::Index(0, 0)) = 0.0;
    EXPECT_EQ(nav2_costmap_2d::LETHAL_OBSTACLE, layer.cost(grid)) << "hazard";
  }

  // overhead_clearance = 20: a 15 m clearance is lethal (default 10: not).
  {
    auto grid = makeCell();
    grid.at("elevation", grid_map::Index(0, 0)) = -20.0;
    grid.at("overhead", grid_map::Index(0, 0)) = 15.0;
    EXPECT_EQ(nav2_costmap_2d::LETHAL_OBSTACLE, layer.cost(grid)) << "overhead 15 m";
  }

  // unsurveyed_cost = 77 (default 100), in deep water.
  {
    auto grid = makeCell();
    grid.at("elevation", grid_map::Index(0, 0)) = -20.0;
    grid.at("unsurveyed", grid_map::Index(0, 0)) = 0.0;
    EXPECT_EQ(77, layer.cost(grid)) << "unsurveyed, suppressed mode";
    layer.depth_costs_ = true;
    EXPECT_EQ(77, layer.cost(grid)) << "unsurveyed, depth mode";
  }

  // The depth ramp itself, with depth_costs switched on by hand (the
  // override switched it off, which is checked above).
  layer.depth_costs_ = true;
  auto grid = makeCell();
  auto & elevation = grid.at("elevation", grid_map::Index(0, 0));

  // minimum_depth = 2.5 (default 0): 2.4 m is lethal, 2.6 m is not.
  elevation = -2.4;
  EXPECT_EQ(nav2_costmap_2d::LETHAL_OBSTACLE, layer.cost(grid)) << "2.4 m";
  elevation = -2.6;
  EXPECT_LT(layer.cost(grid), nav2_costmap_2d::LETHAL_OBSTACLE) << "2.6 m";

  // maximum_caution_depth = 7.5 (default 5): the ramp still raises the cost
  // at 6 m, and stops below 7.5 m.
  elevation = -6.0;
  EXPECT_GT(layer.cost(grid), nav2_costmap_2d::FREE_SPACE) << "6 m";
  EXPECT_LT(layer.cost(grid), nav2_costmap_2d::LETHAL_OBSTACLE) << "6 m";
  elevation = -8.0;
  EXPECT_EQ(nav2_costmap_2d::FREE_SPACE, layer.cost(grid)) << "8 m";
}

// enabled = false is read, not only the default.
TEST_F(InitializeTest, ReadsEnabledFalse)
{
  auto node = makeNode(allOverrides("/s57_layer_test_" + pid() + "/svc", false));
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));
  EXPECT_FALSE(layer.isEnabled());
}

// allow_uncharted = false: with no chart, updateCosts writes NO_INFORMATION.
// (No server answers the overridden service, so no chart arrives.)
TEST_F(InitializeTest, ReadsAllowUnchartedOverride)
{
  auto node = makeNode(allOverrides("/s57_layer_test_" + pid() + "/svc"));
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));

  auto * master = layered_costmap_.getCostmap();
  updateBounds(layer);
  layer.updateCosts(*master, 0, 0, 10, 10);

  int no_information = 0;
  for (unsigned int i = 0; i < 100; ++i) {
    no_information += master->getCharMap()[i] == nav2_costmap_2d::NO_INFORMATION;
  }
  EXPECT_EQ(100, no_information);
}

// chart_datum_frame, sea_surface_frame and tide_invalidate_threshold: the
// only transform is between the two overridden frames, so the default
// frames ("" and "map_tide") find no tide; a 0.2 m change stays below the
// 0.3 m threshold (default 0.01 m) and a 0.4 m change does not.
TEST_F(InitializeTest, ReadsTideOverrides)
{
  auto node = makeNode(allOverrides("/s57_layer_test_" + pid() + "/svc"));
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));
  // The tide is read only in depth mode, which the override switched off.
  layer.depth_costs_ = true;

  setTide(1.5);
  updateBounds(layer);
  EXPECT_DOUBLE_EQ(1.5, layer.tide_offset_);

  setTide(1.7);
  updateBounds(layer);
  EXPECT_DOUBLE_EQ(1.5, layer.tide_offset_) << "0.2 m is below the 0.3 m threshold";

  setTide(1.9);
  updateBounds(layer);
  EXPECT_DOUBLE_EQ(1.9, layer.tide_offset_) << "0.4 m is above the 0.3 m threshold";
}

// get_datasets_service: the layer's own node has exactly one GetDatasets
// client, on exactly the overridden name.
TEST_F(InitializeTest, ClientUsesTheOverriddenServiceName)
{
  const std::string service = "/s57_layer_test_" + pid() + "/svc";
  auto node = makeNode(allOverrides(service));
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));

  ASSERT_TRUE(waitFor([&] {return !getDatasetsClients(*node).empty();}))
    << "no GetDatasets client on the layer's node";
  EXPECT_EQ(std::vector<std::string>{service}, getDatasetsClients(*node));
}

// s57_grids_namespace, without get_datasets_service: the client is
// <namespace>/get_datasets.
TEST_F(InitializeTest, ClientUsesTheOverriddenNamespace)
{
  const std::string ns = "s57_layer_test_" + pid();
  auto node = makeNode({rclcpp::Parameter(full("s57_grids_namespace"), ns)});
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));

  EXPECT_EQ(ns + "/get_datasets",
    node->get_parameter(full("get_datasets_service")).as_string());
  ASSERT_TRUE(waitFor([&] {return !getDatasetsClients(*node).empty();}))
    << "no GetDatasets client on the layer's node";
  EXPECT_EQ(std::vector<std::string>{"/" + ns + "/get_datasets"}, getDatasetsClients(*node));
}

// Without overrides the layer is enabled (Nav2's Layer starts disabled, so
// this proves the declared default is read) and keeps its defaults.
TEST_F(InitializeTest, DefaultsEnableTheLayer)
{
  auto node = makeNode({});
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));

  EXPECT_TRUE(layer.isEnabled());
  EXPECT_TRUE(node->get_parameter(full("enabled")).as_bool());
  EXPECT_TRUE(layer.depth_costs_);
  auto grid = makeCell();
  grid.at("elevation", grid_map::Index(0, 0)) = -20.0;
  grid.at("unsurveyed", grid_map::Index(0, 0)) = 0.0;
  EXPECT_EQ(100, layer.cost(grid));
}

namespace
{

constexpr char kChartLabel[] = "TEST0001.000";
constexpr char kChartTopic[] = "TEST0001_000";

/// A chart grid at 1 m covering [-4.75, 105.25] x [-4.75, 105.25] in "map",
/// with every channel the layer reads. In the 20 m by 20 m costmap the
/// north-east quadrant is land (elevation 1.0, as s57_grids writes LNDARE);
/// the rest is water 10 m deep. Two choices keep this test about the
/// request path, not about generateTile's compositing, which it leaves as is:
/// - the grid reaches past the layer's first tile ([0, 100], 100 cells of
///   1 m) on every side, so the test does not depend on how generateTile
///   handles a chart edge inside a tile (test_tile_edges covers that);
/// - no grid cell edge falls exactly on a tile-cell edge (the quarter-cell
///   offset), where which chart cell a straddled tile cell takes would
///   depend on rounding; test_tile_edges covers exact edges.
grid_map::GridMap makeChartGrid()
{
  grid_map::GridMap grid({"elevation", "overhead", "restricted",
      "unsurveyed", "caution", "hazard"});
  grid.setFrameId("map");
  grid.setGeometry(grid_map::Length(110.0, 110.0), 1.0, grid_map::Position(50.25, 50.25));
  for (const auto & name : grid.getLayers()) {
    grid[name].setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    grid.getPosition(*it, position);
    grid.at("elevation", *it) = (position.x() > 10.0 && position.y() > 10.0) ? 1.0 : -10.0;
  }
  return grid;
}

/// Stands in for s57_grids: serves get_datasets, answering with one chart
/// whose bounds are the request's, and publishes that chart's grid latched
/// on datasets/<topic>, both under the given namespace.
class FakeS57Grids
{
public:
  explicit FakeS57Grids(const std::string & ns)
  : node_(std::make_shared<rclcpp::Node>("fake_s57_grids_" + pid(), ns))
  {
    service_ = node_->create_service<s57_msgs::srv::GetDatasets>(
      "get_datasets",
      [this](const std::shared_ptr<s57_msgs::srv::GetDatasets::Request> request,
      std::shared_ptr<s57_msgs::srv::GetDatasets::Response> response)
      {
        ++requests_;
        s57_msgs::msg::DatasetInfo chart;
        chart.label = kChartLabel;
        chart.topic = kChartTopic;
        chart.bounds = request->bounds;
        chart.scale = 10000.0;
        chart.resolution = 1.0;
        response->datasets.push_back(chart);
      });

    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local();
    latched_qos.keep_last(1);
    publisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(
      std::string("datasets/") + kChartTopic, latched_qos);
    publisher_->publish(*grid_map::GridMapRosConverter::toMessage(makeChartGrid()));
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<s57_msgs::srv::GetDatasets>::SharedPtr service_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  std::atomic<int> requests_{0};
};

}  // namespace

// initialize()'s client reaches get_datasets, the reply's chart is
// subscribed to on the layer's own node, and the grid reaches the costmap:
// land lethal, water free. The costmap is static with its origin at the
// grid's south-west corner (rolker/s57_tools#56 is not exercised here).
TEST_F(InitializeTest, ChartReachesTheCostmap)
{
  const std::string ns = "/s57_layer_test_" + pid();
  FakeS57Grids s57_grids(ns);

  // An east-north-up "map" at latitude 0, longitude 0: map x, y and z are
  // ECEF y, z and x, so worldToLatLon and llToWorld work.
  geometry_msgs::msg::TransformStamped earth_to_map;
  earth_to_map.header.frame_id = "earth";
  earth_to_map.child_frame_id = "map";
  earth_to_map.transform.translation.x = 6378137.0;
  earth_to_map.transform.rotation.w = 0.5;
  earth_to_map.transform.rotation.x = 0.5;
  earth_to_map.transform.rotation.y = 0.5;
  earth_to_map.transform.rotation.z = 0.5;
  tf_->setTransform(earth_to_map, "test_initialize", true);

  layered_costmap_.resizeMap(20, 20, 1.0, 0.0, 0.0);
  auto * master = layered_costmap_.getCostmap();

  auto node = makeNode({rclcpp::Parameter(full("s57_grids_namespace"), ns)});
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(s57_grids.node_);
  executor.add_node(node->get_node_base_interface());

  // One costmap cycle, then the executor's pending work.
  auto cycle = [&] {
      updateBounds(layer);
      layer.updateCosts(*master, 0, 0, 20, 20);
      executor.spin_some(20ms);
    };
  auto cycleUntil = [&](const std::function<bool()> & pred) {
      return waitFor([&] {cycle(); return pred();});
    };

  ASSERT_TRUE(cycleUntil([&] {return s57_grids.requests_ > 0;}))
    << "no request reached " << ns << "/get_datasets";

  const std::string topic = ns + "/datasets/" + kChartTopic;
  auto ownGridSubscriptions = [&] {
      int count = 0;
      for (const auto & info : node->get_subscriptions_info_by_topic(topic)) {
        if (info.node_name() == node->get_name() &&
          info.node_namespace() == node->get_namespace() &&
          info.topic_type() == "grid_map_msgs/msg/GridMap")
        {
          ++count;
        }
      }
      return count;
    };
  ASSERT_TRUE(cycleUntil([&] {return ownGridSubscriptions() > 0;}))
    << "no GridMap subscription on " << topic << " from the layer's node";
  EXPECT_EQ(1, ownGridSubscriptions());

  // The interior of the land quadrant (cells 11 to 19; the cells at the
  // quadrant's edge are shared with water by generateTile's compositing).
  auto landInteriorLethal = [&] {
      for (unsigned int j = 11; j < 20; ++j) {
        for (unsigned int i = 11; i < 20; ++i) {
          if (master->getCost(i, j) != nav2_costmap_2d::LETHAL_OBSTACLE) {
            return false;
          }
        }
      }
      return true;
    };
  ASSERT_TRUE(cycleUntil(landInteriorLethal)) << "the chart's land never reached the costmap";

  // Water, away from the land quadrant's edge, is free.
  int water = 0;
  for (unsigned int j = 0; j < 20; ++j) {
    for (unsigned int i = 0; i < 20; ++i) {
      if (i <= 8 || j <= 8) {
        EXPECT_EQ(nav2_costmap_2d::FREE_SPACE, master->getCost(i, j)) << i << ", " << j;
        ++water;
      }
    }
  }
  EXPECT_EQ(400 - 121, water);
  EXPECT_EQ(1, s57_grids.requests_.load()) << "a static window needs one request";
  EXPECT_TRUE(layer.isCurrent());
}

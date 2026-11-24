// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

// GridPublisher across a cleanup and a new configuration, and with a chart
// that cannot be opened.
//
// on_cleanup sets abort_flag_ to stop grid jobs in flight. It used never to
// be cleared, so after a cleanup and a new configure every grid job returned
// no grid, and checkForNewGrids dereferenced it: the node aborted. A chart
// whose cell cannot be opened returned no grid the same way. These tests pin
// the reset, the dropping of aborted jobs, and the missing-grid check, with
// the catalogue fixture of marine_charts: NOAA's CATALOG.031 without its
// cell, so the one chart it lists cannot be opened.

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "s57_msgs/srv/get_datasets.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

#include "grid_publisher.h"

using namespace std::chrono_literals;

namespace
{

// The one chart CATALOG.031 lists; its cell file is not in the fixture.
constexpr char kChart[] = "US5PSMCC.000";

std::string pid() {return std::to_string(getpid());}

/// Exposes what the tests check.
class TestGridPublisher : public s57_grids::GridPublisher
{
public:
  using GridPublisher::GridPublisher;
  using GridPublisher::abort_flag_;
  using GridPublisher::pending_dataset_grids_;
  using GridPublisher::tf_buffer_;
};

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

class ReconfigureTest : public ::testing::Test
{
protected:
  // Every node of this process goes into a namespace of its own, so the
  // node's get_datasets is not confused with another s57_grids in the domain.
  static void SetUpTestSuite()
  {
    const std::string ns = "__ns:=/s57_grids_test_" + pid();
    const char * argv[] = {"test_reconfigure", "--ros-args", "-r", ns.c_str()};
    rclcpp::init(4, argv);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}

  /// A GridPublisher reading the catalogue fixture.
  static std::shared_ptr<TestGridPublisher> makeNode()
  {
    const std::string test =
      ::testing::UnitTest::GetInstance()->current_test_info()->name();
    auto node = std::make_shared<TestGridPublisher>("s57_grids_" + test + "_" + pid());
    node->declare_parameter("enc_root", std::string(TEST_ENC_ROOT));
    return node;
  }
};

}  // namespace

// A new configuration after a cleanup starts with grid jobs enabled.
TEST_F(ReconfigureTest, ConfigureAfterCleanupClearsTheAbortFlag)
{
  auto node = makeNode();
  ASSERT_EQ("inactive", node->configure().label());
  EXPECT_FALSE(node->abort_flag_);
  ASSERT_EQ("unconfigured", node->cleanup().label());
  EXPECT_TRUE(node->abort_flag_) << "cleanup aborts the jobs in flight";
  ASSERT_EQ("inactive", node->configure().label());
  EXPECT_FALSE(node->abort_flag_);
}

// A job the cleanup aborted is not kept for the next configuration. (No job
// can be in flight here without a chart cell, so an aborted one is made by
// hand: a finished future holding no grid, as getGrid returns.)
TEST_F(ReconfigureTest, CleanupDropsAbortedJobs)
{
  auto node = makeNode();
  ASSERT_EQ("inactive", node->configure().label());
  std::promise<std::shared_ptr<grid_map::GridMap>> aborted;
  aborted.set_value(nullptr);
  node->pending_dataset_grids_[kChart] = aborted.get_future();
  ASSERT_EQ("unconfigured", node->cleanup().label());
  EXPECT_TRUE(node->pending_dataset_grids_.empty());
}

// A chart the catalogue lists but whose cell cannot be opened gives no grid.
// The node logs it and carries on instead of aborting (this test process is
// the node's process), and publishes nothing for it.
TEST_F(ReconfigureTest, ChartThatCannotBeOpenedDoesNotAbortTheNode)
{
  auto node = makeNode();
  auto helper = std::make_shared<rclcpp::Node>("s57_grids_test_helper_" + pid());

  // The grid job's context looks up map -> earth when the job starts.
  tf2_ros::StaticTransformBroadcaster broadcaster(*helper);
  geometry_msgs::msg::TransformStamped earth_to_map;
  earth_to_map.header.frame_id = "earth";
  earth_to_map.child_frame_id = "map";
  earth_to_map.transform.rotation.w = 1.0;
  broadcaster.sendTransform(earth_to_map);

  // The node's errors, as /rosout carries them.
  std::vector<std::string> errors;
  const std::string logger = node->get_logger().get_name();
  auto rosout = helper->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", rclcpp::RosoutQoS(),
    [&errors, logger](const rcl_interfaces::msg::Log & log) {
      if (log.name == logger && log.level == rcl_interfaces::msg::Log::ERROR) {
        errors.push_back(log.msg);
      }
    });

  ASSERT_EQ("inactive", node->configure().label());
  ASSERT_EQ("active", node->activate().label());
  ASSERT_TRUE(waitFor([&] {return node->tf_buffer_->canTransform("map", "earth", tf2::TimePointZero);}))
    << "map -> earth never reached the node";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(helper);

  auto client = helper->create_client<s57_msgs::srv::GetDatasets>("get_datasets");
  ASSERT_TRUE(client->wait_for_service(10s));
  auto request = std::make_shared<s57_msgs::srv::GetDatasets::Request>();
  request->bounds.min_pt.latitude = 43.07;
  request->bounds.min_pt.longitude = -70.76;
  request->bounds.max_pt.latitude = 43.09;
  request->bounds.max_pt.longitude = -70.74;
  // The cell cannot be opened, so its scale reads 0; a negative minimum
  // scale lists it anyway. (The precompute path lists charts by position
  // alone, and reaches the same grid job.)
  request->minimum_scale = -1.0;
  auto future = client->async_send_request(request);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, executor.spin_until_future_complete(future, 10s));
  auto response = future.get();
  ASSERT_EQ(1u, response->datasets.size()) << "the fixture's catalogue lists one chart";
  ASSERT_EQ(kChart, response->datasets[0].label);

  ASSERT_TRUE(waitFor([&] {executor.spin_some(20ms); return !errors.empty();}))
    << "the node never reported the missing grid";
  EXPECT_NE(std::string::npos, errors[0].find(kChart)) << errors[0];
  EXPECT_TRUE(node->pending_dataset_grids_.empty());
  EXPECT_EQ(0u, helper->count_publishers(
      std::string(helper->get_namespace()) + "/datasets/" + response->datasets[0].topic));
}

#include "grid_publisher.h"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto gp = std::make_shared<s57_grids::GridPublisher>("grid_publisher");

  exe.add_node(gp->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();

  return 0;
}

#include "grid_publisher.h"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto gp = std::make_shared<s57_grids::GridPublisher>();
  rclcpp::spin(gp);

  return 0;
}

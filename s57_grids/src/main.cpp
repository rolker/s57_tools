#include "s57_grids/grid_publisher.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "s57_grids");
  ros::NodeHandle n;

  s57_grids::GridPublisher gp;

  ros::spin();

  return 0;
}

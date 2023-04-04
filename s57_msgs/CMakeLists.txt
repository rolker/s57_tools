cmake_minimum_required(VERSION 2.8.3)
project(s57_msgs)
find_package(catkin REQUIRED roscpp std_msgs message_generation geographic_msgs)

add_message_files(
  FILES
  DatasetInfo.msg
)

add_service_files(
  FILES
  GetDatasets.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geographic_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime geographic_msgs)

cmake_minimum_required(VERSION 3.8)
project(s57_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geographic_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DatasetInfo.msg"
  "srv/GetDatasets.srv"
  DEPENDENCIES geographic_msgs
)

ament_export_dependencies(rosidl_default_runtime geographic_msgs)

ament_package()

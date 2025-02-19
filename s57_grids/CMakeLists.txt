cmake_minimum_required(VERSION 3.14.4)
project(s57_grids)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(marine_charts REQUIRED)
find_package(rclcpp REQUIRED)
find_package(s57_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# add_executable(${PROJECT_NAME}
#     src/main.cpp
#     src/grid_publisher.cpp
# )
# target_include_directories(${PROJECT_NAME}
#   PUBLIC
#   ${grid_map_ros_INCLUDE_DIRS}
# )

# target_link_libraries(${PROJECT_NAME} PRIVATE
#     ${grid_map_ros_LIBRARIES}
#     marine_charts::marine_charts
#     rclcpp::rclcpp
#     ${s57_msgs_TARGETS}
#     ${tf2_geometry_msgs_TARGETS}
#     tf2_ros::tf2_ros
# )

# install (TARGETS ${PROJECT_NAME}
#     DESTINATION lib/${PROJECT_NAME}
# )

ament_package()

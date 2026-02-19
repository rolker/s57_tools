cmake_minimum_required(VERSION 3.14.4)
project(s57_grids)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(lifecycle_msgs)
find_package(marine_charts REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(s57_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# s57_grids_node

add_executable(${PROJECT_NAME}_node
    src/main.cpp
    src/grid_publisher.cpp
)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
  ${grid_map_ros_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node PRIVATE
    ${grid_map_ros_LIBRARIES}
    ${lifecycle_msgs_TARGETS}
    marine_charts::marine_charts
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${s57_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    tf2_ros::tf2_ros
)

install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

# s57_grids client library

add_library(${PROJECT_NAME} SHARED
    src/s57_grids_catalog.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${grid_map_ros_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${geographic_msgs_TARGETS}
    ${grid_map_ros_LIBRARIES}
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${s57_msgs_TARGETS}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geographic_msgs
  grid_map_ros
  rclcpp
  rclcpp_lifecycle
  s57_msgs
)

# package stuff

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

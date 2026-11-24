cmake_minimum_required(VERSION 3.14.4)
project(s57_layer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geodesy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(s57_grids REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(s57_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/s57_layer.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# geodesy and grid_map_ros export no CMake target, only include directories
# and libraries, so use their variables. geodesy's headers are included only
# from src/s57_layer.cpp, hence PRIVATE; grid_map's are included from
# src/s57_layer.h, hence PUBLIC, as s57_grids links grid_map_ros.
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${grid_map_ros_INCLUDE_DIRS}
  PRIVATE
    ${geodesy_INCLUDE_DIRS}
)

# PUBLIC: every library this target linked before, plus grid_map_ros, so the
# exported interface does not change. PRIVATE: geodesy's static geoconv
# library, linked into the plugin and kept out of the exported link interface.
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${geometry_msgs_TARGETS}
    ${grid_map_ros_LIBRARIES}
    s57_grids::s57_grids
    nav2_costmap_2d::nav2_costmap_2d_core
    rclcpp::rclcpp
    ${s57_msgs_TARGETS}
    tf2_geometry_msgs::tf2_geometry_msgs
  PRIVATE
    ${geodesy_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(marine_charts nav2_costmap_2d pluginlib rclcpp s57_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav2_util REQUIRED)

  ament_add_gtest(test_allow_uncharted test/test_allow_uncharted.cpp)
  target_include_directories(test_allow_uncharted PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_allow_uncharted
    ${PROJECT_NAME}
    nav2_costmap_2d::nav2_costmap_2d_core
    nav2_util::nav2_util_core
    rclcpp::rclcpp
    tf2_ros::tf2_ros
  )

  ament_add_gtest(test_tide_offset test/test_tide_offset.cpp)
  target_include_directories(test_tide_offset PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_tide_offset
    ${PROJECT_NAME}
    nav2_costmap_2d::nav2_costmap_2d_core
    nav2_util::nav2_util_core
    rclcpp::rclcpp
    tf2_ros::tf2_ros
  )

  ament_add_gtest(test_depth_costs test/test_depth_costs.cpp)
  target_include_directories(test_depth_costs PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_depth_costs
    ${PROJECT_NAME}
    nav2_costmap_2d::nav2_costmap_2d_core
    nav2_util::nav2_util_core
    rclcpp::rclcpp
    tf2_ros::tf2_ros
  )

  ament_add_gtest(test_plugin_loading test/test_plugin_loading.cpp)
  target_link_libraries(test_plugin_loading
    nav2_costmap_2d::nav2_costmap_2d_core
    pluginlib::pluginlib
  )

  # One cycle of a rolling LayeredCostmap with the layer, and Nav2's
  # inflation layer for the uncharted case (#56).
  ament_add_gtest(test_rolling_window test/test_rolling_window.cpp)
  target_include_directories(test_rolling_window PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_rolling_window
    ${PROJECT_NAME}
    nav2_costmap_2d::layers
    nav2_costmap_2d::nav2_costmap_2d_core
    nav2_util::nav2_util_core
    rclcpp::rclcpp
    tf2_ros::tf2_ros
  )

  find_package(ament_lint_auto REQUIRED)
  # Skip copyright: existing files lack headers; tracked separately.
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint/uncrustify: pre-existing code style (Allman braces,
  # compact for-loops) differs from ament defaults. Reformatting the
  # codebase is out of scope for this change.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

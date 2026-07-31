cmake_minimum_required(VERSION 3.8)
project(s57_to_geotiff)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(GDAL REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(marine_autonomy REQUIRED)      # header-only gggs::Level::fromCellSize
find_package(marine_charts REQUIRED)
find_package(marine_vertical_datum REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Core export logic, split out so the test can link it.
add_library(${PROJECT_NAME}_core STATIC
  src/exporter.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  GDAL::GDAL
  marine_autonomy::marine_autonomy
  marine_charts::marine_charts
  marine_vertical_datum::marine_vertical_datum
  ${geographic_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  tf2::tf2
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_core)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_exporter test/test_exporter.cpp)
  target_link_libraries(test_exporter ${PROJECT_NAME}_core GDAL::GDAL)

  find_package(ament_lint_auto REQUIRED)
  # Skip copyright/cpplint/uncrustify: matches sibling packages
  # (marine_charts, s57_grids, s57_layer) — the codebase uses a non-ament
  # style and headers carry no per-file copyright.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

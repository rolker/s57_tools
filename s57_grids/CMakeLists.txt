cmake_minimum_required(VERSION 2.8.3)
project(s57_grids)

find_package(catkin REQUIRED COMPONENTS
    grid_map_ros
    roscpp
    roslib
    s57_msgs
    tf2_geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
        grid_map_ros
        roscpp
        roslib
        s57_msgs
        tf2_geometry_msgs
    INCLUDE_DIRS include
    #LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(GDAL REQUIRED)
include_directories(${GDAL_INCLUDE_DIR})

add_subdirectory(src/iso8211)

add_executable(${PROJECT_NAME} src/main.cpp src/s57_catalog.cpp src/s57_dataset.cpp src/grid_publisher.cpp)
add_dependencies(${PROJECT_NAME} ${s57_grids_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${GDAL_LIBRARY} ${catkin_LIBRARIES} iso8211)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# install(FILES costmap_plugins.xml
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

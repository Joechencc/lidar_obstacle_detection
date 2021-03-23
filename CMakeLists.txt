cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(lidar_obstacle_detection)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(PCL 1.2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp roslaunch nodelet rviz std_msgs std_srvs geometry_msgs sensor_msgs tf laser_geometry message_generation)

SET(CMAKE_MODULE_PATH ${${PROJECT_NAME}_SOURCE_DIR}/cmake)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelets
  CATKIN_DEPENDS roscpp nodelet rviz std_msgs std_srvs geometry_msgs sensor_msgs tf laser_geometry message_runtime
#  DEPENDS system_lib
)

include_directories( 
    include
    ${${PROJECT_NAME}_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS} 
)

#
# Build libs
#
add_library(detect src/detect.cpp)
target_link_libraries(detect ${catkin_LIBRARIES})
add_dependencies(detect ${catkin_EXPORTED_TARGETS})

#
# Build nodes
#

add_executable(detect_node src/nodes/detect_node.cpp)
target_link_libraries(detect_node detect)

#
# Build nodelets
#

add_library(${PROJECT_NAME}_nodelets
  src/nodelets/detect_nodelet.cpp)

target_link_libraries(${PROJECT_NAME}_nodelets detect)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (main src/main.cpp)
target_link_libraries (main ${PROJECT_NAME}_nodelets ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


#add_library(${PROJECT_NAME}_gui)
  # src/displays/obstacles_display.cpp include/${PROJECT_NAME}/displays/obstacles_display.h
  # src/displays/circle_visual.cpp include/${PROJECT_NAME}/displays/circle_visual.h
  # src/displays/segment_visual.cpp include/${PROJECT_NAME}/displays/segment_visual.h
  #
  # src/panels/scans_merger_panel.cpp include/${PROJECT_NAME}/panels/scans_merger_panel.h
  # src/panels/obstacle_extractor_panel.cpp include/${PROJECT_NAME}/panels/obstacle_extractor_panel.h
  # src/panels/obstacle_tracker_panel.cpp include/${PROJECT_NAME}/panels/obstacle_tracker_panel.h
  # src/panels/obstacle_publisher_panel.cpp include/${PROJECT_NAME}/panels/obstacle_publisher_panel.h)
#target_link_libraries(${PROJECT_NAME}_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME}_gui ${catkin_EXPORTED_TARGETS})


#
# Install libraries
#
install(TARGETS ${PROJECT_NAME}_nodelets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#
# Install nodes
#
install(TARGETS detect_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#
# Install header files
#
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


#
# Install nodelet and rviz plugins description
#
install(FILES nodelet_plugins.xml rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


#
# Install launch files
#
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

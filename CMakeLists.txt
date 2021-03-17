cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lidar_obstacle_detection)

find_package(PCL 1.2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp roslaunch nodelet rviz std_msgs std_srvs geometry_msgs sensor_msgs tf laser_geometry message_generation)

SET(CMAKE_MODULE_PATH ${${PROJECT_NAME}_SOURCE_DIR}/cmake)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories( 
    ${${PROJECT_NAME}_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS} 
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (main src/main.cpp)
target_link_libraries (main ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


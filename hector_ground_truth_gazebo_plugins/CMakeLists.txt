cmake_minimum_required(VERSION 3.5.1)
project(hector_ground_truth_gazebo_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS grid_map_ros hector_ground_truth_gazebo_plugins_msgs)

find_package(GAZEBO REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hector_contact_points_publisher_plugin hector_elevation_map_ground_truth_plugin
  CATKIN_DEPENDS hector_ground_truth_gazebo_plugins_msgs
  DEPENDS GAZEBO
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(hector_contact_points_publisher_plugin src/contact_points_publisher_plugin.cpp)
target_link_libraries(hector_contact_points_publisher_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(hector_elevation_map_ground_truth_plugin src/elevation_map_ground_truth_plugin.cpp)
target_link_libraries(hector_elevation_map_ground_truth_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

 install(TARGETS hector_contact_points_publisher_plugin hector_elevation_map_ground_truth_plugin
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )

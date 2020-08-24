cmake_minimum_required(VERSION 3.5.1)
project(hector_ground_truth_gazebo_plugins_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs grid_map_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################


add_message_files(
  FILES
  ContactPointsStamped.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GenerateGroundTruth.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs grid_map_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS geometry_msgs grid_map_msgs message_runtime
)

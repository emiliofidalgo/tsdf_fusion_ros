cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(tsdf_fusion_ros)

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FusionFromDir.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES  
  std_msgs
  nav_msgs
)

# Defining the package
catkin_package(
  CATKIN_DEPENDS
  roscpp 
  rospy 
  std_msgs  
  nav_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
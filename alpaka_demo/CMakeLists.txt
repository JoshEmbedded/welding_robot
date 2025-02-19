cmake_minimum_required(VERSION 3.0.2)
project(alpaka_demo)

## Find required catkin and Gazebo ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  moveit_core
  moveit_ros_planning_interface
  laser_geometry
  
  rosbag
)

find_package(gazebo_ros REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES alpaka_demo
  CATKIN_DEPENDS rospy roscpp std_msgs sensor_msgs geometry_msgs moveit_core moveit_ros_planning_interface gazebo_ros rosbag tf laser_geometry
)

## Specify additional locations of header files
include_directories(
  include
  include/headers
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)


## Link Gazebo libraries
link_directories(${GAZEBO_LIBRARY_DIRS})

## Install Python scripts
catkin_install_python(PROGRAMS
  src/clean_plane_matching.py
  src/clean_sphere_matching.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)







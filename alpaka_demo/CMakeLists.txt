cmake_minimum_required(VERSION 3.0.2)
project(alpaka_demo)

## Find required catkin and Gazebo ROS packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(gazebo_ros REQUIRED)

## Include directories
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs
)

## Link Gazebo libraries
link_directories(${GAZEBO_LIBRARY_DIRS})

## Install Python scripts
catkin_install_python(PROGRAMS
  src/scan_pose.py
  src/scan_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


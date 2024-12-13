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
)

find_package(gazebo_ros REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES alpaka_demo
  CATKIN_DEPENDS rospy roscpp std_msgs sensor_msgs geometry_msgs moveit_core moveit_ros_planning_interface gazebo_ros
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)


## Link Gazebo libraries
link_directories(${GAZEBO_LIBRARY_DIRS})

## Install Python scripts
catkin_install_python(PROGRAMS
  src/scan_pose.py
  src/scan_test.py
  src/marker_publish.py
  src/transformCalculation.py
  src/move_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Declare a C++ executable
add_executable(weld_seam_moveIt src/posePlan_moveIt.cpp)

## Add cmake target dependencies of the executable
add_dependencies(weld_seam_moveIt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link the library to the executable
target_link_libraries(weld_seam_moveIt ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

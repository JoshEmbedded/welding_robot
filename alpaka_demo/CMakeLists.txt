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

add_service_files(
    FILES
    ProcessBag.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

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
  src/scan_pose.py
  src/scan_test.py
  src/laser_scan_viz.py
  src/marker_publish.py
  src/laser_calibration.py
  src/transformCalculation.py
  src/servo_test.py
  src/tf_scan_test.py
  src/y_rot_test.py
  src/plane_matching.py
  src/sphere_matching.py
  src/plane_segmentation.py
  src/clean_plane_matching.py
  src/clean_sphere_matching.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(calibrationClass src/calibrationClass.cpp)

## Declare a C++ executable
add_executable(weld_seam_moveIt src/posePlan_moveIt.cpp)

## Declare a C++ executable
add_executable(calibration src/calibration.cpp)
add_executable(rotation_test src/rotation_test.cpp)
add_executable(pointCloud_test src/test_pointCloud.cpp)

## Add cmake target dependencies of the executable
add_dependencies(weld_seam_moveIt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(rotation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pointCloud_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link the library to the executable
target_link_libraries(weld_seam_moveIt ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(calibration ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} calibrationClass)
target_link_libraries(calibrationClass ${catkin_LIBRARIES})
target_link_libraries(rotation_test ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} calibrationClass)
target_link_libraries(pointCloud_test ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} calibrationClass)
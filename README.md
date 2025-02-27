
# Welding Robot

## alpaka_eff Branch Description

This branch contains the custom config and models of Alpaka custom welding end-effector mounted on a UR10e robot.
The custom config for these robots have been created so the robot can be used in Gazebo with MoveIt. 

This is purely for aestetics. **Important to note that the end-effector does not have a collision model attached**, allowing the laser origin to be inside the model.

## Brief Description

This package contains code that can run the operation of a custom UR5e robot to perform weld seam detection using laser scan data and perform laser scan to flange calibration.

## Purpose/Use Case

Simple implementation of weld seam tracking.

## Installation Instructions

- **ROS Noetic**  
- **Dependencies**:  
  - [MoveIt](https://github.com/moveit/moveit)  
  - [Universal Robots Package](https://github.com/ros-industrial/universal_robot)  
  - [UR Robot Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)  
  - [Welding Robot](https://github.com/JoshEmbedded/alpaka_demo)
  - [sick_scan_xd](https://github.com/SICKAG/sick_scan_xd)  
  - [laser_filters](https://github.com/ros-perception/laser_filters)  
  - **open3d**  
  - **scipy**  

1. Create a custom catkin workspace for ROS.  
2. Install the dependencies using `rosdep install`.  
3. Build all dependencies with `catkin_make`.

**To implement real UR Robot** follow instructions from this [ROS Wiki](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial).

## Usage

Run separate launches in different terminal shells.

### Gazebo Simulation

To launch the robot in Gazebo:

```bash
roslaunch ur10e_alpaka demo_gazebo.launch
```

To launch with simulated noise from the laser scanner:

```bash
roslaunch ur10e_alpaka demo_gazebo.launch enable_noise:=true
```

### Real ROS Robot Driver 

To launch the real UR5e ROS Driver (robot ip = 192.168.251.102):

Update robot_ip parameter with correct address.

```bash
roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.251.102
```

To launch a MoveIt with RViz GUI:

```bash
roslaunch ur10e_moveit_config moveit_planning_execution.launch
```

```bash
roslaunch ur10e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur10e_moveit_config)/launch/moveit.rviz
```

### Sick TIM 561

To launch the driver for the Sick TIM 561 (`/scan` topic):

```bash
roslaunch sick_scan_xd sick_tim_5xx.launch hostname:=192.168.0.55
```
update 'hostname' parameter with the ip address of the laser scanner.

### Seam Detection

To detect a seam and move the robot to the detected marker using MoveIt:

```bash
roslaunch alpaka_demo seam_follow.launch
```

By default, the seam-following method is the local minima/maxima method.

To use the vector intersection method:

```bash
roslaunch alpaka_demo seam_follow vector_intersection_mode:=true
```

To set the laser scan topic (e.g., `/scan` for the real Sick TIM 561 scanner):

```bash
roslaunch alpaka_demo seam_follow scan_topic:=/scan
```


## Features

- The **seam follow launch script** will:
  - Run a seam detection node.
  - Convert the pose to a marker and publish it in RViz.
  - Move the robot in Gazebo to the detected marker.

- The **calibration programs** follow the method from the IEEE published paper:
  - [**'An easy hand-eye calibration method for laser profile scanners in high precision applications using optimized view poses'**](https://ieeexplore.ieee.org/document/9926472/)

## Configuration

- The `laser_filters.yaml` file inside `alpaka_demo/include/config` allows modification of the filter method for laser scan data.

- The custom configuration package `ur10e_alpaka` was abstracted from the UR10e config from the Universal Robots package but added with a custom `laser_scan.xacro` and end-effector model from Alpaka Industries.

## Contribution

Joshua Schiff

## License

MIT


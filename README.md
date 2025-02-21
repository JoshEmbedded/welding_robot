
# Welding Robot

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
  - **laser_filters package**  
  - **open3d**  
  - **scipy**  

1. Create a custom catkin workspace for ROS.  
2. Install the dependencies using `rosdep install`.  
3. Build all dependencies with `catkin_make`.

## Usage

Run separate launches in different terminal shells.

### To launch the robot in Gazebo:

```bash
roslaunch ur5e_laser_sim demo_gazebo.launch
```

### To launch with simulated noise from the laser scanner:

```bash
roslaunch ur5e_laser_sim demo_gazebo.launch enable_noise:=true
```

### To detect a seam and move the robot to the detected marker using MoveIt:

```bash
roslaunch alpaka_demo seam_follow.launch
```

By default, the seam-following method is the local minima/maxima method.

### To use the vector intersection method:

```bash
roslaunch alpaka_demo seam_follow vector_intersection_mode:=true
```

### To set the laser scan topic (e.g., `/scan` for the real Sick TIM 561 scanner):

```bash
roslaunch alpaka_demo seam_follow scan_topic:=/scan
```

### Calibration:

The laser scan filter must be launched in a separate shell:

```bash
roslaunch alpaka_demo laser_filter.launch
```

To run orientation calibration:

```bash
roslaunch alpaka_demo clean_plane_matching.launch
```

For translation calibration, place a sphere with a radius of ~0.01 at position `(0, 0.42, radius)`:

```bash
roslaunch alpaka_demo clean_sphere_matching.launch
```

## Features

- The **seam follow launch script** will:
  - Run a seam detection node.
  - Convert the pose to a marker and publish it in RViz.
  - Move the robot in Gazebo to the detected marker.

- The **calibration programs** follow the method from the IEEE published paper:
  - **'An easy hand-eye calibration method for laser profile scanners in high precision applications using optimized view poses'** by Udo Paschke et al.

## Configuration

- The `laser_filters.yaml` file inside `alpaka_demo/include/config` allows modification of the filter method for laser scan data.

- The custom configuration package `ur5e_laser_sim` was abstracted from the UR5e config from the Universal Robots package but added with a custom `laser_scan.xacro`.

## Contribution

Joshua Schiff

## License

MIT


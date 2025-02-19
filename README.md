# Welding Robot Project

This repository contains the code and configuration for an autonomous welding robot using ROS and Gazebo simulation. The repository is structured into two sub-packages:

- **`ur5e_laser_sim`**: Contains the robot description, URDF, and simulation configuration.
- **`alpaka_demo`**: Includes the laser filtering and calibration scripts.

---

## Prerequisites

Ensure you have the following installed:
- **ROS Noetic**
- **Gazebo**
- **MoveIt**
- **Python dependencies** (for calibration scripts):

  ```bash
  sudo apt install python3-numpy python3-scipy python3-rospkg python3-open3d
  ```
  ## Launching the Welding Robot in Simulation

1. **Start the robot simulation in Gazebo**
   ```bash
   roslaunch ur5e_laser_sim demo_gazebo.launch
   ```
2. **Press the Play button** in Gazebo to start the simulation.  
3. **Add a floor plane** as a scene object (important for path planning to avoid collisions).  

---

## Running the Laser Scanner Filter

The laser scanner filter is used to reduce noise from the sensor plugin.

```bash
roslaunch alpaka_demo laser_filter.launch
```

---

## Running the Calibration Programs

### Orientation Calibration  
To calibrate the orientation of the laser scanner using a flat plane:

```bash
rosrun alpaka_demo clean_plane_matching.py
```

### Translation Calibration  
To calibrate the translation component using a sphere:

```bash
rosrun alpaka_demo clean_sphere_matching.py
```

import spatialmath as sm
import numpy as np
import math

def calculate_sensor_to_tcp_offset(robot_poses, laser_scan_point, rotations, sensor_rotation_angle):
    """
    Calculate the sensor-to-TCP offset based on multiple robot poses and laser scan point.
    
    Args:
    - robot_poses: A list of 4 robot poses (each as a list [x, y, z]).
    - laser_scan_point: The laser scan point in the sensor frame [x, y, z].
    - rotations: A list of Euler angles (rot_z, rot_y, rot_x) for each robot pose.
    
    Returns:
    - sensor_to_tcp_offset: The averaged sensor-to-TCP offset [x, y, z].
    """
    sensor_offsets = []
    
    # Rotation between sensor and TCP (apply rotation in the X-axis)
    sensor_to_tcp_rotation = sm.base.trotx(np.deg2rad(sensor_rotation_angle))

    # Iterate over each robot pose and its corresponding rotation
    for i in range(len(robot_poses)):
        robot_pose = robot_poses[i]
        rot_x, rot_y, rot_z = rotations[i]
        
        # Step 1: Define the robot's TCP pose (in robot base frame) using the pose and orientation
        R_robot = sm.SE3(robot_pose) 
        R_robot *= sm.base.eul2tr(rot_z, rot_y, rot_x)
        
        # Step 2: Define the transformation from TCP to sensor (sensor_to_TCP)
        # For now we assume that sensor_to_TCP is unknown and will calculate it from the robot pose
        # Transform the laser scan point from the sensor frame to the robot's base frame
        laser_scan = laser_scan_point[i]
        laser_scan[0] = -laser_scan[0]
        transformed_scan_point = sm.base.homtrans(sm.base.trinv(R_robot) * sm.base.trinv(sensor_to_tcp_rotation), laser_scan)
        print("robot pose:", robot_pose)
        print("transformed pose: ", transformed_scan_point)
        # Step 3: Calculate the sensor to TCP offset
        sensor_to_tcp_offset = [robot_pose[0] - transformed_scan_point[0],
                                robot_pose[1] - transformed_scan_point[1],
                                robot_pose[2] - transformed_scan_point[2]]
        
        sensor_offsets.append(sensor_to_tcp_offset)
    
    # Step 4: Average the sensor offsets from all poses
    avg_sensor_to_tcp_offset = np.mean(sensor_offsets, axis=0)
    
    return avg_sensor_to_tcp_offset

# Example Usage
if __name__ == '__main__':
    # Define four robot poses in the base frame [x, y, z]
    robot_poses = [
        [-0.9318, -0.3521, 0.38742],  # Pose 1
        [-0.91325, -0.37533, 0.32968],  # Pose 2
        [-0.91777, -0.32534, 0.32977],  # Pose 3
        [-0.91553, -0.35034, 0.32967]   # Pose 4
    ]
    
    # Laser scan point in the sensor frame [x, y, z]
    laser_scan_point = [
        [0, 0, 0.080],
        [0.025,0,0.14],
        [-0.025,0,0.14],
        [0,0, 0.14]
    ]
    
    # Define corresponding rotations (Euler angles) for each pose
    rotations = [
        [5.447, 0.273, -0.759],  # Rotation for Pose 1
        [5.447, 0.273, -0.759],  # Rotation for Pose 2
        [5.447, 0.273, -0.759],  # Rotation for Pose 3
        [5.447, 0.273, -0.759]   # Rotation for Pose 4
    ]
    
    # Rotation between sensor and TCP in the X-axis (in degrees)
    sensor_rotation_angle = 16.1
    
    # Calculate the sensor-to-TCP offset
    sensor_to_tcp_offset = calculate_sensor_to_tcp_offset(robot_poses, laser_scan_point, rotations, sensor_rotation_angle)
    
    print(f"Calculated Sensor-to-TCP Offset: {sensor_to_tcp_offset}")

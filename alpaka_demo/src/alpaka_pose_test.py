# test_example.py
import rospy
import tf
import numpy as np
import math
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

def transform_point(sensor_point, rotation_matrix, translation_vector):
    """
    Transforms a point from the sensor frame to the TCP frame.
    
    Args:
        sensor_point (list or np.array): The point in the sensor frame [x, y, z].
        rotation_matrix (np.array): The 3x3 rotation matrix describing sensor-to-TCP orientation.
        translation_vector (list or np.array): The 3x1 translation vector describing sensor-to-TCP position.
    
    Returns:
        np.array: The transformed point in the TCP frame [x, y, z].
    """
    # Ensure inputs are numpy arrays
    sensor_point = np.array(sensor_point)
    rotation_matrix = np.array(rotation_matrix)
    translation_vector = np.array(translation_vector)
    
    # Verify dimensions
    if sensor_point.shape != (3,):
        raise ValueError("sensor_point must be a 3-element vector [x, y, z].")
    if rotation_matrix.shape != (3, 3):
        raise ValueError("rotation_matrix must be a 3x3 matrix.")
    if translation_vector.shape != (3,):
        raise ValueError("translation_vector must be a 3-element vector [x, y, z].")
    
    # Perform the transformation: p_tcp = R * p_sensor + t
    tcp_point = np.dot(rotation_matrix, sensor_point) + translation_vector
    
    return tcp_point

def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))

if __name__ == '__main__':
    
    robot_pose = []
    robot_pose.append(-0.58248)
    robot_pose.append(-0.57453)
    robot_pose.append(0.70747)
    
    rot_x = 0.034
    rot_y = 0.568
    rot_z = -0.226
    
    robot_rpy = [rot_z, rot_y, rot_x]
    
    robot_rpy = [rot_x, rot_y, rot_z]
    # robot_rpy = [0.059, 0.559, 0.174]
    
    # for i in range(len(robot_rpy)):
    #     if abs(robot_rpy[i]) > math.pi:
    #         if robot_rpy[i] < 0:
    #             robot_rpy[i] += 2*math.pi
    #         else:
    #             robot_rpy[i] -= 2*math.pi
    #     print(robot_rpy[i])
    
    
    # Normalize robot_rpy angles
    # robot_rpy = [normalize_angle(angle) for angle in robot_rpy]
    
    # rx = 0.221
    # ry = 0.518
    # rz = -0.024
    
    #First method: Point is from sensor to point
    point_x = 0.00845
    point_x = -point_x #x axis is inversed on sensor plane
    point_y = 0 #only scans along x,z plane
    point_z = 0.1388
    point_z = point_z
    point_scanned = [point_x, point_y, point_z]
    
    #First attempt where the offset is from TCP frame to sensor
    x_rot_angle = 16.1
    offset_x = 0
    offset_y = 0.07268 #updated based on real calculation
    # offset_z = -0.11511 #calculated from offset on plan diagram
    offset_z = 10**-3 * -122.69 * math.cos(np.deg2rad(x_rot_angle)) # height offset based on testing

    sensor_to_TCP_translation = [offset_x, offset_y, offset_z]
    
    sensor_to_TCP_rotation = R.from_euler('X', x_rot_angle, degrees=True)
    
    sensor_to_TCP_rotation = sensor_to_TCP_rotation.as_matrix()
    
    transformed_point_tcp_frame = transform_point(point_scanned, sensor_to_TCP_rotation, sensor_to_TCP_translation)
    print("Transformed Point in TCP Frame:", transformed_point_tcp_frame)
    
    tcp_to_base_rotation = R.from_euler('xyz', robot_rpy, degrees=False)
    # tcp_to_base_rotation = R.from_rotvec(robot_rpy)
    print("quat:", tcp_to_base_rotation.as_quat())
    tcp_to_base_rotation = tcp_to_base_rotation.as_matrix()
    print("rotation", tcp_to_base_rotation)
#     tcp_to_base_rotation = np.array([
#     [0.3081302, -0.9427088, 0.1278900],
#     [-0.0495385, 0.1183491, 0.9917356],
#   [-0.9500535, -0.3119192, -0.0102335]])
    
    # print("online calculator", tcp_to_base_rotation)
    
    
    tcp_to_base_translation = [robot_pose[0], robot_pose[1], robot_pose[2]]
    
    transformed_point = transform_point(transformed_point_tcp_frame, tcp_to_base_rotation, tcp_to_base_translation)
    print("Relative point to robot pose:", transformed_point)

    
    
    
    
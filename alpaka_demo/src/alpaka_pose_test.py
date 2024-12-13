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


if __name__ == '__main__':
    
    robot_pose = []
    robot_pose.append(-0.81873)
    robot_pose.append(-0.28587)
    robot_pose.append(0.84057)
    
    robot_rpy = [0, 0, 0.362]
    # rx = 0.221
    # ry = 0.518
    # rz = -0.024
    
    #First method: Point is from sensor to point
    point_x = -0.01166
    point_x = -point_x #x axis is inversed on sensor plane
    point_y = 0 #only scans along x,z plane
    point_z = 0.1176
    point_z = point_z
    point_scanned = [point_x, point_y, point_z]
    
    
    #Second method: where the offset is considers going from sensor to tcp frame
    # point_x = -0.01166
    # point_x = -point_x #x axis is inversed on sensor plane
    # point_y = 0 #only scans along x,z plane
    # point_z = 0.1176
    # point_z = -point_z
    # point_scanned = [point_x, point_y, point_z]
    
    #First attempt where the offset is from TCP frame to sensor
    offset_x = 0
    offset_y = 0.07192
    offset_z = -0.11511 #calculated from offset on plan diagram
    
    #Second Method
    # offset_x = 0
    # offset_y = -0.07192
    # offset_z = 0.11511 #calculated from offset on plan diagram
    
    sensor_to_TCP_translation = [offset_x, offset_y, offset_z]
    
    sensor_to_TCP_rotation = R.from_euler('x', 16.1, degrees=True)
    
    sensor_to_TCP_rotation = sensor_to_TCP_rotation.as_matrix()
    
    transformed_point_tcp_frame = transform_point(point_scanned, sensor_to_TCP_rotation, sensor_to_TCP_translation)
    print("Transformed Point in TCP Frame:", transformed_point_tcp_frame)
    
    tcp_to_base_rotation = R.from_euler('xyz', robot_rpy)
    tcp_to_base_rotation = tcp_to_base_rotation.as_matrix()
    tcp_to_base_translation = [robot_pose[0], robot_pose[1], robot_pose[2]]
    
    transformed_point = transform_point(transformed_point_tcp_frame, tcp_to_base_rotation, tcp_to_base_translation)
    print("Relative point to robot pose:", transformed_point)
    # robot_pose[0]+=transformed_point[0]
    # robot_pose[1]+=transformed_point[1]
    # robot_pose[2]+=transformed_point[2]
    # print("Updated robot position:", robot_pose )
    # print(sensor_to_TCP_rotation)
    
    # rotation = R.from_euler('xyz', [rx, ry, rz], degrees=False)
    # rotation = rotation.as_matrix()
    # print(rotation)
    
    
    
    
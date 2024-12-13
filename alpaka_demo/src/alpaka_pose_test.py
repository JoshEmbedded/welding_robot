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
    robot_pose.append(-0.54826)
    robot_pose.append(-0.50286)
    robot_pose.append(0.70163)
    # rx = 0.221
    # ry = 0.518
    # rz = -0.024
    
    point_x = 0.00854
    point_x = -point_x #x axis is inversed on sensor plane
    point_y = 0 #only scans along x,z plane
    point_z = 0.135
    point_scanned = [point_x, point_y, point_z]
    
    offset_x = 0
    offset_y = 0.07237
    offset_z = -0.08944 #calculated from offset on plan diagram
    
    sensor_to_TCP_translation = [offset_x, offset_y, offset_z]
    sensor_to_TCP_rotation = R.from_euler('x', 16.5, degrees=True)
    sensor_to_TCP_rotation = sensor_to_TCP_rotation.as_matrix()
    
    transformed_point = transform_point(point_scanned, sensor_to_TCP_rotation, sensor_to_TCP_translation)
    print("Relative point to robot pose:", transformed_point)
    robot_pose[0]+=transformed_point[0]
    robot_pose[1]+=transformed_point[1]
    robot_pose[2]+=transformed_point[2]
    print("Updated robot position:", robot_pose )
    # print(sensor_to_TCP_rotation)
    
    # rotation = R.from_euler('xyz', [rx, ry, rz], degrees=False)
    # rotation = rotation.as_matrix()
    # print(rotation)
    
    
    
    
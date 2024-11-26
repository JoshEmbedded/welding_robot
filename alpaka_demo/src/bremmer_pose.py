import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf
import geometry_msgs.msg 
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

def parse_data(input_string):
    # Split the input string by the colon ':'
    data_points = input_string.split(':')
    
    # Example: Return a dictionary with the index as keys (if you need to store them)
    parsed_data = {}
    for i, data in enumerate(data_points):
        parsed_data[f"data_{i+1}"] = data.strip()  # .strip() removes any extra spaces
    
    return parsed_data

def create_pose(raw_data):
    
    parsed_data = parse_data(raw_data)
    
    x = parsed_data[0]
    z = parsed_data[1]
    rpy = adjust_angle(parsed_data[3], 45)
    
    
def adjust_angle(angle, target):
    
    # Normalize the angle to ensure it's within the range of 0-360 degrees
    angle = angle % 360
    
    # Check the difference and adjust the angle
    if angle != target:
        # Calculate the difference from target
        difference = angle - target
        
        # Adjust the angle to target by subtracting or adding the difference
        if difference > 0:
            angle -= difference  # Subtract the difference
        else:
            angle += abs(difference)  # Add the absolute difference
    
    return angle

def make_transformation_matrix(x, y, z, rotation):
    
    # rotation can be in any format: Euler ('xyz'), Quaternion, etc.
    rot_matrix = R.from_euler('xyz', rotation).as_matrix()  # Replace 'xyz' with format if needed
    trans_matrix = np.eye(4)
    trans_matrix[:3, :3] = rot_matrix
    trans_matrix[:3, 3] = [x, y, z]
    
    return trans_matrix

def create_transform_stamped(x, y, z, 
                              rpy=None, quaternion=None, 
                              parent_frame="world", child_frame="child", timestamp=None):
    """
    Creates a TransformStamped message.
    
    Parameters:
        x, y, z (float): Translation in meters.
        rpy (tuple of floats): Roll, Pitch, Yaw in radians. Optional if quaternion is provided.
        quaternion (tuple of floats): Quaternion (x, y, z, w). Optional if rpy is provided.
        parent_frame (str): Name of the parent frame.
        child_frame (str): Name of the child frame.
        timestamp (rospy.Time): Timestamp for the transform. Default is rospy.Time.now().
        
    Returns:
        geometry_msgs/TransformStamped: The resulting transform stamped message.
    """
    if not (rpy or quaternion):
        raise ValueError("Either rpy or quaternion must be provided.")
    
    # Compute quaternion if RPY is provided
    if rpy:
        roll, pitch, yaw = rpy
        quaternion = quaternion_from_euler(roll, pitch, yaw)
    
    if not timestamp:
        timestamp = 0

    # Create TransformStamped message
    transform = TransformStamped()
    transform.header.stamp = timestamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    
    # Set translation
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z
    
    # Set rotation
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    
    return transform
    

def get_transformed_pose(T_tcp_to_flange, T_flange_to_sensor, pose_in_sensor_frame):
    
    # Inverse the Sensor to Flange transform
    T_flange_to_sensor = np.linalg.inv(T_flange_to_sensor)

    # Compute the TCP to Sensor transform
    T_tcp_to_sensor = np.dot(T_tcp_to_flange, T_flange_to_sensor)

    pose_in_tcp_frame = np.dot(T_tcp_to_sensor, pose_in_sensor_frame)
    

if __name__ == '__main__':
    
    x, y, z = 72.37, 26.84, 529.14
    rotation = [-16.5, -39, 90]
    transformation_matrix = make_transformation_matrix(x, y, z, np.radians(rotation))
    
    laser_transform = create_transform_stamped(x, y, z, rpy=rotation, parent_frame='welding_eff_link', child_frame='laser_scanner_link')
    tcp_eff_transform = create_transform_stamped(x=0.04561, y=0, z=0.61858, quaternion=[0,0.33348,0,0.94275], parent_frame='welding_eff_link', child_frame='tcp_eff')
    
    print(laser_transform)
    print(tcp_eff_transform)
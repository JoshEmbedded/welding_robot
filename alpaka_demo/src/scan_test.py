# test_example.py
import rospy
from collections import deque
import tf
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import tf
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
import geometry_msgs.msg 
from geometry_msgs.msg import PoseStamped

laser_data_buffer = deque(maxlen=10)  # Store the last 10 messages
new_data = True
laser_data = LaserScan()
global seam_pose

# Create a publisher (global to avoid creating it repeatedly in the function)
weld_seam_pose_pub = rospy.Publisher('/weld_seam_pose', PoseStamped, queue_size=10)


def laser_scan_callback(msg):
    """
    Callback function to process incoming LaserScan messages.
    """
    global new_data, laser_data
    if new_data:
        laser_data = msg
        find_seam(laser_data)
        new_data = False  # Set to False after processing

import numpy as np

def interpolate_nan_with_tolerance(data, tolerance):
    """
    Interpolates NaN values in a 1D array based on surrounding valid values within a certain tolerance.

    Parameters:
        data (np.ndarray): The input 1D array containing `NaN` values.
        tolerance (float): The maximum allowed gap (in indices) for interpolation to occur.

    Returns:
        np.ndarray: A new array with interpolated values for `NaN` elements within the tolerance range.
    """
    # Create a copy to avoid modifying the original data
    data_interpolated = np.copy(data)

    # Find indices of NaN values
    nan_indices = np.where(np.isnan(data))[0]

    for nan_idx in nan_indices:
        # Look for valid values before and after the current NaN index
        valid_before = None
        valid_after = None

        # Search backward for the nearest valid value
        for i in range(nan_idx - 1, -1, -1):
            if not np.isnan(data[i]):
                valid_before = (i, data[i])
                break

        # Search forward for the nearest valid value
        for i in range(nan_idx + 1, len(data)):
            if not np.isnan(data[i]):
                valid_after = (i, data[i])
                break

        # Interpolate only if both valid_before and valid_after are within the tolerance range
        if valid_before and valid_after:
            gap_before = nan_idx - valid_before[0]
            gap_after = valid_after[0] - nan_idx

            if gap_before <= tolerance and gap_after <= tolerance:
                # Linear interpolation
                value = (valid_before[1] * gap_after + valid_after[1] * gap_before) / (gap_before + gap_after)
                data_interpolated[nan_idx] = value

    return data_interpolated


def find_seam(laser_scan):
    
    # Extract the range data (assuming the ranges are in the 'ranges' field of LaserScan message)
    range_data = np.array(laser_scan.ranges)
    
    range_data[range_data == float('inf')] = np.nan  # Replace 'inf' with NaN
    
    rospy.loginfo(f"Processed range data: {range_data}")
    
    range_data = interpolate_nan_with_tolerance(range_data, 3)
    
    rospy.loginfo(f"Interpolated: {range_data}")
    
     # You can either ignore these values or replace NaNs with the previous valid range.
    # range_data = np.nan_to_num(range_data, nan=np.nan)  # This will turn NaNs into a default value like zero
    
    # Print the range data for debugging
    # rospy.loginfo(f"Range data: {range_data}")
    
    # Step 2: Find local maxima and minima
    local_maxima = []
    local_minima = []
    edge = False
    index = -1
    for i in range(1, len(range_data) - 1):  # Avoid the first and last elements
        # Skip if the current or neighboring values are NaN
        if np.isnan(range_data[i-1]) or np.isnan(range_data[i]) or np.isnan(range_data[i+1]):
            continue  # Skip this iteration
        
        if range_data[i-1] < range_data[i] > range_data[i+1]:  # Local maxima
            local_maxima.append(i)
            rospy.loginfo(f"local maxima found at indices: {i}, with range: {range_data[i]}.")
            index = i
            edge = True
        elif range_data[i-1] > range_data[i] < range_data[i+1]:  # Local minima
            local_minima.append(i)
            rospy.loginfo(f"local minima found at indices: {i}, with range: {range_data[i]}.")
            index = i
            edge = True
    
    if edge:
        rospy.loginfo("Change in direction detected:")
        calculate_pose(laser_scan, index)
        
    else:
        rospy.logerr("Change in direction failed. ROS Shutdown...")    
        
    
def calculate_pose(laser_scan, scan_index):
    
    if scan_index < 0 or scan_index >= len(laser_scan.ranges):
        rospy.logwarn(f"Invalid scan index {scan_index}. Must be between 0 and {len(laser_scan.ranges)-1}.")
        return None
    
    # Get the range (distance) at the given index
    r = laser_scan.ranges[scan_index]
    
    # update r to ensure end-effector doesn't move less than min sensor range (0.074)
    r = r - 0.074
    
    rospy.loginfo(f"updated range: {r}")
    # If the range is invalid (NaN or infinite), return None
    if r == float('Inf') or r == float('NaN'):
        rospy.logwarn(f"Invalid range value at index {scan_index}.")
        return None
    
    # Calculate the angle corresponding to the chosen scan index
    angle = laser_scan.angle_min + scan_index * laser_scan.angle_increment
    
    # Convert the polar coordinates (r, angle) to Cartesian coordinates (x, y), calculated: θ=angle_min+scan_index×angle_increment
    x = r * math.cos(angle)
    y = r * math.sin(angle)
    
    rospy.loginfo("calculated x position from laser data: %f ", x)
    rospy.loginfo("calculated y position from laser data: %f ", y)
    rospy.loginfo("calculated angle from laser data: %f ", angle)
    
    convertLaserTransform(x, y, angle)
    # # Shutdown the node after processing is complete
    # rospy.signal_shutdown("Pose calculation complete.")  # Gracefully shuts down the node

def getTransform():
    
    # Create a tf2 Buffer and Listener
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Wait for the listener to start receiving transforms
    rospy.sleep(1.0)
    
    try:
        # Look up the transform from the base_link (robot base) to the laser (sensor frame)
        transform = tf_buffer.lookup_transform('base_link', 'laser_scanner_link', rospy.Time(0))  # 'laser_frame' is the frame of your sensor
        rospy.loginfo("Transform: %s", transform)

        # Access the translation and rotation from the transform
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        rospy.loginfo(f"Translation: x={translation.x}, y={translation.y}, z={translation.z}")
        rospy.loginfo(f"Rotation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}")

        return transform
    
    except TransformException as e:
        rospy.signal_shutdown("Could not get transform: %s", e)

def convertLaserTransform(x, y, angle):
    """
    Convert a position from laser_scanner_link to base_link.

    :param x: x-coordinate in laser_scanner_link
    :param y: y-coordinate in laser_scanner_link
    :param theta: orientation (angle) in laser_scanner_link
    :return: Transformed (x, y, theta) in base_link
    """
    # Create the TF2 buffer and listener
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.sleep(1.0)  # Allow listener to start receiving transforms
    rospy.loginfo("Entered convertLaserTransform")
    try:
        # Get the transform from laser_scanner_link to base_link
        transform = tf_buffer.lookup_transform('base_link', 'laser_scanner_link', rospy.Time(0))

        # Create a PoseStamped message for the laser scanner pose in its own frame
        laser_pose = geometry_msgs.msg.PoseStamped()
        laser_pose.header.frame_id = 'laser_scanner_link'
        laser_pose.header.stamp = rospy.Time(0)
        laser_pose.pose.position.x = x
        laser_pose.pose.position.y = y
        laser_pose.pose.position.z = 0  # Assuming the laser scanner is on a 2D plane
        
        # Calculate the yaw angle representing the direction from the scanner to the point
        yaw = math.atan2(y, x)  # Angle of vector (x, y) from scanner to point
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        
        # Add a rotation to align z-axis with x-axis
        alignment_quaternion = tf.transformations.quaternion_from_euler(0, math.pi / 2, 0)
        final_quaternion = tf.transformations.quaternion_multiply(alignment_quaternion, quaternion)

        laser_pose.pose.orientation.x = final_quaternion[0]
        laser_pose.pose.orientation.y = final_quaternion[1]
        laser_pose.pose.orientation.z = final_quaternion[2]
        laser_pose.pose.orientation.w = final_quaternion[3]
        
        # Transform the laser scanner pose to base_link frame
        seam_pose = do_transform_pose(laser_pose, transform)
        transformed_pose = seam_pose
        # Extract the transformed position and angle (orientation)
        transformed_x = transformed_pose.pose.position.x
        transformed_y = transformed_pose.pose.position.y
        transformed_z = transformed_pose.pose.position.z
        transformed_yaw = tf.transformations.euler_from_quaternion([
            transformed_pose.pose.orientation.x,
            transformed_pose.pose.orientation.y,
            transformed_pose.pose.orientation.z,
            transformed_pose.pose.orientation.w
        ])[2]  # Extract the yaw angle
        
        weld_seam_pose_pub.publish(seam_pose)
        rospy.loginfo(f"Publishing weld seam pose: x={transformed_x}, y={transformed_y}, z={transformed_z}")
        return transformed_x, transformed_y, transformed_yaw

    except tf2_ros.TransformException as e:
        rospy.signal_shutdown(f"Could not get transform: {e}")
        return None, None, None
    
if __name__ == '__main__':
    rospy.init_node('test_example_node', anonymous=True)
    
    # Subscribe to the /scan topic
    rospy.Subscriber('/laser_scan', LaserScan, laser_scan_callback)
    
    
    # Keep the node running
    rospy.spin()
    
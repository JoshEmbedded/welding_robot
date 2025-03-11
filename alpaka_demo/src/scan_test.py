#!/usr/bin/env python3
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
from tf2_geometry_msgs import do_transform_point
import geometry_msgs.msg 
from geometry_msgs.msg import PoseStamped

laser_data_buffer = deque(maxlen=10)  # Store the last 10 messages
new_data = True
laser_data = LaserScan()
global seam_pose
vector_detection_method = False

# Create a publisher (global to avoid creating it repeatedly in the function)
weld_seam_pose_pub = rospy.Publisher('/weld_seam_pose', PoseStamped, queue_size=10)


def laser_scan_callback(msg):
    """
    Callback function to process incoming LaserScan messages.
    """
    global new_data, laser_data
    
    if msg is None or not hasattr(msg, "ranges") or msg.ranges is None:
        rospy.logerr("Received invalid LaserScan message!")
        return
    
    if new_data:
        laser_data = msg
        rospy.loginfo(f"Received LaserScan with {len(laser_data.ranges)} points.")
        if vector_detection_method:
            vector_intersection_seam_detection(laser_data)
        else:
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


def vector_intersection_seam_detection(laser_scan):
    """
    Detects seam by finding intersection points of vectors formed between laser scan points.

    Parameters:
        laser_scan (list or np.ndarray): Laser scan range data.

    Returns:
        list: A list of intersection points (x, y) in 2D space.
    """
    
    # Extract the range data (assuming the ranges are in the 'ranges' field of LaserScan message)
    range_data = np.array(laser_scan.ranges)
    
    range_data[range_data == float('inf')] = np.nan  # Replace 'inf' with NaN
    
    
    angle_min = laser_scan.angle_min
    angle_increment = laser_scan.angle_increment
    
    # Step 1: Convert laser scan ranges to 2D points
    angles = np.arange(angle_min, angle_min + len(range_data) * angle_increment, angle_increment)
    points = np.array([
        (r * np.cos(a), r * np.sin(a)) if np.isfinite(r) else (np.nan, np.nan)
        for r, a in zip(range_data, angles)
    ])
    
    intersection_points = []

    threshold = 0.01
    new_vector = True
    stored_gradients = []
    
    for i in range(len(points)-2):
        
        p1, p2 = points[i], points[i+1]
        
        # Skip if any of the points are invalid
        if np.isnan(p1).any() or np.isnan(p2).any():
            continue
        
        vector = np.array(p2) - np.array(p1)
        gradient = (p2[1]-p1[1])/(p2[0]-p1[0])
        
        if new_vector:
            intersection_points.append(p1)
            prev_gradient = gradient
            stored_gradients.append(prev_gradient)
            new_vector = False
            
            continue
        
        # Compare vector direction based on magnitude
        if abs(prev_gradient - gradient) <= threshold:
            continue
        else:
            # Store the previous vector when a significant change is detected
            # rospy.loginfo(f'Prev vector {prev_gradient}, current vector {gradient} and point: {p1}')
            new_vector = True
            continue
    
    # Store the last gradient after the final iteration
    stored_gradients.append(prev_gradient)
            
    # Make sure we have enough vectors to compute intersection
    if len(stored_gradients) < 2:
        print("Not enough gradients to compute an intersection.")
        return []
    
    # Assuming stored_gradients[0] and stored_gradients[1] are the gradients of the two lines
    m1, m2 = stored_gradients[0], stored_gradients[1]
    
    # Check if gradients are too similar
    if abs(m1 - m2) < 1e-6:
        print("Gradients too similar; lines may be parallel.")
        return None
    
    # Compute the y-intercepts of both lines (b1 and b2)
    b1 = intersection_points[0][1] - m1 * intersection_points[0][0]
    b2 = intersection_points[1][1] - m2 * intersection_points[1][0]
    
    # Calculate the intersection point
    x_intersection = (b2 - b1) / (m1 - m2)
    y_intersection = m1 * x_intersection + b1
    
    return convertLaserTransform(x_intersection, y_intersection)

def find_seam(laser_scan):
    
    # Extract the range data (assuming the ranges are in the 'ranges' field of LaserScan message)
    if laser_scan is None or laser_scan.ranges is None:
        rospy.logerr("find_seam received invalid laser_scan!")
        return

    range_data = np.array(laser_scan.ranges)
    if range_data is None or len(range_data) == 0:
        rospy.logerr("Laser scan data is empty!")
        return
    
    range_data[range_data == float('inf')] = np.nan  # Replace 'inf' with NaN
    
    # range_data = interpolate_nan_with_tolerance(range_data, 3)
    
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
        calculate_pose(laser_scan=laser_scan, scan_index=index)
        
    else:
        rospy.logerr("Change in direction failed. ROS Shutdown...")    
        
    
def calculate_pose(laser_scan, scan_index, centre_point=None, angle_min=None, angle_increment=None):
    
    if scan_index < 0 or scan_index >= len(laser_scan.ranges):
        rospy.logwarn(f"Invalid scan index {scan_index}. Must be between 0 and {len(laser_scan.ranges)-1}.")
        return None
    
    if centre_point is None:
        
        # Get the range (distance) at the given index
        r = laser_scan.ranges[scan_index]

        if r - 0.02 < 0.074:
            r = 0.074
        else:
            r = r - 0.02 #remove small distance to keep a gap for welding
        
        # Calculate the angle corresponding to the chosen scan index
        angle = laser_scan.angle_min + scan_index * laser_scan.angle_increment
        
    else:
        r = centre_point[0]
        scan_index = centre_point[1]
        angle = angle_min + scan_index * angle_increment
    
        
    # If the range is invalid (NaN or infinite), return None
    if r == float('Inf') or r == float('NaN'):
        rospy.logwarn(f"Invalid range value at index {scan_index}.")
        return None
    
    # Convert the polar coordinates (r, angle) to Cartesian coordinates (x, y), calculated: θ=angle_min+scan_index×angle_increment
    x = r * math.sin(angle)
    z = r * math.cos(angle)
    
    rospy.loginfo("calculated x position from laser data: %f ", x)
    rospy.loginfo("calculated y position from laser data: %f ", z)
    rospy.loginfo("calculated angle from laser data: %f ", angle)
    
    convertLaserTransform(x, z, angle)
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

def convertLaserTransform(x, z, angle):
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
        laser_pose.pose.position.y = 0
        laser_pose.pose.position.z = z  # Assuming the laser scanner is on a 2D plane
        
        # Get the robot's current orientation (from base_link)
        robot_transform = tf_buffer.lookup_transform('base_link', 'tcp_eff', rospy.Time(0))  # Assuming 'world' is a fixed reference frame
        robot_orientation = robot_transform.transform.rotation  # Robot's orientation in world frame

        # Apply the transformation
        transformed_pose = do_transform_pose(laser_pose, transform)

        # Use the robot's original orientation
        transformed_pose.pose.orientation = robot_orientation

        # Extract transformed values
        transformed_x = transformed_pose.pose.position.x
        transformed_y = transformed_pose.pose.position.y
        transformed_z = transformed_pose.pose.position.z
        transformed_yaw = tf.transformations.euler_from_quaternion([
            transformed_pose.pose.orientation.x,
            transformed_pose.pose.orientation.y,
            transformed_pose.pose.orientation.z,
            transformed_pose.pose.orientation.w
        ])[2]  # Extract yaw angl
        
        weld_seam_pose_pub.publish(transformed_pose)
        rospy.loginfo(f"Publishing weld seam pose: x={transformed_x}, y={transformed_y}, z={transformed_z}")
        return transformed_x, transformed_y, transformed_yaw

    except tf2_ros.TransformException as e:
        rospy.signal_shutdown(f"Could not get transform: {e}")
        return None, None, None
    
if __name__ == '__main__':
    rospy.init_node('test_example_node', anonymous=True)
    
    vector_detection_method = rospy.get_param('~vector_intersection_method', False)
    scan_topic = rospy.get_param('~scan_topic', '/laser_scan')
    
    if vector_detection_method:
        rospy.loginfo("Using vector intersection method.")
    else:
        rospy.loginfo("Using local maxima and minima method.")
        
    # Subscribe to the /scan topic
    rospy.Subscriber(scan_topic, LaserScan, laser_scan_callback)
    
    # Keep the node running
    rospy.spin()
    
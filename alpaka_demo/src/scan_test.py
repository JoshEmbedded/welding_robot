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
from geometry_msgs.msg import PoseStamped, PoseArray

laser_data_buffer = deque(maxlen=10)  # Store the last 10 messages
new_data = True
laser_data = LaserScan()
global seam_pose

# Store detected seam points
seam_points = []

# Create a publisher (global to avoid creating it repeatedly in the function)
weld_seam_pose_pub = rospy.Publisher('/weld_seam_pose', PoseStamped, queue_size=10)


def laser_scan_callback(msg):
    """
    Callback function to process incoming LaserScan messages.
    """
    global new_data, laser_data
    if new_data:
        laser_data = msg
        find_seam_intersections(laser_data)
        # find_seam(laser_data)
        new_data = False  # Set to False after processing


def find_seam_intersections(laser_scan):
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
    # if r - 0.02 < 0.074:
    #     r = 0.074
    # else:
    #     r = r - 0.02 #remove small distance to keep a gap for welding
    
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
    
    convertLaserTransform(x, y)
    # # Shutdown the node after processing is complete
    # rospy.signal_shutdown("Pose calculation complete.")  # Gracefully shuts down the node

def convertLaserTransform(x, y):
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

        # Offset to ensure gap between end-effector and seam
        offset_distance = 0.02 
        # Calculate the yaw angle representing the direction from the scanner to the point
        yaw = math.atan2(y, x)  # Angle of vector (x, y) from scanner to point
        
        # Apply the offset in the direction of yaw
        x -= offset_distance * math.cos(yaw)
        y -= offset_distance * math.sin(yaw)
        
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        
        # Create a PoseStamped message for the laser scanner pose in its own frame
        laser_pose = geometry_msgs.msg.PoseStamped()
        laser_pose.header.frame_id = 'laser_scanner_link'
        laser_pose.header.stamp = rospy.Time(0)
        laser_pose.pose.position.x = x
        laser_pose.pose.position.y = y
        laser_pose.pose.position.z = 0  # Assuming the laser scanner is on a 2D plane
        
        
        
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
    
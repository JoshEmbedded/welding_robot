#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
from collections import deque
import tf
from geometry_msgs.msg import PointStamped, PoseStamped

laser_data_buffer = deque(maxlen=10)  # Store the last 10 messages

def laser_scan_callback(msg):
    """
    Callback function to process incoming LaserScan messages.
    """
    laser_data_buffer.append(msg)  # Store the latest laser scan message in the buffer
    
def normalize_angle(angle):
    """
    Normalize the angle to be within the range [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def laser_scan_to_cartesian(range, angle):
    # Convert polar coordinates (range, angle) to Cartesian coordinates (x, y)
    x = range * math.cos(angle)
    y = range * math.sin(angle)
    return x, y

def calculate_pose(scan_msg, scan_index):
    
    if scan_index < 0 or scan_index >= len(scan_msg.ranges):
        rospy.logwarn(f"Invalid scan index {scan_index}. Must be between 0 and {len(scan_msg.ranges)-1}.")
        return None
    
    # Get the range (distance) at the given index
    r = scan_msg.ranges[scan_index]
    
    listener = tf.TransformListener()
    
    # Wait for the transformation between the laser and base frames
    try:
        listener.waitForTransform('/base_link', '/laser_sensor_link', rospy.Time(), rospy.Duration(4.0))
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"Error in TF: {e}")  

def calculate_pose_from_scan(scan_msg, scan_index):
    """
    Calculate the pose (x, y) from a selected laser scan index.

    Args:
        scan_msg (LaserScan): The LaserScan message.
        scan_index (int): The index of the chosen scan.

    Returns:
        tuple: (x, y, angle) in meters and radians
    """
    # Check if scan_index is valid
    if scan_index < 0 or scan_index >= len(scan_msg.ranges):
        rospy.logwarn(f"Invalid scan index {scan_index}. Must be between 0 and {len(scan_msg.ranges)-1}.")
        return None

    # Get the range (distance) at the given index
    r = scan_msg.ranges[scan_index]

    # If the range is invalid (NaN or infinite), return None
    if r == float('Inf') or r == float('NaN'):
        rospy.logwarn(f"Invalid range value at index {scan_index}.")
        return None

    # Calculate the angle corresponding to the chosen scan index
    angle = scan_msg.angle_min + scan_index * scan_msg.angle_increment
    
    # Normalize the angle to be within [-pi, pi]
    angle = normalize_angle(angle)

    # Convert the polar coordinates (r, angle) to Cartesian coordinates (x, y), calculated: θ=angle_min+scan_index×angle_increment
    x = r * math.cos(angle)
    y = r * math.sin(angle)

    # Return the pose as a tuple (x, y, angle)
    return (x, y, angle)

    

def laser_scan_listener():
    """
    Initializes the LaserScan subscriber node.
    """
    # Initialize the ROS node
    rospy.init_node('laser_scan_listener', anonymous=True)
    
    # Subscribe to the /scan topic
    rospy.Subscriber('/laser_scan', LaserScan, laser_scan_callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        laser_scan_listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Laser scan listener node terminated.")

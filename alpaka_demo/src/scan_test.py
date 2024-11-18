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

laser_data_buffer = deque(maxlen=10)  # Store the last 10 messages
new_data = True
laser_data = LaserScan()

def laser_scan_callback(msg):
    """
    Callback function to process incoming LaserScan messages.
    """
    global new_data, laser_data  # Declare the global variables to modify them
    if new_data:
        laser_data = msg
        find_seam(laser_data)
        new_data = False  # Set to False after processing

def find_seam(laser_scan):
    
    # Extract the range data (assuming the ranges are in the 'ranges' field of LaserScan message)
    range_data = np.array(laser_scan.ranges)

    range_data[range_data == float('inf')] = np.nan  # Replace 'inf' with NaN
    
     # You can either ignore these values or replace NaNs with the previous valid range.
    range_data = np.nan_to_num(range_data, nan=np.nan)  # This will turn NaNs into a default value like zero
    
    # Print the range data for debugging
    rospy.loginfo(f"Range data: {range_data}")
    
    # Step 2: Find local maxima and minima
    local_maxima = []
    local_minima = []
    edge = False
    index = int()
    for i in range(1, len(range_data) - 1):  # Avoid the first and last elements
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
        rospy.loginfo("Change in direction detected: %s", edge)
        calculate_pose(laser_scan, index)
        
    else:
        rospy.logerr("Change in direction failed. ROS Shutdown...")
        # Shutdown the node after processing is complete
        rospy.signal_shutdown("Seam detection failed.")
    
        
    
def calculate_pose(laser_scan, scan_index):
    
    if scan_index < 0 or scan_index >= len(laser_scan.ranges):
        rospy.logwarn(f"Invalid scan index {scan_index}. Must be between 0 and {len(laser_scan.ranges)-1}.")
        return None
    
    # Get the range (distance) at the given index
    r = laser_scan.ranges[scan_index]
    
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
    
    # Shutdown the node after processing is complete
    rospy.signal_shutdown("Pose calculation complete.")  # Gracefully shuts down the node

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
        rospy.logwarn("Could not get transform: %s", e)
        

if __name__ == '__main__':
    rospy.init_node('test_example_node', anonymous=True)
    
    getTransform()
    # Subscribe to the /scan topic
    rospy.Subscriber('/laser_scan', LaserScan, laser_scan_callback)
    
    
    # Keep the node running
    rospy.spin()
    
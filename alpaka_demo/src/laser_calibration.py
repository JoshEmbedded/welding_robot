#!/usr/bin/env python3
import rospy
import rosbag
from sensor_msgs.msg import LaserScan
from alpaka_demo.srv import ProcessBag, ProcessBagResponse
import numpy as np
from rospy import Time
import math


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

def convertData(laser_scan):
    # Extract the range data (assuming the ranges are in the 'ranges' field of LaserScan message)
    
    range_data = np.array(laser_scan.ranges)
    
    range_data[range_data == float('inf')] = np.nan  # Replace 'inf' with NaN
    
    range_data = interpolate_nan_with_tolerance(range_data, 3)
    
    converted_data = []
    for i in range(len(range_data)):
        converted_data.append((range_data[i],i,laser_scan.header.stamp))
    
    # Convert the list to a NumPy array (if needed)
    converted_data_array = np.array(converted_data)
    return converted_data_array

def find_seam(data):
    
    range_data = [item[0] for item in data]
    
    edge = False
    index = -1
    for i in range(1, len(range_data) - 1):  # Avoid the first and last elements
        # Skip if the current or neighboring values are NaN
        if np.isnan(range_data[i-1]) or np.isnan(range_data[i]) or np.isnan(range_data[i+1]):
            continue  # Skip this iteration
        
        if range_data[i-1] < range_data[i] > range_data[i+1]:  # Local maxima
            # local_maxima.append(i)
            # rospy.loginfo(f"local maxima found at indices: {i}, with range: {range_data[i]}.")
            index = i
            edge = True
        # elif range_data[i-1] > range_data[i] < range_data[i+1]:  # Local minima
        #     local_minima.append(i)
        #     # rospy.loginfo(f"local minima found at indices: {i}, with range: {range_data[i]}.")
        #     index = i
        #     edge = True
    
    if edge:
        return data[index]
        
def find_seam_from_bag(bag_path):
    try:
        # Open the rosbag
        
        bag = rosbag.Bag(bag_path)
        rospy.loginfo(f"Opened rosbag: {bag_path}")

        direction_change = []
        # Process LaserScan messages
        for topic, msg, t in bag.read_messages(topics=['laser_scan']):
            converted_data = convertData(msg)
            if find_seam(converted_data) is None:
                continue
            else:
                direction_change.append(find_seam(converted_data));
        
        vulcram_centre = find_seam(direction_change)
        bag.close()
        return vulcram_centre;
    except Exception as e:
        rospy.logerr(f"Error processing rosbag: {e}")
        return f"Error: {str(e)}"
    
def find_approx_joint_state(bag_path, timestamp):
    
    # Initialize variables
    closest_joint_state = None
    smallest_time_diff = float('inf')

    try:
        # Open the rosbag
        bag = rosbag.Bag(bag_path)
        rospy.loginfo(f"Opened rosbag: {bag_path}")
        # Process LaserScan messages
        for topic, msg, t in bag.read_messages(topics=['joint_states']):
            time_diff = abs((t-timestamp).to_sec())
            
            # Check if this is the closest message so far
            if time_diff < smallest_time_diff:
                closest_joint_state = msg
                smallest_time_diff = time_diff
        bag.close()        
    except Exception as e:
        rospy.logerr(f"Error processing rosbag: {e}")
        return f"Error: {str(e)}"
    
    return closest_joint_state

def calculate_pose(centre_point=None, angle_min=None, angle_increment=None):
    
    r = centre_point[0]
    scan_index = centre_point[1]
    angle = angle_min + scan_index * angle_increment
    
        
    # If the range is invalid (NaN or infinite), return None
    if r == float('Inf') or r == float('NaN'):
        rospy.logwarn(f"Invalid range value at index {scan_index}.")
        return None
    
    # Convert the polar coordinates (r, angle) to Cartesian coordinates (x, y), calculated: θ=angle_min+scan_index×angle_increment
    x = r * math.cos(angle)
    y = r * math.sin(angle)
    
    rospy.loginfo("calculated x position from laser data: %f ", x)
    rospy.loginfo("calculated y position from laser data: %f ", y)
    rospy.loginfo("calculated angle from laser data: %f ", angle)
    
    return x,y,angle

def get_pose(bag_path, centre_point):
    
    angle_min = None
    angle_increment = None
    
    try:
        # Open the rosbag
        bag = rosbag.Bag(bag_path)
        rospy.loginfo(f"Opened rosbag: {bag_path}")
        # Process LaserScan messages
        for topic, msg, t in bag.read_messages(topics=['laser_scan']):
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment    
            break
        bag.close()
        return calculate_pose(centre_point=centre_point, angle_min=angle_min, angle_increment=angle_increment)
           
    except Exception as e:
        rospy.logerr(f"Error processing rosbag: {e}")
        return f"Error: {str(e)}"
        
def handle_process_bag(req):
    rospy.loginfo(f"Received request to process rosbag: {req.bag_path}")
    vulcram_centre_scan = find_seam_from_bag(req.bag_path)
    
    x,y,angle = get_pose(req.bag_path, vulcram_centre_scan)
    
    scan_time_stamp = vulcram_centre_scan[2]
    scan_joint_state = find_approx_joint_state(req.bag_path, scan_time_stamp)
    
    if scan_joint_state is None:
        rospy.logerr("Failed to process rosbag properly.")
        return ProcessBagResponse(joint_state = None, analysed = False, x=None, y=None, angle=None)
    
    return ProcessBagResponse(joint_state = scan_joint_state, analysed = True, x=x, y=y, angle=angle)

def process_bag_server():
    rospy.init_node('bag_calibration_service')
    service = rospy.Service('process_bag', ProcessBag, handle_process_bag)
    rospy.loginfo("Ready to process rosbag files.")
    rospy.spin()

if __name__ == '__main__':
    process_bag_server()    
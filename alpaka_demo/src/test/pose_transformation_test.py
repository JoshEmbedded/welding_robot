import spatialmath as sm
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Pose

# Create a publisher (global to avoid creating it repeatedly in the function)
calculated_point_pub = rospy.Publisher('/pose_viz', Pose, queue_size=10)

if __name__ == '__main__':
    
    rospy.init_node('calculation_node', anonymous=True)
    
    # Input robot pose
    robot_x = -0.1375
    robot_y =0.9593
    robot_z = 0.4248
    
    robot_position = [robot_x, robot_y, robot_z]
    
    # Input transformation quaternion from transformcalCalculation script.
    sensor_to_tcp_rotation = [0.03166, -0.0893, 0.0126, 0.9954]
    tcp_offset_x = -0.1151
    tcp_offset_y = 0
    tcp_offset_z = -0.07236
    sensor_to_tcp_translation = [tcp_offset_x, tcp_offset_y, tcp_offset_z]
                                 
    # Input data from sensor reading
    reading_x = 0.1506
    reading_y = -0.0025
    reading_z = 0
    point_scanned = [reading_x, reading_y, reading_z]
    
    # Create transformation matrix of Sensor to TCP
    R1 = sm.SE3(sensor_to_tcp_translation) * sm.base.r2t(sm.base.q2r(sensor_to_tcp_rotation, 'xyzs'))
    
    # Point Transformation to TCP frame
    transformed_point = sm.base.homtrans(R1, point_scanned)
    
    point_difference = [robot_position[0]+transformed_point[0], robot_position[1]+transformed_point[1],robot_position[2]+transformed_point[2]]
    print(point_difference)
    
    pose = Pose()
    pose.position.x = transformed_point[0]
    pose.position.y = transformed_point[1]
    pose.position.z = transformed_point[2]
    pose.orientation.w = 1
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    
    calculated_point_pub.publish(pose)
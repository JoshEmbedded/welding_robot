#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped

def move_robot():
    rospy.init_node("servo_test", anonymous=True)
    
    # Publisher to the MoveIt Servo Cartesian command topic
    twist_pub = rospy.Publisher("/servo_server/delta_twist_cmds", TwistStamped, queue_size=10)
    
    rospy.sleep(1)  # Allow some time for the publisher to initialize

    # Create a TwistStamped message
    twist_msg = TwistStamped()
    twist_msg.header.frame_id = "base_link"  # The reference frame for the movement
    twist_msg.twist.linear.x = 0.1  # Move along the X-axis at 0.1 m/s
    twist_msg.twist.linear.y = 0.0
    twist_msg.twist.linear.z = 0.0
    twist_msg.twist.angular.x = 0.0
    twist_msg.twist.angular.y = 0.0
    twist_msg.twist.angular.z = 0.0

    rospy.loginfo("Publishing velocity command to move the robot along the X-axis...")

    # Publish the command for 1 second to move the robot 0.1 m
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now()

    while rospy.Time.now() - start_time < rospy.Duration(1.0):  # 1-second duration
        twist_pub.publish(twist_msg)
        rate.sleep()

    rospy.loginfo("Finished sending commands. The robot should have moved 0.1 m along the X-axis.")

if __name__ == "__main__":
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

def move_to_pose(move_group,target_pose):
    
    # Print basic information about the robot
    rospy.loginfo("Reference frame: %s", move_group.get_planning_frame())
    rospy.loginfo("End-effector link: %s", move_group.get_end_effector_link())
    rospy.loginfo("Robot groups: %s", moveit_commander.RobotCommander().get_group_names())

    # Set the target pose
    move_group.set_pose_target(target_pose)

    plan = move_group.go(wait=True)
    # # Plan and execute the motion
    # plan, success = move_group.plan()
    # if success:
    #     rospy.loginfo("Planning succeeded.")
    #     move_group.execute(plan, wait=True)
    # else:
        
    #     rospy.logerr("Planning failed.")
    #     return

    # Ensure no residual movement commands are sent
    move_group.stop()
    
    # Clear targets after execution
    move_group.clear_pose_targets()

    if plan:
        rospy.loginfo("Movement to target pose succeeded!")
    else:
        rospy.logerr("Movement to target pose failed!")

    # Shutdown MoveIt
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    # Define the target pose
    # Create a publisher (global to avoid creating it repeatedly in the function)
    rospy.init_node('move_robot_to_target', anonymous=True)
    # Initialize the MoveIt Commander and a ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    
    
    # Instantiate a MoveGroupCommander object for the manipulator
    group_name = "manipulator"  # Replace with your planning group name
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_workspace([-2.0, -2.0, 0.0, 2.0, 2.0, 2.0])  # [min_x, min_y, min_z, max_x, max_y, max_z]
    
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'world'
    target_pose.header.stamp = rospy.Time(0)

    # target_pose = Pose()
    # target_pose.position.x = -0.92539  # Replace with your target X coordinate
    # target_pose.position.y = -0.09219  # Replace with your target Y coordinate
    # target_pose.position.z = 0.83867  # Replace with your target Z coordinate
    # target_pose.orientation.x = 0.103535  # Replace with your target quaternion X
    # target_pose.orientation.y = 0.2558128  # Replace with your target quaternion Y
    # target_pose.orientation.z = 0.0167123  # Replace with your target quaternion Z
    # target_pose.orientation.w = 0.9610208  # Replace with your target quaternion W
    
    target_pose.pose.position.x = 0  # Replace with your target X coordinate
    target_pose.pose.position.y = 1  # Replace with your target Y coordinate
    target_pose.pose.position.z = 0.2  # Replace with your target Z coordinate
    target_pose.pose.orientation.x = 0  # Replace with your target quaternion X
    target_pose.pose.orientation.y = 0 # Replace with your target quaternion Y
    target_pose.pose.orientation.z = 0.7068252  # Replace with your target quaternion Z
    target_pose.pose.orientation.w = 0.7073883  # Replace with your target quaternion W

    # Publish the pose (for visualization or other processing)
    pose_publisher = rospy.Publisher('/weld_seam_pose', PoseStamped, queue_size=10)
    rospy.sleep(1)  # Allow time for publisher registration
    pose_publisher.publish(target_pose)
    rospy.loginfo("Published target pose to /weld_seam_pose.")

    rospy.spin()
    # try:
    #     move_to_pose(move_group, target_pose)
    # except rospy.ROSInterruptException:
    #     rospy.logerr("ROS node interrupted.")
    # except KeyboardInterrupt:
    #     rospy.loginfo("Script terminated by user.")
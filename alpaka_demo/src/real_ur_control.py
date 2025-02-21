#!/usr/bin/env python

import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Header

class WeldSeamFollower:
    def __init__(self):
        rospy.init_node('weld_seam_follower', anonymous=True)

        # Initialize MoveIt
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")

        # Action client for sending trajectory to UR ROS driver
        self.client = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()

        # Subscriber for weld seam pose
        self.pose_sub = rospy.Subscriber("/weld_seam_pose", geometry_msgs.msg.PoseStamped, self.pose_callback)
        rospy.loginfo("Subscribed to /weld_seam_pose topic")

    def pose_callback(self, pose_msg):
        rospy.loginfo("Received new weld seam pose, planning trajectory...")

        # Set target pose
        self.group.set_pose_target(pose_msg.pose)

        # Plan trajectory
        plan = self.group.plan()
        if not plan[0]:
            rospy.logerr("Failed to compute a valid trajectory")
            return
        
        trajectory = plan[1]  # Extract trajectory from MoveIt plan result
        self.execute_trajectory(trajectory)

    def execute_trajectory(self, trajectory):
        rospy.loginfo("Executing planned trajectory on real robot...")

        # Convert MoveIt trajectory to FollowJointTrajectoryGoal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory.joint_trajectory

        # Send trajectory to the robot
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("Trajectory execution completed.")

if __name__ == '__main__':
    try:
        weld_seam_follower = WeldSeamFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted")

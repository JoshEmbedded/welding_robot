#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

class WeldSeamTracker:
    def __init__(self):
        rospy.init_node("weld_seam_tracker", anonymous=True)
        
        # Initialize MoveIt Commander
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")

        # Pose storage
        self.start_pose = None
        self.end_pose = None
        self.weld_seam_poses = []  # List of weld seam points
        
        # Movement Boolean
        self.track = False

        # Subscriber for weld seam detection
        rospy.Subscriber("/weld_seam_pose", PoseStamped, self.weld_seam_callback)
        
        self.record_poses()

    def get_current_pose(self):
        """Gets the current end-effector pose."""
        return self.group.get_current_pose().pose

    def weld_seam_callback(self, msg):
        """Stores detected weld seam poses."""
        self.weld_seam_poses.append(msg.pose)
        rospy.loginfo(f"Stored weld seam pose: {msg.pose}")
        rospy.sleep(0.1)  # Allow some time for weld seam data to be collected
        tracker.execute_path()  # Plan and execute Cartesian motion

    def record_poses(self):
        """Records start and end poses on user input."""
        input("Press Enter to record START pose...")
        self.start_pose = self.get_current_pose()
        print(f"Start Pose Recorded: {self.start_pose}")

        input("Press Enter to record END pose...")
        self.end_pose = self.get_current_pose()
        print(f"End Pose Recorded: {self.end_pose}")

    def plan_cartesian_path(self):
        """Creates a Cartesian path following the weld seam."""
        waypoints = [self.group.get_current_pose]  # Start with the recorded start pose

        # Add all weld seam points
        waypoints.extend(self.weld_seam_poses)

        waypoints.append(self.end_pose)  # End with the recorded end pose

        # Plan Cartesian path
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,   # Waypoints for Cartesian path
            0.01,        # Step size (adjust for smoother paths)
            0.0          # Jump threshold (0 disables jump detection)
        )

        if fraction < 1.0:
            rospy.logwarn(f"Only {fraction*100:.2f}% of the Cartesian path was planned.")

        return plan

    def execute_path(self):
        """Executes the computed Cartesian path."""
        self.weld_seam_poses.clear()
        plan = self.plan_cartesian_path()
        if plan:
            self.group.execute(plan, wait=True)
            rospy.loginfo("Cartesian path execution complete.")
        else:
            rospy.logwarn("No valid Cartesian path was planned.")

    def move_start(self):
    
        self.group.set_pose_target(self.start_pose)
        movement = self.group.go()
        if movement:
            rospy.loginfo("Successful movement to start position.")
        else:
            rospy.logwarn("Unsuccessful movement to start position.")
        return movement
    
        
if __name__ == "__main__":
    tracker = WeldSeamTracker()
    ready = tracker.move_start()
    if ready:
        rospy.loginfo("Robot is ready to track seam.")
        input("Press Enter to begin movement...")
        tracker.track = True
    else:
        rospy.logerr("Robot movement has failed...")

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def move_to_pose():
    # Initialize moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_pose_execution', anonymous=True)
    
    # Instantiate a MoveGroupCommander for the manipulator
    
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    move_group.set_max_velocity_scaling_factor(0.3)
    
    # Define the target pose
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.5
    target_pose.position.y = 0
    target_pose.position.z = 0.5
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 1.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 0.0
    
    # Set the pose target
    move_group.set_pose_target(target_pose)
    
    # Plan the trajectory to the goal
    plan = move_group.go(wait=True)
    
    # Ensure no residual movement
    move_group.stop()
    move_group.clear_pose_targets()
    
    print("Pose executed successfully.")
    
    # Shutdown MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    move_to_pose()

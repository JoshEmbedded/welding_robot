import sys  # Add this import to resolve the NameError
import rospy
import moveit_commander
import tf
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Pose
import math
import tf.transformations as tft
from laser_geometry import LaserProjection
import numpy as np
import open3d as o3d
import pickle
import copy
import rosbag
from tf2_msgs.msg import TFMessage
import roslib; roslib.load_manifest('laser_assembler')
from laser_assembler.srv import *
import sensor_msgs.point_cloud2 as pc2


class RobotMovement:
    def __init__(self):
        # Initialize the MoveIt commander and ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_movement')

        # Initialize robot, scene, and group (manipulator in this case)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_goal_position_tolerance(0.0005)
        
        # Initialize the TF listener for flange position
        self.listener = tf.TransformListener()
        
        # Define the rosbag file
        self.bag = rosbag.Bag("tf_laserscan_data.bag", "w")

        # Set up the laser scan subscriber
        self.laser_sub = rospy.Subscriber("/laser_scan", LaserScan, self.laser_scan_callback, tcp_nodelay=True)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_callback, tcp_nodelay=True)
        self.cloud_pub = rospy.Publisher('assembled_cloud', PointCloud2, queue_size=10, latch=True)

        # To store the latest laser scan and flange position
        self.latest_scan = None
        self.latest_tf = None
        self.latest_flange_position = None
        self.latest_flange_rotation = None
        self.scanning = False
        self.data_storage = []
        self.rate = rospy.Rate(15)  # 10 Hz
        self.counter = 0
        self.projector = LaserProjection()     
        self.errors = []  # To store residual errors for each scan
        
    
    def move_robot(self, pose, blocking=True):
        # Set the target pose for the robot
        self.group.set_pose_target(pose)
        
        self.target_pose = pose

        # Start the movement asynchronously (non-blocking)
        self.group.go(wait=blocking)  # The movement will start, but the program won't block here
        
        self.group.clear_pose_targets()

        rospy.loginfo("Movement initiated!")

    def stop_robot(self):
        # Stop the robot if needed
        self.group.stop()
        
    def tf_callback(self, msg):
        """ Callback to store TF data in the rosbag """
        self.latest_tf = msg

    def laser_scan_callback(self, msg):
        """ Callback to store LaserScan data in the rosbag """
        self.latest_scan = msg
        
        # if self.scanning:
        #     self.record_data()

    def get_flange_position(self):
        try:
            # Wait for the transform to be available
            self.listener.waitForTransform('/world', '/flange', rospy.Time(0), rospy.Duration(1.0))
            # Get the current position of the flange
            (robot_trans, robot_rot) = self.listener.lookupTransform('/world', '/flange', rospy.Time(0))
            self.latest_flange_position = robot_trans
            self.latest_flange_rotation = robot_rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Could not get flange position: {e}")
            
    def record_data(self):
        
        rospy.loginfo("Recording data...")
        try:
            self.bag.write("/scan", self.latest_scan, rospy.Time.now())
            self.bag.write("/tf_data", self.latest_tf, rospy.Time.now())
        except Exception as e:
            rospy.logerr(f"Error writing data to rosbag: {e}")
            
            
    def is_robot_moving(self):
        """
        Checks if the robot is still moving by comparing the current pose to the target pose.
        Returns:
            bool: True if the robot is moving, False otherwise.
        """
        # Get the current pose of the end-effector
        current_pose = self.group.get_current_pose().pose

        # Ensure the target pose is available
        if not hasattr(self, 'target_pose') or self.target_pose is None:
            rospy.logwarn("No target pose set. Cannot check if the robot is moving.")
            return False

        # Define tolerances for position (meters) and orientation (quaternion difference)
        position_tolerance = 0.005  # 0.5 cm
        orientation_tolerance = 0.005  # 0.5%

        # Compare positions
        position_close = (
            abs(current_pose.position.x - self.target_pose.position.x) <= position_tolerance and
            abs(current_pose.position.y - self.target_pose.position.y) <= position_tolerance and
            abs(current_pose.position.z - self.target_pose.position.z) <= position_tolerance
        )

        # Compare orientations (using quaternion difference)
        orientation_close = (
            abs(current_pose.orientation.x - self.target_pose.orientation.x) <= orientation_tolerance and
            abs(current_pose.orientation.y - self.target_pose.orientation.y) <= orientation_tolerance and
            abs(current_pose.orientation.z - self.target_pose.orientation.z) <= orientation_tolerance and
            abs(current_pose.orientation.w - self.target_pose.orientation.w) <= orientation_tolerance
        )

        return not (position_close and orientation_close)
    
    def offset_movement(self, pose, X, Y, Z, roll_offset, pitch_offset, yaw_offset):
        """
        Offsets the position and orientation of a pose by specified amounts.

        Args:
            pose (geometry_msgs.Pose): The original pose to modify.
            X (float): The offset to apply to the X position.
            Y (float): The offset to apply to the Y position.
            Z (float): The offset to apply to the Z position.
            roll_offset (float): The offset to apply to the roll angle (in radians).
            pitch_offset (float): The offset to apply to the pitch angle (in radians).
            yaw_offset (float): The offset to apply to the yaw angle (in radians).

        Returns:
            geometry_msgs.Pose: The modified pose with offsets applied.
        """

        # Apply position offsets
        new_pose = Pose()
        new_pose.position.x = pose.position.x + X
        new_pose.position.y = pose.position.y + Y
        new_pose.position.z = pose.position.z + Z

        # Extract current RPY angles from the quaternion
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        roll, pitch, yaw = tft.euler_from_quaternion(quaternion)

        # Apply the offset to the RPY angles
        roll += roll_offset
        pitch += pitch_offset
        yaw += yaw_offset

        # Convert the updated RPY angles back to a quaternion
        updated_quaternion = tft.quaternion_from_euler(roll, pitch, yaw)

        # Update the pose's orientation with the new quaternion
        new_pose.orientation.x = updated_quaternion[0]
        new_pose.orientation.y = updated_quaternion[1]
        new_pose.orientation.z = updated_quaternion[2]
        new_pose.orientation.w = updated_quaternion[3]

        return new_pose

def pointCloud_assemble(start_time, end_time):
    resp = None
    try:
            rospy.wait_for_service('assemble_scans2')  # Ensure service is available
            assemble_scans2 = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
            resp = assemble_scans2(start_time, end_time)
            if len(resp.cloud.data) > 0:   
                rospy.loginfo("Got cloud with %u points", len(resp.cloud.data) // resp.cloud.point_step)
                
                print(f"Header: {resp.cloud.header}")
                print(f"Width: {resp.cloud.width}, Height: {resp.cloud.height}")
                print(f"Point Step: {resp.cloud.point_step}")
                print(f"Row Step: {resp.cloud.row_step}")
                print(f"Data Size: {len(resp.cloud.data)}")
                robot_movement.cloud_pub.publish(resp.cloud)  # Directly publish PointCloud2
    
            else:
                rospy.logwarn("No points in assembled cloud! Check laser scan topic and TF frames.")
                
    except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            
    return resp
            
def merge_pointclouds(cloud1, cloud2):
    """ Merges two PointCloud2 messages into one while preserving all fields. """

    # Extract all field names dynamically
    field_names = [f.name for f in cloud1.fields]

    # Extract points from both clouds
    points1 = list(pc2.read_points(cloud1, field_names=field_names, skip_nans=True))
    points2 = list(pc2.read_points(cloud2, field_names=field_names, skip_nans=True))

    # Merge the point lists
    merged_points = points1 + points2

    # Create a new PointCloud2 message
    merged_cloud = pc2.create_cloud(cloud1.header, cloud1.fields, merged_points)
    merged_cloud.header.stamp = rospy.Time.now()  # Update timestamp

    return merged_cloud

if __name__ == '__main__':
    try:
        # Create an instance of RobotMovement
        robot_movement = RobotMovement()
        
        # orientations = []
        # square_pose_list = []
        

        # Create a target pose for the robot (example)
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.35
        pose.position.z = 0.055
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, 0, -1.57)
        # Move robot asynchronously (non-blocking)
        robot_movement.move_robot(pose, True)
        robot_movement.group.set_max_velocity_scaling_factor(0.05)
        robot_movement.group.set_max_acceleration_scaling_factor(0.05)
        
        first_movement = robot_movement.offset_movement(pose, 0, 0.10, 0, 0, 0, 0)
        
        second_movement = robot_movement.offset_movement(first_movement, 0, 0, 0, 0, 0, 3.14)
        third_movement = robot_movement.offset_movement(second_movement, 0, -0.2, 0, 0, 0, 0)
        robot_movement.scanning = True
        
        
        start_time = rospy.get_rostime()
        # Initial testing of movement and bag
        robot_movement.move_robot(first_movement, True)
        end_time = rospy.get_rostime()
        robot_movement.scanning = False
        
        resp1 = pointCloud_assemble(start_time, end_time)
        
        robot_movement.move_robot(second_movement, True)
        
        start_time = rospy.get_rostime()
        robot_movement.scanning = True
        robot_movement.move_robot(third_movement, True)
        robot_movement.scanning = False
        end_time = rospy.get_rostime()
        
        resp2 = pointCloud_assemble(start_time, end_time)
        
        if len(resp1.cloud.data) > 0 and len(resp2.cloud.data) > 0:
            rospy.loginfo("Merging two point clouds...")

            # Merge the two clouds
            merged_cloud = merge_pointclouds(resp1.cloud, resp2.cloud)

            # Publish the merged cloud
            rospy.loginfo(f"Publishing merged PointCloud2 with {len(merged_cloud.data)} bytes of data.")
            robot_movement.cloud_pub.publish(merged_cloud)
            rospy.loginfo("Published merged PointCloud2 successfully.")

        else:
            rospy.logwarn("One of the clouds is empty! Check laser scan topic and TF frames.")

        
        rospy.sleep(0.1)
        
        rospy.loginfo("Closing rosbag file.")
        robot_movement.bag.close()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
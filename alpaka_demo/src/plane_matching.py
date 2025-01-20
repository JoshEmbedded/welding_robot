import sys  # Add this import to resolve the NameError
import rospy
import moveit_commander
import tf
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
import math
import tf.transformations as tft
import tf2_ros
import tf2_geometry_msgs
from laser_geometry import LaserProjection

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

        # Set up the laser scan subscriber
        rospy.Subscriber("/laser_scan", LaserScan, self.laser_scan_callback, tcp_nodelay=True)
        self.cloud_pub = rospy.Publisher('cloud_publisher', PointCloud2, queue_size=10)

        # To store the latest laser scan and flange position
        self.latest_scan = None
        self.latest_flange_position = None
        self.latest_flange_rotation = None
        self.scanning = False
        self.data_storage = []
        self.rate = rospy.Rate(15)  # 10 Hz
        self.counter = 0
        self.projector = LaserProjection()

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

    def laser_scan_callback(self, msg):
        # Store the latest laser scan data
        self.latest_scan = msg
        self.cloud_msg = self.projector.projectLaser(msg)
        if self.scanning:
            self.record_data()
            self.counter +=1
            self.rate.sleep()
        # rospy.loginfo("Received laser scan data")

    def get_flange_position(self):
        try:
            # Wait for the transform to be available
            self.listener.waitForTransform('/world', '/flange', rospy.Time(0), rospy.Duration(1.0))
            # Get the current position of the flange
            (trans, rot) = self.listener.lookupTransform('/world', '/flange', rospy.Time(0))
            self.latest_flange_position = trans
            self.latest_flange_rotation = rot
            # rospy.loginfo(f"Received flange position: {trans}")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Could not get flange position: {e}")

    def record_data(self):
        # Store the laser scan and flange position as a dictionary
        if self.latest_scan:
            self.get_flange_position()
            if self.latest_scan and self.latest_flange_position:
                scan_and_flange_data = {
                    'laser_scan': self.latest_scan.ranges,
                    'point_cloud': self.cloud_msg,
                    'trans': self.latest_flange_position,
                    'rot': self.latest_flange_rotation
                }
                self.data_storage.append(scan_and_flange_data)
                self.cloud_pub.publish(self.cloud_msg)
                
                # rospy.loginfo(f"Recorded laser scan and flange position.")
            else:
                rospy.logwarn("Missing laser scan data or flange position")
            
            
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
    
    def process_laser_scan_data(self, data_list):
        """
        Process a list of dictionaries containing laser scan data.
        Replaces 'inf' values with NaN and calculates the max value for each laser_scan list.
        
        Args:
            data_list (list): List of dictionaries containing laser scan data.
            
        Returns:
            list: Updated list of dictionaries with replaced 'inf' and added 'max_value'.
        """
        # Iterate through the list of dictionaries
        for data in data_list:
            # Check if 'laser_scan' exists in the dictionary
            if 'laser_scan' in data:
                # Replace all inf values with NaN
                data['laser_scan'] = [math.nan if x == float('inf') else x for x in data['laser_scan']]
                
                # Calculate the maximum value from the laser_scan list (ignoring NaN values)
                data['max_value'] = max([x for x in data['laser_scan'] if not math.isnan(x)], default=float('-inf'))

            # Calculate the average of the laser_scan list (ignoring NaN values)
            valid_scans = [x for x in data['laser_scan'] if not math.isnan(x)]
            data['average_scan'] = sum(valid_scans) / len(valid_scans) if valid_scans else float('nan')
            
            # Get the middle_scan value at index 48 (if it exists)
            if len(data['laser_scan']) > 48:
                data['middle_scan'] = data['laser_scan'][48]
            else:
                data['middle_scan'] = float('nan')  # Set to NaN if the list is too short

        return data_list
    
    def find_groove(self, data_list, tolerance=0.0005):
        """
        Find local maxima where the average_scan value is larger than both the previous
        and next average_scan values, within a specified tolerance to avoid noise.
        
        Args:
            data_list (list): List of dictionaries containing 'average_scan' values.
            tolerance (float): Minimum difference required between average_scan values to detect a peak.
            
        Returns:
            list: List of indices where local maxima are detected.
        """
        local_maxima = []
        
        # Iterate through the list (ignoring the first and last entries)
        for i in range(1, len(data_list) - 1):
            previous_avg = data_list[i - 1].get('average_scan', float('nan'))
            current_avg = data_list[i].get('average_scan', float('nan'))
            next_avg = data_list[i + 1].get('average_scan', float('nan'))
            
            # Ensure current_avg is valid and compare with previous and next values
            if not (math.isnan(previous_avg) or math.isnan(current_avg) or math.isnan(next_avg)):
                # Check if the current value is larger than both the previous and next values (local maxima)
                if (current_avg > previous_avg + tolerance) and (current_avg > next_avg + tolerance):
                    local_maxima.append(i)
        return local_maxima
    
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
    
    def find_highest_max_average(self, data_list):
        """
        Find the index of the dictionary with the highest 'average_scan' value.
        
        Args:
            data_list (list): List of dictionaries containing 'average_scan' values.
            
        Returns:
            tuple: Index of the dictionary with the highest 'average_scan' value
                and the dictionary itself, or (-1, None) if the list is empty.
        """
        if not data_list:
            return -1, None  # Return -1 if the list is empty
        
        highest_max_index = 0
        highest_max_value = data_list[0].get('middle_scan', float('-inf'))  # Start with the first element
        
        # Iterate through the list to find the highest 'average_scan'
        for i, data in enumerate(data_list):
            current_avg = data.get('middle_scan', float('-inf'))
            
            # Update if the current 'average_scan' is higher than the previous max
            if current_avg > highest_max_value:
                highest_max_value = current_avg
                highest_max_index = i
        
        return highest_max_index

if __name__ == '__main__':
    try:
        # Create an instance of RobotMovement
        robot_movement = RobotMovement()
        
        orientations = []
        square_pose_list = []
        

        # Create a target pose for the robot (example)
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.35
        pose.position.z = 0.03
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, 0, -1.57)
        # Move robot asynchronously (non-blocking)
        robot_movement.move_robot(pose, True)
        robot_movement.group.set_max_velocity_scaling_factor(0.1)
        robot_movement.group.set_max_acceleration_scaling_factor(0.1)
        
        
        rate = rospy.Rate(10)  # 10 Hz
        
        orientations.append(pose)
        x_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, -0.5235, 0, 0) #rotation around x
        orientations.append(x_rot_pose)
        y_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, 0.5235, 0) #rotation around y
        orientations.append(y_rot_pose)
        z_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, 0, 0.5235) #rotation around z
        orientations.append(z_rot_pose)
        
        for orient in orientations:
            square_pose = []
            square_pose.append(orient)
            move_1 = robot_movement.offset_movement(orient, 0, 0.1, 0, 0, 0, 0)
            square_pose.append(move_1)
            move_2 = robot_movement.offset_movement(move_1, 0.1, 0, 0, 0, 0, 0)
            square_pose.append(move_2)
            move_3 = robot_movement.offset_movement(move_2, 0, -0.1, 0, 0, 0, 0)
            square_pose.append(move_3)
            move_4 = robot_movement.offset_movement(move_3, -0.1, 0, 0, 0, 0, 0)
            square_pose.append(move_4)
            square_pose_list.append(square_pose)
        
        scan_data = []
        
        for i, square_pose in enumerate(square_pose_list):
            if i == 2:
                break
            robot_movement.scanning = False
            print(f"Scan Counter: {robot_movement.counter}")
            robot_movement.counter = 0
            print(f"Processing square poses for orientation {i+1}:")
            scan_data.append(robot_movement.data_storage)
            robot_movement.data_storage.clear()
            # Iterate over each pose in the current square pose list
            for j, move in enumerate(square_pose):
                if j != 0:
                    robot_movement.scanning = True  
                print(f"Pose {j+1}: {move}")
                robot_movement.move_robot(move, True)
        robot_movement.scanning = False
        scan_data.append(robot_movement.data_storage)
        print(f"cloud data: {scan_data[0][0].get('point_cloud', None)}")
            
    except rospy.ROSInterruptException:
        pass

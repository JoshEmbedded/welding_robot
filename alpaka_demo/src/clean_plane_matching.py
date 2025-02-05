import sys
import rospy
import moveit_commander
import tf
import tf.transformations as tft
import numpy as np
import open3d as o3d
import pickle
import copy
import math
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from laser_geometry import LaserProjection
from scipy.optimize import minimize
from sklearn.linear_model import RANSACRegressor
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class RobotMovement:
    def __init__(self):
        """Initialize the ROS MoveIt commander and robot interfaces."""
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_movement')

        # MoveIt setup
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_goal_position_tolerance(0.0005)

        # TF listener for flange position
        self.listener = tf.TransformListener()

        # Laser scan setup
        rospy.Subscriber("/laser_scan", LaserScan, self.laser_scan_callback, tcp_nodelay=True)
        self.cloud_pub = rospy.Publisher('cloud_publisher', PointCloud2, queue_size=10)
        self.projector = LaserProjection()

        # Data storage
        self.latest_scan = None
        self.latest_flange_position = None
        self.latest_flange_rotation = None
        self.scanning = False
        self.data_storage = []
        self.rate = rospy.Rate(5)
        self.counter = 0
        self.plane_parameters = []
        self.errors = []

    # ======================== ROBOT MOVEMENT ======================== #
    
    def move_robot(self, pose, blocking=True):
        """Move the robot to the specified pose."""
        self.group.set_pose_target(pose)
        self.target_pose = pose
        self.group.go(wait=blocking)
        self.group.clear_pose_targets()
        rospy.loginfo("Movement initiated.")

    def stop_robot(self):
        """Stop robot movement."""
        self.group.stop()

    def is_robot_moving(self):
        """Check if the robot is still moving based on its target pose."""
        current_pose = self.group.get_current_pose().pose
        if not hasattr(self, 'target_pose') or self.target_pose is None:
            rospy.logwarn("No target pose set. Cannot check if the robot is moving.")
            return False

        position_tolerance = 0.005
        orientation_tolerance = 0.005

        position_close = (
            abs(current_pose.position.x - self.target_pose.position.x) <= position_tolerance and
            abs(current_pose.position.y - self.target_pose.position.y) <= position_tolerance and
            abs(current_pose.position.z - self.target_pose.position.z) <= position_tolerance
        )

        orientation_close = all(
            abs(getattr(current_pose.orientation, attr) - getattr(self.target_pose.orientation, attr)) <= orientation_tolerance
            for attr in ['x', 'y', 'z', 'w']
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

    # ======================== FLANGE POSITION ======================== #

    def get_flange_position(self):
        """Retrieve the current flange position and rotation in the world frame."""
        try:
            self.listener.waitForTransform('/world', '/flange', rospy.Time(0), rospy.Duration(1.0))
            robot_trans, robot_rot = self.listener.lookupTransform('/world', '/flange', rospy.Time(0))
            self.latest_flange_position = robot_trans
            self.latest_flange_rotation = robot_rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Could not get flange position: {e}")

    # ======================== LASER SCAN PROCESSING ======================== #

    def laser_scan_callback(self, msg):
        """Callback function for laser scan data."""
        self.latest_scan = msg
        if self.scanning and self.counter < 100:
            self.record_data()
            self.counter += 1
            self.rate.sleep()

    def record_data(self):
        """Record the latest laser scan data and flange position."""
        if self.latest_scan:
            self.get_flange_position()
            if self.latest_flange_position:
                self.data_storage.append({
                    'laser_scan': copy.deepcopy(self.latest_scan),
                    'trans': copy.deepcopy(self.latest_flange_position),
                    'rot': copy.deepcopy(self.latest_flange_rotation)
                })
                # rospy.loginfo(f"Recorded data: trans={self.latest_flange_position}, rot={self.latest_flange_rotation}")
            else:
                rospy.logwarn("Missing laser scan data or flange position.")

    def laser_scan_to_points(self, laser_scan):
        """Convert LaserScan to a 2D array of points."""
        points = []
        angle = laser_scan.angle_min
        for r in laser_scan.ranges:
            if laser_scan.range_min < r < laser_scan.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y, 0.0])
            angle += laser_scan.angle_increment
        return np.array(points)

    # ======================== TRANSFORMATIONS ======================== #

    def transform_points_to_world(self, points, trans, rot):
        """Transform 3D points from sensor frame to world frame using flange transform."""
        transform_matrix = tf.transformations.quaternion_matrix(rot)
        transform_matrix[:3, 3] = trans
        world_points = np.dot(transform_matrix[:3, :3], points.T).T + trans
        return world_points

    def transform_single_point_to_world(self, point, trans, rot):
        """Transform a single point from sensor frame to world frame."""
        transform_matrix = tf.transformations.quaternion_matrix(rot)
        transform_matrix[:3, 3] = trans
        point_hom = np.append(point, 1.0)
        return np.dot(transform_matrix, point_hom)[:3]
    
    def transform_points_with_euler(self, points, roll, pitch, yaw):
        """
        Transform points using Euler angles (roll, pitch, yaw).

        Args:
            points (np.ndarray): Array of points to transform (N, 3).
            roll (float): Rotation around the X-axis (in radians).
            pitch (float): Rotation around the Y-axis (in radians).
            yaw (float): Rotation around the Z-axis (in radians).

        Returns:
            np.ndarray: Transformed points (N, 3).
        """
        # Create a rotation matrix from Euler angles
        rotation_matrix = tf.transformations.euler_matrix(roll, pitch, yaw)[:3, :3]

        # Apply the rotation to all points in one matrix operation
        transformed_points = np.dot(points, rotation_matrix.T)
    
        return np.array(transformed_points)

    # ======================== DATA PROCESSING ======================== #

    def preprocess_data(self, data):
        """Preprocess laser scan data and filter outliers."""
        processed_scans = []
        for i, scan_pass in enumerate(data):
            print(f"Processing scan pass: {i+1}.")
            scan_points = []
            trans_data, rot_data = [], []

            for scan in scan_pass:
                laser_scan = scan['laser_scan']
                trans, rot = scan['trans'], scan['rot']
                sensor_points = self.laser_scan_to_points(laser_scan)
                scan_points.extend(sensor_points)
                trans_data.extend([trans] * len(sensor_points))
                rot_data.extend([rot] * len(sensor_points))

            scan_points = np.array(scan_points)
            trans_data, rot_data = np.array(trans_data), np.array(rot_data)

            plane_params, inliers = self.fit_ransac_2d(scan_points, distance_threshold=0.001)
            filtered_points = scan_points[inliers]
            world_points = np.array([self.transform_single_point_to_world(p, trans_data[i], rot_data[i]) for i, p in enumerate(filtered_points)])

            processed_scans.append(world_points)
        return processed_scans

    def fit_ransac_2d(self, points, distance_threshold=0.001):
        """Perform RANSAC regression on 2D data."""
        x = points[:, 0].reshape(-1, 1)
        y = points[:, 1].reshape(-1, 1)
        ransac = RANSACRegressor(residual_threshold=distance_threshold, max_trials=1000)
        ransac.fit(x, y)
        a, d = ransac.estimator_.coef_[0], -ransac.estimator_.intercept_
        return [a, -1, 0, d], np.where(ransac.inlier_mask_)[0]
    
    # ====================== LINEAR EQUATIONS ====================== #
    
    def construct_linear_system(self, points):
        """
        Constructs the A matrix and b vector for plane fitting.
        
        Args:
            points (np.ndarray): Array of points of shape (N, 3), where each row is (x, y, z).
        
        Returns:
            tuple: A matrix (N, 3), b vector (N,).
        """
        A = points[:, [1,2]]  # Use y and z for A
        A = np.hstack((A, np.ones((A.shape[0], 1))))  # Add the constant column for d
        b = points[:, 0]  # Use x for b
        return A, b

    def solve_plane(self, A, b):
        """
        Solves the linear system to find the best-fit plane parameters.
        
        Args:
            A (np.ndarray): Matrix constructed from x, y points (N, 3).
            b (np.ndarray): Vector of z values (N,).
        
        Returns:
            np.ndarray: Plane parameters [a, b, d].
        """
        pseudo_inverse = np.linalg.pinv(A)  # Use pseudo-inverse for over-determined systems
        plane_params = np.dot(pseudo_inverse, b)  # Solve for [b, c, d]
        return plane_params

    def calculate_residuals(self, A, b, plane_params):
        """
        Calculates residuals (errors) for the given plane fit.
        
        Args:
            A (np.ndarray): Matrix constructed from x, y points (N, 3).
            b (np.ndarray): Vector of z values (N,).
            plane_params (np.ndarray): Plane parameters [a, b, d].
        
        Returns:
            np.ndarray: Residuals for each data point.
        """
        predicted_b = np.dot(A, plane_params)  # Predicted z values
        residuals = b - predicted_b  # Calculate residuals
        return residuals
    
     # ================= OPTIMISATION FUNCTION ===================== #
     
    def calculate_cost(self, error):
        """
        Calculate the cost as the L2 norm of the error vector.
        
        Args:
            error (np.ndarray or list): Array of residual errors for all data points.
            
        Returns:
            float: L2 norm of the error vector.
        """
        # Sum of squared errors
        cost = sum(pow(s, 2) for s in error)
        
        # Take the square root of the sum
        return math.sqrt(cost)
    
    def objective_function(self, euler_angles, preprocessed_scans, counter):
        """
        Objective function for Euler angle optimization.

        Args:
            euler_angles (np.ndarray): Current guess for [roll, pitch, yaw] (3,).
            preprocessed_scans (list): List of inlier points for each scan.

        Returns:
            float: Total cost (C).
        """
        roll, pitch, yaw = euler_angles  # Unpack Euler angles
        
        total_cost = 0
        
        residuals = []
        total_transform_points = []
        total_plane = []
        
        for i, inlier_points in enumerate(preprocessed_scans):
            # Apply the guessed RPY to transform the points
            transformed_points = self.transform_points_with_euler(inlier_points, roll, pitch, yaw)
            
            # Fit a plane to the transformed points
            A, b = self.construct_linear_system(transformed_points)
            plane_params = self.solve_plane(A, b)
            total_transform_points.append(transformed_points)
            total_plane.append(plane_params)
            calc_residuals = self.calculate_residuals(A, b, plane_params)
            total_cost += self.calculate_cost(calc_residuals)
            residuals.extend(calc_residuals)
            
        # if not counter % 10:
        #     # self.plot_multiple_planes_with_points(total_plane, total_transform_points, margin=0.1, resolution=50)
        # # return np.array(residuals)
        # counter += 1
        return total_cost
    
    # ============ PLANE & POINT PLOTTING (optional) =================== #
    def plot_multiple_planes_with_points(self, plane_params_list, points_list, margin=0.1, resolution=10):
        """
        Plot multiple fitted planes and their corresponding points, color-coded for distinction.

        Args:
            plane_params_list (list): List of plane parameters [[a, b, d], ...] for each scan.
            points_list (list): List of point arrays [points1, points2, ...], where each is (N, 3).
            margin (float): Extra margin added to the axis limits (in meters).
            resolution (int): Number of points in each axis for plane plotting.
        """
        # Define a set of distinct colors
        colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan', 'magenta', 'yellow']

        # Create a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Initialize axis limits
        all_points = np.vstack(points_list)  # Combine all points to calculate global bounds
        min_vals = all_points.min(axis=0)
        max_vals = all_points.max(axis=0)

        x_min, y_min, z_min = min_vals - margin
        x_max, y_max, z_max = max_vals + margin

        # Loop through each scan and plot points and planes
        for i, (plane_params, points) in enumerate(zip(plane_params_list, points_list)):
            a, c, d = plane_params
            b = 1.0  # Since the equation is ax + by + z + d = 0, c is implicitly 1

            # Generate grid points for the plane within the global bounds
            x = np.linspace(x_min, x_max, resolution)
            y = np.linspace(y_min, y_max, resolution)
            xx, yy = np.meshgrid(x, y)
            zz = (d - a * xx - b * yy) / c  # Rearrange the plane equation for z

            # Plot points for this scan
            color = colors[i % len(colors)]  # Cycle through colors
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], color=color, label=f'Scan {i + 1} Points')

            # Plot the plane for this scan
            ax.plot_surface(xx, yy, zz, alpha=0.3, color=color)

        # Set axis limits based on the global bounds
        ax.set_xlim([x_min, x_max])
        ax.set_ylim([y_min, y_max])
        ax.set_zlim([z_min, z_max])

        # Add labels and legend
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Fitted Planes with Points')
        ax.legend()

        plt.show()

# ======================== ROBOT SCAN ============================ #

def collect_data(robot_movement):
    
    try:
        orientations = []
        square_pose_list = []

        # Create a target pose for the robot
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.35
        pose.position.z = 0.055
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        # Apply an initial offset to pose
        pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, 0, -1.57)

        # Move robot to initial position
        robot_movement.move_robot(pose, True)
        robot_movement.group.set_max_velocity_scaling_factor(0.1)
        robot_movement.group.set_max_acceleration_scaling_factor(0.1)

        # Generate different rotated orientations
        orientations.append(pose)
        x_neg_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, -0.8, 0, 0)  # Rotation around X
        orientations.append(x_neg_rot_pose)
        y_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, 0.8, 0)  # Rotation around Y
        orientations.append(y_rot_pose)
        z_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, 0, 0.9)  # Rotation around Z
        orientations.append(z_rot_pose)
        x_pos_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, 0.8, 0, 0)  # Rotation around X
        orientations.append(x_pos_rot_pose)
        y_neg_rot_pose = robot_movement.offset_movement(pose, 0, 0, -0.02, 0, -0.3, 0)  # Rotation around Y
        orientations.append(y_neg_rot_pose)
        z_neg_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, 0, -0.9)  # Rotation around Z
        orientations.append(z_neg_rot_pose)
        xy_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, -0.4, 0.4, 0)  # Rotation around X
        orientations.append(xy_rot_pose)
        yz_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, -0.3, 0.2)  # Rotation around Y
        orientations.append(yz_rot_pose)
        xz_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, 0.3, 0, -0.3)  # Rotation around Z
        orientations.append(xz_rot_pose)

        # Create square movement paths for each orientation
        for orient in orientations:
            square_pose = [orient]
            move_1 = robot_movement.offset_movement(orient, 0, 0.15, 0, 0, 0, 0)
            square_pose.append(move_1)
            move_2 = robot_movement.offset_movement(move_1, 0.15, 0, 0, 0, 0, 0)
            square_pose.append(move_2)
            move_3 = robot_movement.offset_movement(move_2, 0, -0.15, 0, 0, 0, 0)
            square_pose.append(move_3)
            move_4 = robot_movement.offset_movement(move_3, -0.15, 0, 0, 0, 0, 0)
            square_pose.append(move_4)
            square_pose_list.append(square_pose)

        scan_data = []

        # Execute square scanning movements
        for i, square_pose in enumerate(square_pose_list):
            robot_movement.scanning = False
            robot_movement.counter = 0
            print(f"Processing square poses for orientation {i + 1}:")

            # Store previous scan data before clearing
            if robot_movement.data_storage:
                scan_data.append(copy.deepcopy(robot_movement.data_storage))
            robot_movement.data_storage.clear()

            # Move through each pose in the square path
            for j, move in enumerate(square_pose):
                if j != 0:
                    robot_movement.scanning = True
                # print(f"Pose {j + 1}: {move}")
                robot_movement.move_robot(move, True)
            print(f"Number of scans for orientation {i + 1}: {robot_movement.counter}")
        # Stop scanning after all movements
        robot_movement.scanning = False
        scan_data.append(copy.deepcopy(robot_movement.data_storage))
        
        # Save scan data
        with open("scan_data.pkl", "wb") as f:
            pickle.dump(scan_data, f)
                        
        return scan_data
    except:
        rospy.logwarn("An error occurred while collecting data.")
        robot_movement.stop_robot()

# ====================== ROTATION OPTIMIZATION ===================== #

def optimise_rotation():
    # Load scan data
    with open("scan_data.pkl", "rb") as f:
        scan_data = pickle.load(f)

    # Process data before optimization
    clean_world_points = robot_movement.preprocess_data(scan_data)

    # Initial guess for Euler angles (roll, pitch, yaw)
    initial_euler = np.array([0.01, 0.01, -0.01])

    # Define search bounds for optimization
    global_lower_bound = -2 * np.pi
    global_upper_bound = 2 * np.pi
    max_deviation = np.pi / 4

    lower_bounds = np.maximum(initial_euler - max_deviation, global_lower_bound)
    upper_bounds = np.minimum(initial_euler + max_deviation, global_upper_bound)
    bounds = list(zip(lower_bounds, upper_bounds))

    # Run optimization
    counter = 0
    result = minimize(
        fun=robot_movement.objective_function,
        x0=initial_euler,
        args=(clean_world_points, counter),
        method='trust-constr',
        bounds=bounds,
        options={'verbose': 2, 'maxiter': 5000},  
        tol=1e-9,
    )

    optimized_euler = result.x
    print(f"Optimized Euler Angles (roll, pitch, yaw): {optimized_euler}")

# ======================== MAIN EXECUTION ======================== #

if __name__ == '__main__':
    try:
        robot_movement = RobotMovement()

        # Collect data
        scan_data = collect_data(robot_movement)

        # # Optimise rotation
        optimise_rotation()
        

    except rospy.ROSInterruptException:
        pass


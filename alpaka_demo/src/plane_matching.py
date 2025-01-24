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
import numpy as np
import open3d as o3d
import pickle
import copy
from scipy.optimize import minimize
from sklearn.linear_model import RANSACRegressor
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
        self.plane_parameters = []  # To store plane parameters for each scan
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

    def laser_scan_callback(self, msg):
        # Store the latest laser scan data
        self.latest_scan = msg
        # Point cloud conversion
        # self.cloud_msg = self.projector.projectLaser(msg)
        if self.scanning and self.counter < 200:
            self.record_data()
            self.counter +=1
            self.rate.sleep()
        # rospy.loginfo("Received laser scan data")

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
        # Store the laser scan and flange position as a dictionary
        if self.latest_scan:
            self.get_flange_position()
            if self.latest_scan and self.latest_flange_position:
                scan_and_flange_data = {
                    'laser_scan': copy.deepcopy(self.latest_scan),
                    # 'point_cloud': self.cloud_msg, #point cloud storage
                    'trans': copy.deepcopy(self.latest_flange_position),
                    'rot': copy.deepcopy(self.latest_flange_rotation)
                }
                self.data_storage.append(copy.deepcopy(scan_and_flange_data))
                rospy.loginfo(f"Recorded data: trans={self.latest_flange_position}, rot={self.latest_flange_rotation}")
                # Publish point cloud
                # self.cloud_pub.publish(self.cloud_msg)
                
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
    
    def laser_scan_to_points(self, laser_scan):
        """
        Convert geometry_msgs/LaserScan to 2D points in the sensor frame.
        """
        points = []
        angle = laser_scan.angle_min
        for r in laser_scan.ranges:
            if laser_scan.range_min < r < laser_scan.range_max:  # Filter valid range
                x = r * np.sin(angle)
                z = r * np.cos(angle)
                points.append([x, 0.0, z])  # y = 0 for 2D scans
            angle += laser_scan.angle_increment
            
        return np.array(points)
        
    def transform_points_to_world(self, points, trans, rot):
        """
        Transform points from the sensor frame to the world frame using the flange transform.
        """
        # Create the transformation matrix
        transform_matrix = tf.transformations.quaternion_matrix(rot)
        transform_matrix[:3, 3] = trans

        # Transform points
        world_points = []
        for point in points:
            point_hom = np.append(point, 1.0)  # Homogeneous coordinates
            transformed_point = np.dot(transform_matrix, point_hom)[:3]  # Transform
            world_points.append(transformed_point)
            
        return np.array(world_points)
    
    def transform_single_point_to_world(self, single_point, trans, rot):
        """
        Transform points from the sensor frame to the world frame using the flange transform.
        """
        # Create the transformation matrix
        transform_matrix = tf.transformations.quaternion_matrix(rot)
        transform_matrix[:3, 3] = trans

        # Transform points
        point_hom = np.append(single_point, 1.0)  # Homogeneous coordinates
        transformed_point = np.dot(transform_matrix, point_hom)[:3]  # Transform
        return transformed_point 
    
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
    
    def fit_plane_ransac(self, points, distance_threshold=0.01, ransac_n=3, num_iterations=1000, voxel_size=0.001):
        """
        Fit a plane to the given points using RANSAC.
        """
        # Convert points to Open3D format
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        print(f"Point cloud bounds: {pcd.get_min_bound()} to {pcd.get_max_bound()}")
        print(f"Sample points: {points[:10]}")

        # # Apply voxel downsampling (optional)
        # voxel_size = voxel_size
        # if len(points) > 1000:
        #     pcd = pcd.voxel_down_sample(voxel_size)
        
        # nb_neighbors = 10
        # std_ratio = 3.0
        
        # pcd, _ = pcd.remove_statistical_outlier(nb_neighbors, std_ratio)
    
        print(f"size before ransac: {len(pcd.points)}")
        # Perform RANSAC plane fitting
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )
        
        
        distances = np.abs(np.dot(points, plane_model[:3]) + plane_model[3]) / np.linalg.norm(plane_model[:3])
        print(f"Distance stats: Min = {np.min(distances)}, Max = {np.max(distances)}, Mean = {np.mean(distances)}")

        rospy.loginfo(f"Plane Model: {plane_model}")
        rospy.loginfo(f"Number of Inliers: {len(inliers)}")    
            
        # Visualize inliers
        inlier_cloud = pcd.select_by_index(inliers)
        
        # Convert Open3D point cloud to NumPy array
        inlier_points = np.asarray(inlier_cloud.points)
        distances = np.abs(np.dot(inlier_points, plane_model[:3]) + plane_model[3]) / np.linalg.norm(plane_model[:3])
        print(f"Distances: {distances[:10]}")
        print(f"inlier data: {np.asarray(inlier_cloud.points)}")
        
        
        inlier_cloud.paint_uniform_color([1.0, 0.0, 0.0])  # Red for inliers

        # Visualize outliers
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        outlier_cloud.paint_uniform_color([0.0, 0.0, 1.0])  # Blue for outliers

        # Draw the inliers and outliers
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
            
        return plane_model, inliers
    
    def fit_ransac_2d(self, points, distance_threshold=0.001):
        """
        Perform RANSAC for a 2D dataset (x and z coordinates).
        
        Args:
            points (np.ndarray): Array of points (N, 3), where each row is [x, y, z].
            distance_threshold (float): Distance threshold to identify inliers.
            
        Returns:
            tuple: Coefficients of the fitted model (a, c, d for ax + cz = d) and inliers.
        """
        # Extract x and z coordinates
        x = points[:, 0].reshape(-1, 1)  # Feature (independent variable)
        z = points[:, 2]  # Target (dependent variable)

        # RANSAC model
        ransac = RANSACRegressor(
            residual_threshold=distance_threshold,
            max_trials=1000
        )
        ransac.fit(x, z)

        # Extract model coefficients
        slope = ransac.estimator_.coef_[0]  # Coefficient for x
        intercept = ransac.estimator_.intercept_  # Intercept (c)

        # Reconstruct the plane model: ax + cz = d
        a = slope
        c = -1.0  # Coefficient for z
        d = -intercept
        plane_model = [a, 0.0, c, d]  # Assuming y coefficient is 0

        # Get inliers
        inlier_mask = ransac.inlier_mask_
        inliers = np.where(inlier_mask)[0]

        print(f"RANSAC Model: {plane_model}")
        print(f"Number of Inliers: {len(inliers)}")

        return plane_model, inliers
    
    def process_laser_scans(self, data_storage):
        all_world_inliers = []
        all_fitted_planes = [] # Store plane models and point clou

        for i, scan_pass in enumerate(data_storage):
            aggregated_points = []
            
            # Step 1: Aggregate all points from the scan pass
            for scan_data in scan_pass:
                laser_scan = scan_data['laser_scan']  # geometry_msgs/LaserScan
                trans = scan_data['trans']
                rot = scan_data['rot']

                # Convert LaserScan to 2D points in the sensor frame
                sensor_points = self.laser_scan_to_points(laser_scan)

                # Transform points to the world frame
                world_points = self.transform_points_to_world(sensor_points, trans, rot)

                aggregated_points.extend(world_points)
                
            # Step 2: Fit a plane to the world points using RANSAC
            aggregated_points = np.array(aggregated_points)
            
            
            rospy.loginfo(f"Scan Pass {i + 1}: {len(aggregated_points)} points")
            distances = np.linalg.norm(np.diff(aggregated_points, axis=0), axis=1)
            print(f"Min Distance: {np.min(distances)}, Max Distance: {np.max(distances)}")

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(aggregated_points)
            pcd.paint_uniform_color([0.1, 0.9, 0.1])  # Green
            o3d.visualization.draw_geometries([pcd])

            plane_model, inliers = self.fit_plane_ransac(aggregated_points)
            
            all_world_inliers.extend(np.array(aggregated_points[inliers]))
            # Store results
            all_fitted_planes.append({
                'plane_model': plane_model,  # Plane coefficients [a, b, c, d]
                'inliers': inliers,          # Indices of inliers
                'world_points': aggregated_points  # All points in world frame
            })
            
        self.visualize_with_open3d(all_fitted_planes, np.array(all_world_inliers), interval=1, size=0.05, resolution=20)

        return all_fitted_planes
    
    def create_plane_mesh(self, plane_model, inlier_points, size=0.2, resolution=10):
        """
        Create a plane mesh for visualization.
        :param plane_model: Coefficients [a, b, c, d] of the plane equation ax + by + cz + d = 0.
        :param center: A point on the plane, e.g., the centroid of inliers.
        :param size: Half-width of the plane's extent (in meters).
        :param resolution: Number of grid points along each axis.
        :return: Open3D TriangleMesh representing the plane.
        """
        a, b, c, d = plane_model

        # Generate grid points
        x = np.linspace(-size, size, resolution)
        y = np.linspace(-size, size, resolution)
        xx, yy = np.meshgrid(x, y)

        # Compute z values based on the plane equation
        zz = (-a * xx - b * yy - d) / c

        # Create vertices for the mesh
        vertices = np.vstack((xx.ravel(), yy.ravel(), zz.ravel())).T
        
        # Translate the vertices to align with inlier points
        centroid = np.mean(inlier_points, axis=0)
        vertices += centroid

        # Create triangles for the grid
        triangles = []
        for i in range(resolution - 1):
            for j in range(resolution - 1):
                idx = i * resolution + j
                triangles.append([idx, idx + resolution, idx + 1])
                triangles.append([idx + 1, idx + resolution, idx + resolution + 1])

        # Create Open3D TriangleMesh
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices)  # Translate to the plane center
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        mesh.compute_vertex_normals()
        
        return mesh

    def visualize_with_open3d(self, fitted_planes, aggregated_points, interval=1, size=0.1, resolution=20):
        """
        Visualize aggregated planes and inliers in 3D.
        """
        geometries = []

        # Add a coordinate frame
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        geometries.append(axis)
        
        # Add a scaled grid based on aggregated points
        if len(aggregated_points) > 0:
            grid = self.create_scaled_grid(data=aggregated_points, size=0.2, resolution=20)
            geometries.append(grid)
        else:
            rospy.logwarn("No points available to scale the grid.")
        
        colors = [
        [1.0, 0.0, 0.0],  # Red
        [0.0, 1.0, 0.0],  # Green
        [0.0, 0.0, 1.0],  # Blue
        [1.0, 1.0, 0.0],  # Yellow
        ]
        

        for i, plane_data in enumerate(fitted_planes):
            if i % interval != 0:
                continue

            plane_model = plane_data['plane_model']  # [a, b, c, d]
            inlier_points = np.array(plane_data['world_points'])[plane_data['inliers']]  # Extract inliers
            return

            # Create a plane mesh
            plane_mesh = self.create_plane_mesh(plane_model, inlier_points, size=size, resolution=resolution)
            plane_mesh.paint_uniform_color(colors[i])  # Random color for each plane
            geometries.append(plane_mesh)

            # Create point cloud for inliers
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(inlier_points)
            pcd.paint_uniform_color(colors[i])  # Green for points
            geometries.append(pcd)

        # Visualize all geometries
        o3d.visualization.draw_geometries(geometries)
    
    def create_scaled_grid(self, data, size=0.1, resolution=10):
        """
        Create a scaled grid for visualization.
        :param size: Total extent of the grid (meters).
        :param resolution: Number of lines along each axis.
        :return: Open3D LineSet representing the grid.
        """
        
         # Ensure data is a numpy array
        data = np.array(data)
    
        # Compute bounds of the points
        min_bound = np.min(data, axis=0)
        max_bound = np.max(data, axis=0)
        center = (min_bound + max_bound) / 2
    
        lines = []
        grid_points = []

        step = size / resolution
        for i in range(-resolution, resolution + 1):
            # Lines parallel to the X-axis
            grid_points.append([i * step, -size / 2, center[2]])
            grid_points.append([i * step, size / 2, center[2]])
            lines.append([len(grid_points) - 2, len(grid_points) - 1])

            # Lines parallel to the Y-axis
            grid_points.append([-size / 2, i * step, center[2]])
            grid_points.append([size / 2, i * step, center[2]])
            lines.append([len(grid_points) - 2, len(grid_points) - 1])

        # Create a LineSet
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(grid_points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.paint_uniform_color([0.5, 0.5, 0.5])  # Gray grid lines

        return line_set    

    def construct_linear_system(self, points):
        """
        Constructs the A matrix and b vector for plane fitting.
        
        Args:
            points (np.ndarray): Array of points of shape (N, 3), where each row is (x, y, z).
        
        Returns:
            tuple: A matrix (N, 3), b vector (N,).
        """
        A = points[:, :2]  # Use x and y for A
        A = np.hstack((A, np.ones((A.shape[0], 1))))  # Add the constant column for d
        b = points[:, 2]  # Use z for b
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
        plane_params = np.dot(pseudo_inverse, b)  # Solve for [a, b, d]
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

    def fit_plane_for_scan(self, scan_itt):
        """
        Fits a plane to the given set of points and calculates residuals.
        
        Args:
            points (np.ndarray): Array of points of shape (N, 3), where each row is (x, y, z).
        
        Returns:
            tuple: Plane parameters [a, b, d] and residuals.
        """
        print(f"shape of scan: {scan_itt.shape}")

        A, b = self.construct_linear_system(scan_itt)
        plane_params = self.solve_plane(A, b)
        residuals = self.calculate_residuals(A, b, plane_params)
        return plane_params, residuals
    
    
    def convert_laser_scan_to_list_of_points(self, data):
        
        points_per_scan = []
        
        for i, scan_pass in enumerate(data):
            
            scan_points = []
            
            # Step 1: Aggregate all points from the scan pass
            for data_scan in scan_pass:
                laser_scan = data_scan['laser_scan']  # geometry_msgs/LaserScan
                translation = data_scan['trans']
                rotation = data_scan['rot']
                # print(f"trans: {translation} rotation: {rotation}")
                # break

                # Convert LaserScan to 2D points in the sensor frame
                sensor_points = self.laser_scan_to_points(laser_scan)
                
                # # Filter points for ransac
                # plane_params, inliers = self.fit_plane_ransac(sensor_points)

                # Transform points to the world frame
                world_points = self.transform_points_to_world(sensor_points, translation, rotation)
                
                # Add the points for this laser scan to the current scan pass
                scan_points.extend(world_points)
                
            # Step 2: Make a list of each scan
            points_per_scan.append(np.array(scan_points))
           
           # Debugging: Print the shape of the aggregated points for the current scan
            print(f"Scan {i + 1}: {len(scan_points)} points, Shape: {np.array(scan_points).shape}")
            print(f"Scan {i + 1}: {scan_points[:5]} (Showing first 5 points)")


        return points_per_scan   
    
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
                    
        
    def process_scans(self, data):
        """
        Processes multiple scans and fits a plane to each scan separately.
        
        Args:
            scans (list): List of np.ndarray, where each array contains points (N, 3) for one scan.
        
        Returns:
            list: List of dictionaries containing plane parameters and residuals for each scan.
        """
        scans = self.convert_laser_scan_to_list_of_points(data)
        
        results = []
        for i, scan in enumerate(scans):
            # print(f"Processing scan {i + 1}/{len(scans)}, shape: {scan.shape}")
            plane_params, inliers = self.fit_plane_ransac(scan)
            plane_params, residuals = self.fit_plane_for_scan(scan[inliers])
            cost_Fi = self.calculate_cost(residuals)
            self.plane_parameters.append(plane_params)
            self.errors.append(residuals)
            results.append({
                "scan_index": i,
                "plane_parameters": plane_params,
                "residuals": residuals,
                "Fi_cost": cost_Fi
            })
        
        total_cost = sum(F['Fi_cost'] for F in results)
            
        return results, total_cost
    
    def preprocess_data(self, data):
        
        points_per_scan = []
        
        for i, scan_pass in enumerate(data):
            
            scan_points = []
            trans_data = []
            rot_data = []
            
            
            # Step 1: Aggregate all points from the scan pass
            for data_scan in scan_pass:
                laser_scan = data_scan['laser_scan']  # geometry_msgs/LaserScan
                translation = data_scan['trans']
                rotation = data_scan['rot']
                # print(f"trans: {translation} rotation: {rotation}")
                # break
    
                # Convert LaserScan to 2D points in the sensor frame
                sensor_points = self.laser_scan_to_points(laser_scan)
                
                # Add the points for this laser scan to the current scan pass
                scan_points.extend(sensor_points)
                trans_data.extend([translation] * len(sensor_points))
                rot_data.extend([rotation] * len(sensor_points))
            
            scan_points = np.array(scan_points)
            trans_data = np.array(trans_data)
            rot_data = np.array(rot_data)
            
            # Filter points for ransac
            # plane_params, inliers = self.fit_plane_ransac(scan_points, distance_threshold=0.0001, num_iterations=1000)
            plane_params, inliers = self.fit_ransac_2d(scan_points, distance_threshold=0.0005)
            print(f"shape of scan_points: {np.array(scan_points).shape}")
            
            filtered_points = scan_points[inliers]
            filtered_trans = trans_data[inliers]
            filtered_rot = rot_data[inliers]
            
            world_points = []
            
            for j, point in enumerate(filtered_points):
                # Transform points to the world frame
                print(f"flange trans: {filtered_trans[j]}")
                break
                transformed_point = self.transform_single_point_to_world(point, filtered_trans[j], filtered_rot[j])
                world_points.append(transformed_point) 
                
            # Step 2: Make a list of each scan
            points_per_scan.append(np.array(world_points))
           
           # Debugging: Print the shape of the aggregated points for the current scan
            # print(f"Scan {i + 1}: {len(world_points)} points, Shape: {np.array(world_points).shape}")
            # print(f"Scan {i + 1}: {world_points[:5]} (Showing first 5 points)")

        return points_per_scan
    
    def objective_function(self, euler_angles, preprocessed_scans):
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
            
        self.plot_multiple_planes_with_points(total_plane, total_transform_points, margin=0.1, resolution=50)
        return np.array(residuals)
    
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
            a, b, d = plane_params
            c = 1  # Since the equation is ax + by + z + d = 0, c is implicitly 1

            # Generate grid points for the plane within the global bounds
            x = np.linspace(x_min, x_max, resolution)
            y = np.linspace(y_min, y_max, resolution)
            xx, yy = np.meshgrid(x, y)
            zz = -a * xx - b * yy - d  # Compute z values for the plane

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
    

if __name__ == '__main__':
    try:
        # Create an instance of RobotMovement
        robot_movement = RobotMovement()
        
        # orientations = []
        # square_pose_list = []
        

        # # Create a target pose for the robot (example)
        # pose = Pose()
        # pose.position.x = 0.0
        # pose.position.y = 0.35
        # pose.position.z = 0.055
        # pose.orientation.x = 1.0
        # pose.orientation.y = 0.0
        # pose.orientation.z = 0.0
        # pose.orientation.w = 0.0
        # pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, 0, -1.57)
        # # Move robot asynchronously (non-blocking)
        # robot_movement.move_robot(pose, True)
        # robot_movement.group.set_max_velocity_scaling_factor(0.1)
        # robot_movement.group.set_max_acceleration_scaling_factor(0.1)
        
        
        # rate = rospy.Rate(10)  # 10 Hz
        
        # orientations.append(pose)
        # x_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, -0.8, 0, 0) #rotation around x
        # orientations.append(x_rot_pose)
        # y_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, 0.8, 0) #rotation around y
        # orientations.append(y_rot_pose)
        # z_rot_pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, 0, 0.9) #rotation around z
        # orientations.append(z_rot_pose)
        
        # for orient in orientations:
        #     square_pose = []
        #     square_pose.append(orient)
        #     move_1 = robot_movement.offset_movement(orient, 0, 0.15, 0, 0, 0, 0)
        #     square_pose.append(move_1)
        #     move_2 = robot_movement.offset_movement(move_1, 0.15, 0, 0, 0, 0, 0)
        #     square_pose.append(move_2)
        #     move_3 = robot_movement.offset_movement(move_2, 0, -0.15, 0, 0, 0, 0)
        #     square_pose.append(move_3)
        #     move_4 = robot_movement.offset_movement(move_3, -0.15, 0, 0, 0, 0, 0)
        #     square_pose.append(move_4)
        #     square_pose_list.append(square_pose)
        
        # scan_data = []
        
        # for i, square_pose in enumerate(square_pose_list):
        #     # if i == 1:
        #     #     break
            
        #     robot_movement.scanning = False
        #     robot_movement.counter = 0
        #     print(f"Processing square poses for orientation {i+1}:")
        #     if robot_movement.data_storage:
        #         scan_data.append(copy.deepcopy(robot_movement.data_storage))
        #     robot_movement.data_storage.clear()
            
        #     # Iterate over each pose in the current square pose list
        #     for j, move in enumerate(square_pose):
        #         if j != 0:
        #             robot_movement.scanning = True  
        #         print(f"Pose {j+1}: {move}")
        #         robot_movement.move_robot(move, True)
        # robot_movement.scanning = False
        # scan_data.append(copy.deepcopy(robot_movement.data_storage))
        
        # with open("scan_data.pkl", "wb") as f:
        #     pickle.dump(scan_data, f)
            
        
        
        with open("scan_data.pkl", "rb") as f:
            scan_data = pickle.load(f)
            
        # print(f"Data stored for processing: {scan_data[1]}")
        # results, total_cost = robot_movement.process_scans(scan_data)
        # print(results)
        # print(f'total cost: {total_cost}')
        
        # Initial guess for Euler angles (roll, pitch, yaw)
        initial_euler = np.array([-0.5, -0.4, -0.5])  # No rotation

        clean_world_points = robot_movement.preprocess_data(scan_data)
        
        # Define bounds for optimization (e.g., ±π/4 for Euler angles or unconstrained for quaternion)
        # bounds = [
        #     (-np.pi / 4, np.pi / 4),  # Roll
        #     (-np.pi / 4, np.pi / 4),  # Pitch
        #     (-np.pi / 4, np.pi / 4),  # Yaw
        # ]
        bounds = (
            [-np.pi / 4, -np.pi / 4, -np.pi / 4],  # Lower bounds
            [np.pi / 4, np.pi / 4, np.pi / 4]      # Upper bounds
        )
        
        # Run optimization
        # result = minimize(
        #     fun=robot_movement.objective_function,
        #     x0=initial_euler,
        #     args=(clean_world_points,),
        #     method='trust-constr',
        #     bounds=bounds,
        #     options={'verbose': 2},  # For detailed optimization output
        #     tol=1e-9  # Adjust tolerance for desired precision
        # )
        
        # Call least_squares
        # result = least_squares(
        #     fun=robot_movement.objective_function,
        #     x0=initial_euler,
        #     args=(clean_world_points,),
        #     method='trf',  # Trust-region reflective algorithm
        #     bounds=bounds,  # Specify bounds
        #     xtol=1e-9,  # Tolerance for termination
        #     verbose=2  # Detailed output
        # )

        # # Retrieve optimized Euler angles
        # optimized_euler = result.x
        # print(f"Optimized Euler Angles (roll, pitch, yaw): {optimized_euler}")
        # print("Residual sum of squares:", result.cost)

        robot_movement.objective_function(initial_euler,clean_world_points)
     
            
    except rospy.ROSInterruptException:
        pass

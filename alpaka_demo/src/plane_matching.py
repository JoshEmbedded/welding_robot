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
                    'laser_scan': self.latest_scan,
                    # 'point_cloud': self.cloud_msg, #point cloud storage
                    'trans': self.latest_flange_position,
                    'rot': self.latest_flange_rotation
                }
                self.data_storage.append(scan_and_flange_data)
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
    
    def fit_plane_ransac(self, points, distance_threshold=0.02, ransac_n=3, num_iterations=2000):
        """
        Fit a plane to the given points using RANSAC.
        """
        # Convert points to Open3D format
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Apply voxel downsampling (optional)
        voxel_size = 0.005
        if len(points) > 1000:
            pcd = pcd.voxel_down_sample(voxel_size)
        
        nb_neighbors = 10
        std_ratio = 3.0
        
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors, std_ratio)
    
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
        inlier_cloud.paint_uniform_color([1.0, 0.0, 0.0])  # Red for inliers

        # Visualize outliers
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        outlier_cloud.paint_uniform_color([0.0, 0.0, 1.0])  # Blue for outliers

        # Draw the inliers and outliers
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
            
        return plane_model, inliers
    
    def process_laser_scans(self, data_storage):
        all_world_points = []
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
            
            # Store results
            all_fitted_planes.append({
                'plane_model': plane_model,  # Plane coefficients [a, b, c, d]
                'inliers': inliers,          # Indices of inliers
                'world_points': aggregated_points  # All points in world frame
            })
            
            self.visualize_with_open3d(all_fitted_planes, interval=1, size=0.05, resolution=20)

        return all_fitted_planes
    
    def create_plane_mesh(self, plane_model, center, size=0.01, resolution=10):
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

        # Create triangles for the grid
        triangles = []
        for i in range(resolution - 1):
            for j in range(resolution - 1):
                idx = i * resolution + j
                triangles.append([idx, idx + resolution, idx + 1])
                triangles.append([idx + 1, idx + resolution, idx + resolution + 1])

        # Create Open3D TriangleMesh
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices + center)  # Translate to the plane center
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        mesh.compute_vertex_normals()
        
        return mesh

    def visualize_with_open3d(self, fitted_planes, interval=1, size=0.1, resolution=20):
        """
        Visualize aggregated planes and inliers in 3D.
        """
        geometries = []

        # Add a coordinate frame
        
        # Add a scaled grid
        grid = self.create_scaled_grid(size=0.1, resolution=10)
        geometries.append(grid)
    
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        geometries.append(axis)

        for i, plane_data in enumerate(fitted_planes):
            if i % interval != 0:
                continue

            plane_model = plane_data['plane_model']  # [a, b, c, d]
            inlier_points = np.array(plane_data['world_points'])[plane_data['inliers']]  # Extract inliers
            plane_center = np.mean(inlier_points, axis=0)  # Use centroid as a point on the plane

            # Create a plane mesh
            plane_mesh = self.create_plane_mesh(plane_model, plane_center, size=size, resolution=resolution)
            plane_mesh.paint_uniform_color(np.random.rand(3))  # Random color for each plane
            geometries.append(plane_mesh)

            # Create point cloud for inliers
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(inlier_points)
            pcd.paint_uniform_color([0.1, 0.9, 0.1])  # Green for points
            geometries.append(pcd)

        # Visualize all geometries
        o3d.visualization.draw_geometries(geometries)
    
    def create_scaled_grid(self, size=0.05, resolution=10):
        """
        Create a scaled grid for visualization.
        :param size: Total extent of the grid (meters).
        :param resolution: Number of lines along each axis.
        :return: Open3D LineSet representing the grid.
        """
        lines = []
        points = []

        step = size / resolution
        for i in range(-resolution, resolution + 1):
            # Lines parallel to the X-axis
            points.append([i * step, -size / 2, 0])
            points.append([i * step, size / 2, 0])
            lines.append([len(points) - 2, len(points) - 1])

            # Lines parallel to the Y-axis
            points.append([-size / 2, i * step, 0])
            points.append([size / 2, i * step, 0])
            lines.append([len(points) - 2, len(points) - 1])

        # Create a LineSet
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.paint_uniform_color([0.5, 0.5, 0.5])  # Gray grid lines

        return line_set
    
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
        pose.position.z = 0.05
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, 0, -1.57)
        # Move robot asynchronously (non-blocking)
        robot_movement.move_robot(pose, True)
        robot_movement.group.set_max_velocity_scaling_factor(0.05)
        robot_movement.group.set_max_acceleration_scaling_factor(0.05)
        
        
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
            move_1 = robot_movement.offset_movement(orient, 0, 0.05, 0, 0, 0, 0)
            square_pose.append(move_1)
            move_2 = robot_movement.offset_movement(move_1, 0.05, 0, 0, 0, 0, 0)
            square_pose.append(move_2)
            move_3 = robot_movement.offset_movement(move_2, 0, -0.05, 0, 0, 0, 0)
            square_pose.append(move_3)
            move_4 = robot_movement.offset_movement(move_3, -0.05, 0, 0, 0, 0, 0)
            square_pose.append(move_4)
            square_pose_list.append(square_pose)
        
        scan_data = []
        
        for i, square_pose in enumerate(square_pose_list):
            if i == 1:
                break
            
            robot_movement.scanning = False
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
        
        fitted_planes = robot_movement.process_laser_scans(scan_data)
        # print(f"fitted_planes size: {len(fitted_planes)}")
        # Display point cloud conversion
        # print(f"cloud data: {scan_data[0][0].get('point_cloud', None)}")
            
    except rospy.ROSInterruptException:
        pass

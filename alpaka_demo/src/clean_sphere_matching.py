import sys
import rospy
import moveit_commander
import tf
import tf2_ros
import numpy as np
import open3d as o3d
import copy
import rosbag
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tft
import pickle

from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
from laser_geometry import LaserProjection

from scipy.optimize import least_squares


class RobotScanner:
    def __init__(self):
        # Initialize MoveIt and ROS
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_goal_position_tolerance(0.0005)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Laser scan processing
        self.projector = LaserProjection()
        self.scan_sub = rospy.Subscriber("/scan_filtered", LaserScan, self.laser_scan_callback, tcp_nodelay=True)
        self.cloud_pub = rospy.Publisher('assembled_cloud', PointCloud2, queue_size=10, latch=True)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.latest_scan = None
        self.latest_tf = None
        self.scanning = False
        self.data_storage = []
        self.first_scan_transform = None  # Reference transform for alignment
        self.cloud_list = []

    def move_robot(self, pose, blocking=True):
        """Move the robot to a specified pose."""
        self.group.set_pose_target(pose)
        success = self.group.go(wait=blocking)
        self.group.clear_pose_targets()
        return success

    def laser_scan_callback(self, scan_msg):
        """Process incoming laser scan data."""
        if self.scanning:
            T_world_flange = self.get_latest_transform("world", "flange")
            self.process_scan(scan_msg, T_world_flange)
    
    def process_scan(self, scan_msg, transform):
        """Convert LaserScan to PointCloud2 and transform to a consistent frame."""
        cloud_msg = self.laserscan_to_pointcloud(scan_msg)
        if len(cloud_msg) == 0:
            print("Empty cloud, skipping...")
            return
        # T_world_flange = self.get_transform("world", "flange")

        if self.first_scan_transform is None:
            self.first_scan_transform = copy.deepcopy(transform)

        T_first_scan_world = np.linalg.inv(self.first_scan_transform)
        T_flange_first = T_first_scan_world @ transform

        self.data_storage.append({  'cloud': copy.deepcopy(cloud_msg),
                                    'transform': copy.deepcopy(T_flange_first)
                                    })
        
    def get_latest_transform(self, target_frame, source_frame):
        """Retrieve the latest transformation and ensure it's not stale."""
        try:
            # ✅ Ensure the transform is fresh
            if not self.tf_buffer.can_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0)):
                rospy.logwarn(f"⚠️ Transform not available yet: {source_frame} → {target_frame}")
                return np.eye(4)  # Use identity matrix if TF is not available

            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))

            translation = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

            matrix = tf.transformations.quaternion_matrix(rotation)
            matrix[:3, 3] = translation

            # print(f"✅ Using new transform:\n{matrix}")  # ✅ Debugging

            return matrix  # ✅ Returns fresh transformation

        except tf2_ros.LookupException:
            rospy.logwarn(f"⚠️ TF lookup failed: {source_frame} → {target_frame}, using identity matrix")
            return np.eye(4)  # Use identity matrix as a fallback`


    def laserscan_to_pointcloud(self, scan_msg):
        """Convert LaserScan to PointCloud2."""
        points = []
        angle = scan_msg.angle_min

        for r in scan_msg.ranges:
            if r < scan_msg.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                z = 0.0
                points.append([x, y, z])
            angle += scan_msg.angle_increment
        return points
        # return pc2.create_cloud_xyz32(scan_msg.header, points)

    def pointcloud2_to_open3d(self, ros_cloud):
        """Convert ROS PointCloud2 to Open3D PointCloud."""
        points = [list(point) for point in pc2.read_points(ros_cloud, field_names=("x", "y", "z"), skip_nans=True)]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float32))
        return pcd

    def transform_pointcloud(self, pcd, transform_matrix):
        """Apply a transformation matrix to an Open3D point cloud."""
        points = np.asarray(pcd.points)
        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
        transformed_points = (transform_matrix @ points_homogeneous.T).T[:, :3]
        transformed_pcd = o3d.geometry.PointCloud()
        transformed_pcd.points = o3d.utility.Vector3dVector(transformed_points)
        return transformed_pcd

    def merge_pointclouds(self, pcd_list):
        """Merge a list of Open3D point clouds into a single point cloud."""
        merged_pcd = o3d.geometry.PointCloud()
        for pcd in pcd_list:
            merged_pcd += pcd
        return merged_pcd
    
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
    
def merge_pointclouds(pcd_list):
    """
    Merges a list of Open3D point clouds into a single point cloud.
    
    :param pcd_list: List of Open3D.geometry.PointCloud objects
    :return: A merged Open3D PointCloud
    """
    merged_points = []
    
    for i, pcd in enumerate(pcd_list):
        points_array = np.asarray(copy.deepcopy(pcd).points)  # ✅ Ensure new copy
        
        # print(f"🔍 Scan {i+1}: {points_array.shape[0]} points")
        
        if points_array.shape[0] == 0:
            print(f"❌ Warning: Scan {i+1} is empty!")
            
        merged_points.extend(np.asarray(points_array))  # Extend list with new points

    # Create a new Open3D point cloud
    merged_pcd = o3d.geometry.PointCloud()
    merged_pcd.points = o3d.utility.Vector3dVector(np.array(merged_points))
    o3d.visualization.draw_geometries([merged_pcd])
    return copy.deepcopy(merged_pcd)


def robot_scan(robot_movement):
    
    orientations = []
    point_clouds = []
    
    # Create a target pose for the robot (example)
    pose = Pose()
    pose.position.x = -0.02
    pose.position.y = 0.40
    pose.position.z = 0.055
    pose.orientation.x = 1.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0
    
    pose = robot_movement.offset_movement(pose, 0, 0, 0, 0, 0, -1.57)
    second_viewing_angle = robot_movement.offset_movement(pose, 0.0, 0, 0, 0.7, 0, 0)
    third_viewing_angle =  robot_movement.offset_movement(pose, 0.02, 0, 0.01, 0, 0.7, 0)
    
    end_pose = robot_movement.offset_movement(pose, 0.04, 0.12, 0, 0, 0, 0)
    second_end_pose = robot_movement.offset_movement(pose, 0.04, 0.12, 0, -0.7, 0, 0)
    third_end_pose = robot_movement.offset_movement(pose, 0.02, 0.12, -0.02, 0, -0.7, 0)
    
    orientation = np.array([pose, end_pose, second_viewing_angle, second_end_pose, third_viewing_angle, third_end_pose])
    direction = 1
    for i, pose_n in enumerate(orientation):
        
        robot_movement.move_robot(pose_n, True)
        robot_movement.group.set_max_velocity_scaling_factor(0.05)
        robot_movement.group.set_max_acceleration_scaling_factor(0.05)
        
        y_movement = 0.12 * direction
        first_movement = robot_movement.offset_movement(pose_n, 0, y_movement, 0, 0, 0, 0)
        
        robot_movement.scanning = True
        robot_movement.rate.sleep()
        
        while not robot_movement.move_robot(first_movement, True):
            robot_movement.rate.sleep()
        
        robot_movement.scanning = False
        
        point_clouds.append(copy.deepcopy(robot_movement.data_storage))
        robot_movement.data_storage.clear()
        robot_movement.first_scan_transform = None
        
        direction = -direction
        
     # Save scan data
    with open("cloud_data.pkl", "wb") as f:
            pickle.dump(point_clouds, f)
            
    return point_clouds

def segment_plane(pcd, distance_threshold=0.005, ransac_n=3, num_iterations=1000):
    """
    Plane segmentation to remove calibration board.
    - distance_threshold: Defines inlier points close to the plane.
    - ransac_n: Minimum points required for RANSAC.
    - num_iterations: Number of iterations for RANSAC.
    """
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                             ransac_n=ransac_n,
                                             num_iterations=num_iterations)
    
    plane_cloud = pcd.select_by_index(inliers)
    object_cloud = pcd.select_by_index(inliers, invert=True)  # Everything NOT in the plane
    
    return object_cloud, plane_cloud  # Return non-plane (sphere) and plane
    
def pointcloud_segmentation(scan):

    # pcd = o3d.io.read_point_cloud(scan)
    pcd = scan 
    
    # pcd = pcd.voxel_down_sample(voxel_size=0.001)
    # print("Inital Cloud")
    # o3d.visualization.draw_geometries([pcd])

    # Remove outliers
    pcd_filtered, ind = pcd.remove_statistical_outlier(nb_neighbors=5, std_ratio=1.5)

    pcd_sphere, pcd_plane = segment_plane(pcd_filtered)
        
    # print("Plane removed! Displaying remaining object (sphere).")
   
    # print("Additional Radius Removal! Displaying remaining object (sphere).")
    final_sphere, inliers = pcd_sphere.remove_radius_outlier(nb_points=7, radius=0.005)
    
    # Visualize the filtered cloud
    # print("Final Sphere")
    # o3d.visualization.draw_geometries([final_sphere])
    
    return final_sphere
    
    
def fit_sphere_pseudoinverse(points):
    """
    Solve for sphere center (x_k, y_k, z_k) using pseudo-inverse.
    """
    points = np.asarray(points.points)  # Convert Open3D PointCloud to NumPy
    X, Y, Z = points[:, 0], points[:, 1], points[:, 2]  # Extract coordinates

    # Construct matrices A and B
    A = np.column_stack((2 * X, 2 * Y, 2 * Z, np.ones(len(X))))
    B = X**2 + Y**2 + Z**2

    # Solve for X = (x_k, y_k, z_k, r^2 - x_k^2 - y_k^2 - z_k^2) using pseudo-inverse
    X_k = np.linalg.pinv(A) @ B

    # Extract center coordinates
    x_k, y_k, z_k = X_k[:3]
    r_squared = X_k[3] + x_k**2 + y_k**2 + z_k**2
    r = np.sqrt(r_squared)

    return np.array([x_k, y_k, z_k]), r

def filter_invalid_fits(sphere_centers, sphere_radii, expected_radius, threshold=0.5):
    """
    Remove sphere centers where the estimated radius deviates too much from the known radius.
    - sphere_centers: List of estimated sphere centers.
    - sphere_radii: List of estimated sphere radii.
    - expected_radius: The known calibration sphere radius.
    - threshold: Allowed deviation percentage (0.5 = 50% deviation allowed).
    """
    valid_centers = []
    for center, radius in zip(sphere_centers, sphere_radii):
        if abs(radius - expected_radius) / expected_radius < threshold:
            valid_centers.append(center)

    return np.array(valid_centers)

def translation_to_homogeneous(translation):
    """
    Create a 4x4 homogeneous transformation matrix from a translation vector.
    
    Parameters:
    translation (list or numpy array): A 3-element list or array representing the translation [tx, ty, tz].
    
    Returns:
    numpy array: A 4x4 homogeneous transformation matrix.
    """
    T = np.eye(4)  # Initialize as identity matrix
    T[:3, 3] = translation  # Set translation part
    return T

def transform_points_to_world(flange_world_transform, sensor_flange_transform, points):
        
        sensor_world_transform = np.dot(flange_world_transform, sensor_flange_transform)
        
        # print(f"points shape before hstack: {points.shape}")
        # print(f"ones shape: {np.ones((points.shape[0], 1)).shape}")
        
        # points = np.atleast_2d(points)
        
        # **Handle empty point clouds**
        if points.shape[0] == 0:
            return np.empty((0, 3))  # ✅ Return an empty (0,3) array
        
        # Convert points to homogeneous coordinates (Nx4)
        points_hom = np.hstack((points, np.ones((points.shape[0], 1))))  # Add 1 as the 4th dimension
        
        points_world_frame = np.dot(sensor_world_transform, points_hom.T)
        
        return points_world_frame.T[:,:3]
    
def create_open3d_point_cloud(position_vectors):
    """
    Convert a list of 3D position vectors into an Open3D PointCloud.

    Parameters:
    position_vectors (list of lists or numpy array): A list of 3D coordinates [[x1, y1, z1], [x2, y2, z2], ...]

    Returns:
    open3d.geometry.PointCloud: The resulting Open3D point cloud.
    """
    # Convert input to numpy array
    points = np.array(position_vectors, dtype=np.float64)

    # Create an Open3D PointCloud object
    point_cloud = o3d.geometry.PointCloud()

    # Assign points to the point cloud
    point_cloud.points = o3d.utility.Vector3dVector(points)

    return point_cloud


def post_process_data(data, sensor_flange_transform):
        """Preprocess laser scan data and filter outliers."""
        processed_scans = []
        for i, scan_pass in enumerate(data):
            # print(f"Processing scan pass: {i+1}.")
            world_points = []
            
            # print(f"scan size: {len(scan_pass)}")

            for scan in scan_pass:
                cloud = scan['cloud']
                offset_transform = scan['transform']
                
                cloud = np.array(cloud)
                if cloud.size == 0:
                    # print("Skipping empty cloud")
                    continue
                transformed = transform_points_to_world(offset_transform, sensor_flange_transform, cloud)
                if transformed.shape[0] == 0:
                    # print("Skipping empty transformed points")   
                    continue
                else:
                    world_points.extend(transformed)
            
            if world_points:  # ✅ Ensure we don't pass empty lists
                converted_points = create_open3d_point_cloud(world_points)
                processed_scans.append(converted_points)
            
        return processed_scans

def cost_function(sphere_centers):
    """
    Compute the sum of squared distances between sphere centers.
    - params: (dx, dy, dz) - translation correction for T_sensor→flange.
    - sphere_centers: List of estimated sphere centers from different views.
    """

    transformed_centers = sphere_centers

    # Compute sum of squared distances between each pair of centers
    cost = 0
    for i in range(len(transformed_centers)):
        for j in range(i+1, len(transformed_centers)):
            cost += np.linalg.norm(transformed_centers[i] - transformed_centers[j])**2

    return cost

def objective_function(params, cloud_list):
    
    total_centre_points = []
    total_radii = []
    expected_radius = 0.011
    
    translation_transform = translation_to_homogeneous(params)
    
    transformed_cloud = post_process_data(cloud_list, translation_transform)
    
    for cloud in transformed_cloud:
        sphere = pointcloud_segmentation(cloud)
        centre_point, radius = fit_sphere_pseudoinverse(sphere)
        total_centre_points.append(centre_point)
        total_radii.append(radius)
        
     # Filter out invalid sphere fits
    total_centre_points = np.array(total_centre_points)
    total_radii = np.array(total_radii)
    
    filtered_centre_points = filter_invalid_fits(total_centre_points, total_radii, expected_radius)
    # cost = cost_function(filtered_centre_points)
    cost = cost_function(filtered_centre_points)
    return cost

def sphere_optimisation():
    
    with open("cloud_data.pkl", "rb") as f:
        cloud_data = pickle.load(f)
        
    # print(f"Loaded {len(cloud_data[0])} scan in pass 0")
        
    # Optimize translation correction
    initial_guess = [0.025, 0.0, 0.050]
    
    # Solve using Trust-Region Reflective Algorithm
    result = least_squares(objective_function, x0=initial_guess, args=(cloud_data,), method='trf', verbose=2)

    # Extract optimized translation correction
    optimized_translation = result.x
    print(f"Optimized Sensor-to-Flange Translation: {optimized_translation}")
    print("Calibration complete!")
    
if __name__ == '__main__':
    try:
        rospy.init_node('robot_scanner', anonymous=True)
        
        scanner = RobotScanner()
        
        robot_scan(scanner)
        
        sphere_optimisation()
                
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

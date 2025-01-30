import rospy
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from scipy.optimize import least_squares

def ros_pointcloud2_to_open3d(ros_cloud):
    """ Convert ROS sensor_msgs/PointCloud2 to Open3D point cloud. """
    points_list = []

    for point in pc2.read_points(ros_cloud, field_names=("x", "y", "z"), skip_nans=True):
        points_list.append([point[0], point[1], point[2]])

    # Convert to Open3D format
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.array(points_list, dtype=np.float32))
    return cloud

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
    """ Callback function to process incoming PointCloud2 data """
    rospy.loginfo("Received PointCloud2 message")

    # Convert to Open3D point cloud
    # pcd = ros_pointcloud2_to_open3d(msg)
    
    # o3d.io.write_point_cloud("raw_cloud.pcd", pcd)
    
    pcd = o3d.io.read_point_cloud(scan)
    
    pcd = pcd.voxel_down_sample(voxel_size=0.0015)
    # Visualize the filtered cloud
    # o3d.visualization.draw_geometries([pcd])
    # Remove outliers
    pcd_filtered, ind = pcd.remove_statistical_outlier(nb_neighbors=5, std_ratio=1.5)
        
    # o3d.visualization.draw_geometries([pcd_filtered])

    pcd_sphere, pcd_plane = segment_plane(pcd_filtered)
        
    # # Save & visualize results
    # o3d.io.write_point_cloud("sphere_cloud.pcd", pcd_sphere)
    # o3d.io.write_point_cloud("plane_cloud.pcd", pcd_plane)

    print("Plane removed! Displaying remaining object (sphere).")
    # o3d.visualization.draw_geometries([pcd_sphere])
    # o3d.visualization.draw_geometries([pcd_plane])
    
    print("Additional Radius Removal! Displaying remaining object (sphere).")
    final_sphere, inliers = pcd_sphere.remove_radius_outlier(nb_points=5, radius=0.005)
    
    # Visualize the filtered cloud
    print("Final Sphere")
    o3d.visualization.draw_geometries([final_sphere])

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

def cost_function(params, sphere_centers):
    """
    Compute the sum of squared distances between sphere centers.
    - params: (dx, dy, dz) - translation correction for T_sensor→flange.
    - sphere_centers: List of estimated sphere centers from different views.
    """
    # Apply translation correction (params = [dx, dy, dz])
    transformed_centers = sphere_centers + params  

    # Compute sum of squared distances between each pair of centers
    cost = 0
    for i in range(len(transformed_centers)):
        for j in range(i+1, len(transformed_centers)):
            cost += np.linalg.norm(transformed_centers[i] - transformed_centers[j])**2

    return cost


def main():
    rospy.init_node("pointcloud2_open3d_converter", anonymous=True)
    # rospy.Subscriber("/assembled_cloud", PointCloud2, pointcloud_callback)
    # List of individual scans
    scan_files = ["scan_1.pcd", "scan_2.pcd", "scan_3.pcd", "scan_4.pcd", "scan_5.pcd"]  # 

    total_centre_points = []
    total_radii = []
    expected_radius = 0.027
    for scan in scan_files:
        sphere = pointcloud_segmentation(scan)
        centre_point, radius = fit_sphere_pseudoinverse(sphere)
        total_centre_points.append(centre_point)
        total_radii.append(radius)
    
    # Filter out invalid sphere fits
    total_centre_points = np.array(total_centre_points)
    total_radii = np.array(total_radii)
    
    filtered_centre_points = filter_invalid_fits(total_centre_points, total_radii, expected_radius)
    
    print(f"🔍 Debug: Number of sphere centers: {filtered_centre_points.shape[0]}")
    print(f"🔍 Debug: Sphere Centers Before Optimization:\n{filtered_centre_points}")

    # Optimize translation correction
    initial_guess = [0.001, 0.001, 0.001]
    
    # Solve using Trust-Region Reflective Algorithm
    result = least_squares(cost_function, x0=initial_guess, args=(filtered_centre_points,), method='trf', verbose=2)

    # Extract optimized translation correction
    optimized_translation = result.x
    print(f"Optimized Sensor-to-Flange Translation: {optimized_translation}")
    print("Calibration complete!")
if __name__ == "__main__":
    main()

import numpy as np
from scipy.optimize import least_squares
import os

dir_path = os.path.realpath(os.path.dirname(__file__))

# file_path1 = f"{dir_path}/data/matrix1.npy" 
# file_path2 = f"{dir_path}/data/scan1.npy"     

# T_F_W_list = np.load(file_path1)

# scans = np.load(file_path2)
# num_rows = scans.shape[0]
# orientation_scans = np.hstack((scans, np.ones((num_rows, 1))))

# Load all matrix files (matrix1.npy to matrix10.npy) into a list of NumPy arrays
matrix_files = [f"{dir_path}/updated_orientation/matrix{i}.npy" for i in range(1, 11)]
T_F_W_list = [np.load(file) for file in matrix_files]  # List of (N_i, 4, 4) arrays

# Load all scan files (scan1.npy to scan10.npy) into a list of NumPy arrays
scan_files = [f"{dir_path}/updated_orientation/scan{i}.npy" for i in range(1, 11)]
scans = [np.load(file) for file in scan_files]  # List of (N_i, 3) arrays

# Convert each scan to homogeneous coordinates (add extra column of ones)
orientation_scans = [np.hstack((scan, np.ones((scan.shape[0], 1)))) for scan in scans]  # List of (N_i, 4) arrays

reshaped_scans = [scan.reshape(scan.shape[0], 4, 1) for scan in orientation_scans]

# Print the new shape of each scan list
print("\nReshaped scan dimensions:")
for i, scan in enumerate(reshaped_scans):
    print(f"Scan {i+1} shape: {scan.shape}")  # Expected: (N_i, 1, 4, 1)

# Print info for verification
print(f"Loaded {len(T_F_W_list)} transformation matrix files.")
print(f"Loaded {len(scans)} scan files.")

for i in range(10):
    print(f"\nScan {i+1} shape: {scans[i].shape}")  # Expected: (N_i, 3)
    print(f"Matrix {i+1} shape: {T_F_W_list[i].shape}")  # Expected: (N_i, 4, 4)
    print(f"Orientation Scan {i+1} shape: {orientation_scans[i].shape}")  # Expected: (N_i, 4)

# Print the first scan value from each scan list
print("\nFirst scan value from each scan list:")
for i, scan in enumerate(orientation_scans):
    if scan.shape[0] > 0:  # Check if there is data in the scan
        print(f"Scan {i+1}: {scan[0]}")
    else:
        print(f"Scan {i+1}: [EMPTY] No scan data available.")

# Homogeneous transformation utilities
def homogeneous_transform(rotation, translation):
    """Create a 4x4 homogeneous transformation matrix."""
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T

def rotation_matrix_from_euler(rx, ry, rz):
    """Convert Euler angles to rotation matrix (XYZ order)."""
    Rx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

# Orientational Calibration
def fit_plane(points):
    """Fit a plane to 3D points using least squares."""
    A = np.c_[points[:3, :].T, np.ones(points.shape[1])]
    b = points[2, :]
    coeffs, _, _, _ = np.linalg.lstsq(A[:, :-1], b, rcond=None)
    return coeffs  # [a, b, d] for plane ax + by + z = d

def orientation_cost(params, scans, T_F_W_list):
    """Cost function for orientation optimization."""
    rx, ry, rz = params
    R = rotation_matrix_from_euler(rx, ry, rz)
    T_S_F = homogeneous_transform(R, np.zeros(3))  # Translation not optimized here
    
    total_cost = 0
    for i, (scan, T_F_W) in enumerate(zip(scans, T_F_W_list)):
        # Transform points to world frame
        print(scan[0])
        points_world = T_F_W @ T_S_F @ scan
        coeffs = fit_plane(points_world)
        a, b = coeffs[:2]
        d = coeffs[2]
        errors = a * points_world[0, :] + b * points_world[1, :] + points_world[2, :] - d
        cost = np.sqrt(np.sum(errors**2))
        total_cost += cost
    return total_cost

def calibrate_orientation(scans, T_F_W_list, initial_guess = [0.1, 0.1, 0.1]):
    """Calibrate the rotational part of T_S_F."""
    bounds = ([-np.pi/4, -np.pi/4, -np.pi/4], [np.pi/4, np.pi/4, np.pi/4])
    result = least_squares(orientation_cost, initial_guess, args=(scans, T_F_W_list),
                          method='trf', bounds=bounds)
    rx, ry, rz = result.x
    R = rotation_matrix_from_euler(rx, ry, rz)
    print(R)
    return homogeneous_transform(R, np.zeros(3))

# def calibrate_orientation(scans, T_F_W_list, initial_guess=None):
#     """Calibrate the rotational part of T_S_F."""
#     if initial_guess is None:
#         # Use a deterministic or bounded random initial guess within bounds
#         initial_guess = np.random.uniform(-np.pi/4, np.pi/4, 3)  # Random values between -π/4 and π/4
#         # Alternatively, use a fixed starting point (e.g., zeros or small angles)
#         # initial_guess = np.zeros(3)  # Or np.array([0.1, 0.1, 0.1]) for small angles
#     bounds = ([-np.pi/4, -np.pi/4, -np.pi/4], [np.pi/4, np.pi/4, np.pi/4])
#     result = least_squares(orientation_cost, initial_guess, args=(scans, T_F_W_list),
#                           method='trf', bounds=bounds)
#     rx, ry, rz = result.x
#     R = rotation_matrix_from_euler(rx, ry, rz)
#     return homogeneous_transform(R, np.zeros(3))




# # Main execution
if __name__ == "__main__":
    
    # Orientation calibration
    T_S_F_rot = calibrate_orientation(reshaped_scans, T_F_W_list)
    print("Orientation T_S_F:\n", T_S_F_rot)
    
 
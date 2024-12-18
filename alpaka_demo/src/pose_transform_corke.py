import spatialmath as sm
import numpy as np
import math
import matplotlib.pyplot as plt


if __name__ == '__main__':
    
    robot_pose = []
    robot_pose.append(-0.56255)
    robot_pose.append(-0.58127)
    robot_pose.append(0.71202)
    
    robot_positions = [
        [-0.56255, -0.58127, 0.71202],
        [-0.57194, -0.58087, 0.72249],
        [-0.56942, -0.57740, 0.71264],
        [-0.55281, -0.57039, 0.71077]
    ]
    rot_x = 0.238
    rot_y = 0.410
    rot_z = -0.077
    robot_rotation = [
        [0.238, 0.410, -0.077],
        [0.090, 0.553, 0.176],
        [3.313, -3.686, -2.940],
        [0.325, -4.060, -3.769]
    ]
    
    sensor_points = [
    [-0.016685, 0, 0.11943],
    [-0.00503, 0, 0.12149],
    [0.001007, 0, 0.14388],
    [0.0083427, 0, 0.1289]
    ]
    
    # robot_rpy = [rot_z, rot_y, rot_x]
    
    robot_rpy = [rot_x, rot_y, rot_z]
    
    #First method: Point is from sensor to point
    point_x = 0.01668
    point_x = -point_x #x axis is inversed on sensor plane
    point_y = 0 #only scans along x,z plane
    point_z = 0.1194
    point_z = point_z
    point_scanned = [point_x, point_y, point_z]
    
     #First attempt where the offset is from TCP frame to sensor
    x_rot_angle = 16.1
    offset_x = 0
    offset_y = 0.07268 #updated based on real calculation
    # offset_z = -0.11511 #calculated from offset on plan diagram
    offset_z = 10**-3 * -122.69 * math.cos(np.deg2rad(x_rot_angle)) # height offset based on testing
    print("Calculated z offset:", offset_z)
    
    sensor_to_TCP_translation = [offset_x, offset_y, offset_z]
    
    # R1 = sm.base.trotx(theta=np.deg2rad(x_rot_angle), t=sensor_to_TCP_translation)
    # print(R1)
    
    # rotated_point = sm.base.homtrans(R1,point_scanned) 
    # print("Point calculated from Homogenous Transformation:", rotated_point)
    
    # point_difference = [robot_pose[0]+rotated_point[0], robot_pose[1]+rotated_point[1],robot_pose[2]+rotated_point[2]]

    R_base = sm.SE3()
    TCP_Pose = sm.SE3(robot_pose) * sm.base.eul2tr(rot_z, rot_y, rot_x)
    Sensor_Frame = TCP_Pose * sm.SE3(sm.base.trotx(theta=x_rot_angle, unit='deg', t=sensor_to_TCP_translation))
    # Create a single plot for both frames
    fig, axs = plt.subplots(2,2,subplot_kw={'projection': '3d'},figsize=(10, 10))
    axs = axs.flatten()  # Flatten the 2D axes array to make it easier to index
    colors = ['green', 'red', 'blue', 'black']
    
    for i, (robot_pose, robot_rpy, point_scanned, color) in enumerate(zip(robot_positions, robot_rotation, sensor_points, colors)):
        # Adjust sensor point for x-axis inversion
        print(f"DATA {i}: ")
        point_scanned[0] = -point_scanned[0]  # x-axis inversion
        R1 = sm.base.trotx(theta=np.deg2rad(x_rot_angle), t=sensor_to_TCP_translation)
        rotated_point = sm.base.homtrans(R1,point_scanned) 
        print("Point offset relative to tcp:", rotated_point)
        point_difference = [robot_pose[0]+rotated_point[0], robot_pose[1]+rotated_point[1],robot_pose[2]+rotated_point[2]]
        print("Added offset from tcp position:",point_difference)
        
        # Define TCP pose: position and RPY rotation
        TCP_Pose = sm.SE3(robot_pose) * sm.base.eul2tr(robot_rpy[2], robot_rpy[1], robot_rpy[0], unit='rad')
        Sensor_Frame = TCP_Pose * sm.SE3(sm.base.trotx(theta=x_rot_angle, unit='deg', t=sensor_to_TCP_translation))
        transformed_point = sm.base.homtrans(Sensor_Frame,point_scanned) 

        # Plot robot TCP frame and transformed sensor point
        sm.base.trplot(Sensor_Frame, frame=f'Sensor{i+1}', rviz=True, width=1, color='blue', dims=[-1, 1, -1, 1, -1, 1], ax=axs[i])
        sm.base.trplot(TCP_Pose, frame=f'TCP{i+1}', rviz=True, width=1, color='green', ax=axs[i])
        axs[i].scatter(transformed_point[0], transformed_point[1], transformed_point[2], c='black', marker='o', label=f"Point {i+1}")
        print(f"Transformed point {i+1}: {transformed_point}")
        print(f"Error from estimated point: x= {transformed_point[0]-(-0.57378)}, y= {transformed_point[1]-(-0.54244)}, z={transformed_point[2]-(0.72446)}")
        
        axs[i].scatter(-0.57378, -0.54244, 0.72446, c='black', marker='x', label=f"Approx Point")
        sm.base.trplot( np.eye(3), frame='Base', rviz=True, width=2, color='red', dims=[-1, 1, -1, 1, -1, 1], ax=axs[i])

        axs[i].set_xlabel('X')
        axs[i].set_ylabel('Y')
        axs[i].set_zlabel('Z')
        axs[i].legend()
    
    
    # sm.base.trplot( R1, frame='B', rviz=True, width=2, color='blue', dims=[-1, 1, -1, 1, -1, 1])
    # sm.base.trplot( np.eye(3), frame='A', rviz=True, width=2, color='red')
    # sm.base.trplot( np.eye(3), frame='Base', rviz=True, width=2, color='red', dims=[-1, 1, -1, 1, -1, 1])
    # sm.base.trplot( TCP_Pose, frame='TCP', rviz=True, width=2, color='blue')
    # sm.base.trplot( Sensor_Frame, frame='Sensor', rviz=True, width=2, color='green')
    # ax.scatter(point_difference[0], point_difference[1], point_difference[2], color='pink', s=50, label='Transformed Point')    # R = sm.base.eul2r(rot_z, rot_y, rot_x, unit='rad')
    # axs.scatter(-0.57378, -0.54244, 0.72446, c=color, marker='x', label=f"Approx Point")
    plt.tight_layout()  # Adjust subplots for better spacing
    plt.show()
    # print(R)
    
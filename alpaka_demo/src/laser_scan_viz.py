#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import math

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

class LaserScanPlotter:
    def __init__(self):
        rospy.init_node('laser_scan_plotter', anonymous=True)
        self.scan_data = None
        rospy.Subscriber('/laser_scan', LaserScan, self.laser_callback)
        plt.ion()  # Enable interactive mode
        self.figure, self.ax = plt.subplots(figsize=(10, 6))  # Set figure size (width, height in inches)
        self.cbar = None  # Placeholder for colorbar

    def laser_callback(self, data):
        self.scan_data = data

    def plot_scan(self):
        while not rospy.is_shutdown():
            if self.scan_data:
                # Extract ranges and angles
                ranges = self.scan_data.ranges
                angle_min = self.scan_data.angle_min
                angle_increment = self.scan_data.angle_increment
                angles = [angle_min + i * angle_increment for i in range(len(ranges))]
                x = [ranges[i]*math.sin(angles[i]) for i in range(len(ranges))]
                y = [ranges[i]*math.cos(angles[i]) for i in range(len(ranges))]
                # Clear the plot and plot new data
                self.ax.clear()
                # Create scatter plot with color-coded distances
                self.ax.scatter(x, y, c=ranges, s=10, label="LaserScan Data")
                
                self.ax.set_xlabel("x (meters)")
                self.ax.set_ylabel("y (meters)")
                self.ax.set_title("LaserScan Data Visualization")
                    
                self.ax.legend()
                self.ax.set_xlim([-0.035, 0.035])  # Example: angles from -pi to pi
                self.ax.set_ylim([0.070, 0.160])         # Example: distances from 0 to 10 meters
                self.ax.invert_yaxis()
                
                 # Add grid for better visualization
                self.ax.grid(True)

                plt.pause(0.1)  # Small delay for updating the plot
                

if __name__ == '__main__':
    try:
        plotter = LaserScanPlotter()
        plotter.plot_scan()
    except rospy.ROSInterruptException:
        pass
    
    
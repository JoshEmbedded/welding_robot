#include <ros/ros.h>
#include <rosbag/bag.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include "headers/calibrationClass.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_plan");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);

    spinner.start(); 

    // Set up the MoveIt! MoveGroup interface for the UR5e robot
    static const std::string PLANNING_GROUP = "manipulator"; // Replace with your robot's planning group
    LaserCalibration Calibration(PLANNING_GROUP, node_handle);

    bool success = Calibration.sensorCalibration();

    if (success)
    {
        ROS_INFO("Calibration Movement Successful");
    }

    else{
        ROS_INFO("Calibration Movement Failed");
    }
    spinner.stop();
    ros::shutdown();
    return 0;
}

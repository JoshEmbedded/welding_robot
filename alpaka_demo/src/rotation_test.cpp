#include <ros/ros.h>
#include <rosbag/bag.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <string>
#include "headers/calibrationClass.h"

// Function to detect all ridge tips in LaserScan data
std::vector<int> detectAllRidgeTips(const sensor_msgs::LaserScan& scan) {
    std::vector<int> ridge_indices;

    // Check if there is sufficient data
    if (scan.ranges.size() < 3) {
        std::cerr << "Not enough scan points to detect ridge tips." << std::endl;
        return ridge_indices; // Return an empty vector
    }

    // Loop through the ranges to find all local maxima
    for (size_t i = 1; i < scan.ranges.size() - 1; ++i) {
        float prev = scan.ranges[i - 1];
        float curr = scan.ranges[i];
        float next = scan.ranges[i + 1];

        // Ignore invalid or out-of-range values
        if (std::isinf(prev) || std::isinf(curr) || std::isinf(next)) continue;

        // Check for a local maximum (ridge tip)
        if (curr < prev && curr < next) {
            if (abs(prev-curr)>0.0001)
            {
                ridge_indices.push_back(i); // Store the index of the ridge tip
                ROS_INFO("Ridge Detected at scan index: %i ", i);
            }
        }
    }

    // If no ridge tips are found, print a message
    if (ridge_indices.empty()) {
        std::cerr << "No ridge tips detected." << std::endl;
    }

    return ridge_indices;
}

// Function to calculate the average of LaserScan ranges
float calculateAverageRange(const sensor_msgs::LaserScan& scan) {
    float sum = 0.0;
    int valid_count = 0;

    // Iterate through the ranges
    for (const float& range : scan.ranges) {
        // Skip invalid or out-of-range values
        if (std::isinf(range) || std::isnan(range) || range < scan.range_min || range > scan.range_max) {
            continue;
        }
        sum += range;
        ++valid_count;
    }

    // Check if there are any valid ranges
    if (valid_count == 0) {
        std::cerr << "No valid range data available to calculate average." << std::endl;
        return std::nan(""); // Return NaN if no valid ranges are available
    }

    return sum / valid_count;
}


void calibrate_z_rot(LaserCalibration& robot){
    
    geometry_msgs::Pose start_pose;

    start_pose.position.x = 0.0;
    start_pose.position.y = 0.4;
    start_pose.position.z = 0.06;
    start_pose.orientation.x = 1;
    start_pose.orientation.y = 0;
    start_pose.orientation.z = 0;
    start_pose.orientation.w = 0;

    if(robot.robotMovement(start_pose)){
        ROS_INFO("First Movement Completed.");
    }

    robot.offsetMovement(start_pose, 0, 0.03, 0, 0, 0, 0);
    robot.robotMovement(start_pose);

    int ridge_num = 0;
    float angle = 0;

    sensor_msgs::LaserScan laser_scan = robot.getLaserScan();
    ridge_num = detectAllRidgeTips(laser_scan).size();

    geometry_msgs::PoseStamped first_ridge;
    geometry_msgs::PoseStamped second_ridge;

    while(ridge_num<3 && ros::ok()){
        angle -=0.05;
        robot.offsetMovement(start_pose, 0, 0, 0, 0, 0, -0.05);
        robot.robotMovement(start_pose);
        sensor_msgs::LaserScan laser_scan = robot.getLaserScan();
        ridge_num = detectAllRidgeTips(laser_scan).size();
    }
    angle -=0.05;
    robot.offsetMovement(start_pose, 0, 0, 0, 0, 0, -0.02);
    robot.robotMovement(start_pose);
    while(ridge_num>1 && ros::ok()){
        robot.offsetMovement(start_pose, 0, 0.05, 0, 0, 0, 0);
        robot.robotMovement(start_pose);
        sensor_msgs::LaserScan laser_scan = robot.getLaserScan();
        ridge_num = detectAllRidgeTips(laser_scan).size();
    }
    bool aligned = false;
    int prev_scan = 0;
    float movement = 0;
    while(!aligned && ros::ok()){
        sensor_msgs::LaserScan laser_scan = robot.getLaserScan();
        ridge_num = detectAllRidgeTips(laser_scan).size();

        if ((ridge_num==3) && (prev_scan == 1)){
            aligned = true;
            break;
        }

        if ((ridge_num==2) && (prev_scan ==1)){
            first_ridge = robot.move_group.getCurrentPose();
        }

        if ((ridge_num==3) && (prev_scan == 2)){
            second_ridge = robot.move_group.getCurrentPose();
            float x_diff = first_ridge.pose.position.x-second_ridge.pose.position.x;
            float y_diff = first_ridge.pose.position.y-second_ridge.pose.position.y;
            float angle = atan(x_diff/y_diff);
            ROS_INFO("Angle difference calculated: %f", angle);
            robot.offsetMovement(start_pose, 0, movement, 0, 0, 0, angle);
            robot.robotMovement(start_pose);
            movement = 0;
        }

        robot.offsetMovement(start_pose, 0, -0.001, 0, 0, 0, 0);
        robot.robotMovement(start_pose);

        movement +=0.001;
        prev_scan = ridge_num;
    }

    std::vector<double> rpy_z_correct = robot.move_group.getCurrentRPY();
    geometry_msgs::PoseStamped pose_z_correct = robot.move_group.getCurrentPose();
}

void calibrate_y_rot(LaserCalibration& robot)
{
    // geometry_msgs::Pose pose = robot.move_group.getCurrentPose();
}

float detectGroove(LaserCalibration& robot, geometry_msgs::Pose& pose, float movement, float threshold)
{
    sensor_msgs::LaserScan laser_scan;
    bool ridge_found = false;
    float horizontal_movement = 0;

    robot.offsetMovement(pose, 0, movement, 0, 0, 0, 0);
    while(!robot.robotMovement(pose)){
        ros::Duration(0.5).sleep();
    }
    laser_scan = robot.getLaserScan();
    float prev = calculateAverageRange(laser_scan);
    horizontal_movement += movement;

    robot.offsetMovement(pose, 0, movement, 0, 0, 0, 0);
    while(!robot.robotMovement(pose)){
        ros::Duration(0.5).sleep();
    }
    laser_scan= robot.getLaserScan();
    float current = calculateAverageRange(laser_scan);
    horizontal_movement += movement;

    float next = 0;

    while(!ridge_found && ros::ok())
    {
        robot.offsetMovement(pose, 0, movement, 0, 0, 0, 0);
        while(!robot.robotMovement(pose))
        {
            ros::Duration(0.5).sleep();
        }
        
        horizontal_movement += movement;
        laser_scan = robot.getLaserScan();
        int ridges = detectAllRidgeTips(laser_scan).size();

        next = calculateAverageRange(laser_scan);
        ROS_INFO("Scan Difference: %f", next-current);
        // if (next - current > 0.0009){
        //     ridge_found = true;
        //     ROS_INFO("Calibration Groove Found with avg. distance: %f", current);
        //     break;
        // }
        ROS_INFO("Previous: %f, Current: %f, Next: %f", prev, current, next);
        if (prev < current && current > next){
            if ((abs(current-prev)>threshold || abs(current-next)>threshold) && ridges == 0){
                robot.offsetMovement(pose, 0, -movement, 0, 0, 0, 0);
                while(!robot.robotMovement(pose)){
                    ros::Duration(0.5).sleep();
                }
                horizontal_movement += -movement;
                ridge_found = true;
                ROS_INFO("Calibration Groove Found with avg. distance: %f", current);
                break;
            }      
        }
        prev = current;
        current = next;
    }
    return horizontal_movement;
}

void calibrate_x_rot(LaserCalibration& robot)
{
    geometry_msgs::Pose start_pose;

    start_pose.position.x = 0.0;
    start_pose.position.y = 0.43;
    start_pose.position.z = 0.03;
    start_pose.orientation.x = 1;
    start_pose.orientation.y = 0;
    start_pose.orientation.z = 0;
    start_pose.orientation.w = 0;

    if(robot.robotMovement(start_pose)){
        ROS_INFO("First Movement Completed.");
    }
    
    geometry_msgs::Pose pose = start_pose;

    robot.offsetMovement(pose, 0, 0.001, 0, 0, 0, -1.57);
    robot.robotMovement(pose);

    detectGroove(robot, pose, 0.0005, 0.0003);
    ros::Duration(1.0).sleep();

    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Wait for Enter

    std::cout << "Continuing the program..." << std::endl;

    float horizontal_movement = 0;

    float vertical_distance = 0.05;
    robot.offsetMovement(pose, 0, 0, vertical_distance, 0, 0, 0);
    while(!robot.robotMovement(pose)){
        ros::Duration(0.5).sleep();
    }
    ros::Duration(1.0).sleep();

    horizontal_movement = detectGroove(robot, pose, -0.0005, 0.0002);
    ROS_INFO("First loop of horizontal scan completed");
    ros::Duration(1.0).sleep();

    float angle = atan(horizontal_movement/vertical_distance);
    ROS_INFO("Calculated X-Rotation: %f", angle);
}

void alt_calibrate_x_rot(LaserCalibration& robot)
{
    geometry_msgs::Pose start_pose;

    start_pose.position.x = 0.0;
    start_pose.position.y = 0.45;
    start_pose.position.z = 0.03;
    start_pose.orientation.x = 1;
    start_pose.orientation.y = 0;
    start_pose.orientation.z = 0;
    start_pose.orientation.w = 0;

    geometry_msgs::Pose pose = start_pose;

    robot.offsetMovement(pose, 0, 0.001, 0, 0, 0, -1.57);
    robot.robotMovement(pose);

    robot.offsetMovement(pose, 0, 0.05, 0, 0, 0, 0);
    robot.recordCalibration(pose);
    if(!robot.processScan()){
        ROS_INFO("Failed ROS BAG Analysis");
    }
    robot.jointMovement(robot.getJointScan());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_plan");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(2);

    spinner.start(); 

    // Set up the MoveIt! MoveGroup interface for the UR5e robot
    static const std::string PLANNING_GROUP = "manipulator"; // Replace with your robot's planning group
    LaserCalibration robot(PLANNING_GROUP, node_handle);
    robot.move_group.setMaxAccelerationScalingFactor(0.1);
    robot.move_group.setWorkspace (-2.0, -2.0, 0.01, 2.0, 2.0, 2.0);
    // calibrate_z_rot(robot);
    // calibrate_x_rot(robot);
    alt_calibrate_x_rot(robot);

    spinner.stop();
    ros::shutdown();
    return 0;
}

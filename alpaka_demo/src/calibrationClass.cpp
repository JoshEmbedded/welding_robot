#include "headers/calibrationClass.h"

LaserCalibration::LaserCalibration(const std::string &group_name, ros::NodeHandle node_handle)
    : move_group(group_name), nh(node_handle), bag_open(false)
{
    scan_sub = nh.subscribe("laser_scan", 10, &LaserCalibration::laserScanCallback, this);
    joint_sub = nh.subscribe("joint_states", 10, &LaserCalibration::jointStateCallback, this);
}

bool LaserCalibration::handlePlanError(const moveit::core::MoveItErrorCode &my_plan, const std::string planning = "planning")
{
    switch (my_plan.val)
    {
    case moveit::core::MoveItErrorCode::SUCCESS:
        if (planning == "planning")
        {
            ROS_INFO("Planning successful!");
        }
        else
        {
            ROS_INFO("Path Executed Successfully...");
        }
        return true;
    case moveit::core::MoveItErrorCode::PLANNING_FAILED:
        ROS_ERROR("Error: Planning failed.");
        break;
    case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN:
        ROS_ERROR("Error: Invalid motion plan.");
        break;
    case moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        ROS_ERROR("Error: Motion plan invalidated by environment change.");
        break;
    case moveit::core::MoveItErrorCode::CONTROL_FAILED:
        ROS_ERROR("Error: Control failed.");
        break;
    case moveit::core::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
        ROS_ERROR("Error: Unable to acquire sensor data.");
        break;
    case moveit::core::MoveItErrorCode::TIMED_OUT:
        ROS_ERROR("Error: Timed out.");
        break;
    case moveit::core::MoveItErrorCode::PREEMPTED:
        ROS_ERROR("Error: Preempted.");
        break;
    case moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION:
        ROS_ERROR("Error: Start state in collision.");
        break;
    case moveit::core::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        ROS_ERROR("Error: Start state violates path constraints.");
        break;
    case moveit::core::MoveItErrorCode::GOAL_IN_COLLISION:
        ROS_ERROR("Error: Goal in collision.");
        break;
    case moveit::core::MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS:
        ROS_ERROR("Error: Goal violates path constraints.");
        break;
    case moveit::core::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED:
        ROS_ERROR("Error: Goal constraints violated.");
        break;
    case moveit::core::MoveItErrorCode::INVALID_GROUP_NAME:
        ROS_ERROR("Error: Invalid group name.");
        break;
    case moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS:
        ROS_ERROR("Error: Invalid goal constraints.");
        break;
    case moveit::core::MoveItErrorCode::INVALID_ROBOT_STATE:
        ROS_ERROR("Error: Invalid robot state.");
        break;
    case moveit::core::MoveItErrorCode::INVALID_LINK_NAME:
        ROS_ERROR("Error: Invalid link name.");
        break;
    case moveit::core::MoveItErrorCode::INVALID_OBJECT_NAME:
        ROS_ERROR("Error: Invalid object name.");
        break;
    default:
        ROS_ERROR("Error: Unknown error.");
        break;
    }
    return false;
}

bool LaserCalibration::computeTrajectory(const geometry_msgs::PoseStamped &goal_pose, moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    move_group.setPoseTarget(goal_pose);
    return handlePlanError(move_group.plan(plan), "planning");
}

bool LaserCalibration::robotMovement(const geometry_msgs::Pose &goal_pose)
{
    move_group.setPoseTarget(goal_pose);
    if (handlePlanError(move_group.plan(plan), "planning"))
    {
        if (handlePlanError(move_group.execute(plan), "execution"))
        {
            return true;
        }
    }
    return false;
}

bool LaserCalibration::cartesianMovement(std::vector<geometry_msgs::Pose> &goal_poses, moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(goal_poses, 0.01, 0.0, trajectory);
    ROS_INFO("Cartesian path achieved: %.2f%%", fraction * 100.0);
    plan.trajectory_ = trajectory;
    return fraction > 0.8 && handlePlanError(move_group.plan(plan), "planning") && handlePlanError(move_group.execute(plan), "execution");
}

geometry_msgs::Pose LaserCalibration::offsetMovement(geometry_msgs::Pose &pose, float X, float Y, float Z, float w, float x, float y, float z)
{
    pose.position.x += X;
    pose.position.y += Y;
    pose.position.z += Z;
    pose.orientation.w += w;
    pose.orientation.x += x;
    pose.orientation.y += y;
    pose.orientation.z += z;
    return pose;
}

bool LaserCalibration::sensorCalibration()
{
    std::vector<geometry_msgs::Pose> poses;
    geometry_msgs::Pose start_pose;

    start_pose.position.x = 0.0;
    start_pose.position.y = 0.4;
    start_pose.position.z = 0.04;
    start_pose.orientation.x = 1;
    start_pose.orientation.y = 0;
    start_pose.orientation.z = 0;
    start_pose.orientation.w = 0;

    robotMovement(start_pose);
    ROS_INFO("First Movement Completed.");
    geometry_msgs::Pose offset_pose = start_pose;
    // lower eff into vulcram
    offsetMovement(offset_pose, 0, 0, -0.025, 0, 0, 0, 0);
    robotMovement(offset_pose);
    ros::Duration(1.0).sleep(); // Wait before checking again

    // //lift eff from vulcram.
    offsetMovement(offset_pose, 0, 0, 0.025, 0, 0, 0, 0);
    robotMovement(offset_pose);
    ros::Duration(1.0).sleep(); // Wait before checking again

    // //move in y direction
    offsetMovement(offset_pose, 0.08, 0, 0, 0, 0, 0, 0);
    robotMovement(offset_pose);
    ros::Duration(1.0).sleep(); // Wait before checking again

    // ROS_INFO("Entered sensor calibration before record");
    offsetMovement(offset_pose, -0.16, 0, 0, 0, 0, 0, 0);
    recordCalibration(offset_pose);
    ros::Duration(1.0).sleep(); // Wait before checking again

    return true;
    // return cartesianMovement(poses, plan);
}

void LaserCalibration::recordCalibration(geometry_msgs::Pose pose)
{
    // Resolve the package path
    std::string bag_path;
    std::string package_path = ros::package::getPath("alpaka_demo");
    if (package_path.empty())
    {
        ROS_ERROR("Package 'alpaka_demo' not found!");
        bag_path = "calibration_data.bag";
    }
    else
    {
        // Append the relative path to the bag file
        bag_path = package_path + "/include/bag/calibration_data.bag";
    }
    
    ROS_INFO("Bag file path: %s", bag_path.c_str());
    
    ros::Duration(0.5).sleep(); // Wait before checking again
    geometry_msgs::PoseStamped tcp_pose = move_group.getCurrentPose();

    // Print position
    ROS_INFO("Position: x = %.3f, y = %.3f, z = %.3f", 
            tcp_pose.pose.position.x, 
            tcp_pose.pose.position.y, 
            tcp_pose.pose.position.z);

    // Print orientation (quaternion)
    ROS_INFO("Orientation: x = %.3f, y = %.3f, z = %.3f, w = %.3f", 
            tcp_pose.pose.orientation.x, 
            tcp_pose.pose.orientation.y, 
            tcp_pose.pose.orientation.z, 
            tcp_pose.pose.orientation.w);
    move_group.startStateMonitor(); // initalise for faster state request.
    move_group.setPoseTarget(pose);
    bool plan_success = handlePlanError(move_group.plan(plan));
    try
    {
        bag.open(bag_path, rosbag::bagmode::Write);
        ROS_INFO("Bag file opened successfully for writing.");
        
        if (plan_success)
        {
            ros::Duration(0.05).sleep(); // Wait before checking again
            move_group.execute(plan);
        }
        else
        {
            ROS_ERROR("Calibration scan failed.");
        }
        std::lock_guard<std::mutex> lock(bag_mutex);
        bag.close();
        ROS_INFO("Bag Closed");
    } catch (const rosbag::BagException& e) {
        ROS_ERROR("Failed to open bag file: %s", e.what());
    }
}

void LaserCalibration::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(bag_mutex); // Ensure thread-safe access to the bag
    if (bag.isOpen())
    {
        bag.write("laser_scan", msg->header.stamp, *msg);
    }
}

void LaserCalibration::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(bag_mutex); // Ensure thread-safe access to the bag
    if (bag.isOpen())
    {
        bag.write("joint_states", msg->header.stamp, *msg);
    }
}



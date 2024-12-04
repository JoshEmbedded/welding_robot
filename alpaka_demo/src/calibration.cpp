#include <ros/ros.h>
#include <rosbag/bag.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

// Open a bag for writing
rosbag::Bag bag;


bool handlePlanError(const moveit::core::MoveItErrorCode& my_plan, 
                     std::string planning="planning")
{
    ROS_INFO("Handling error code: %d", my_plan.val); // Log the error code value
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

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // bag.write("laser_scan", msg->header.stamp, *msg);
    sensor_msgs::LaserScan scan = *msg;
}

void recordCalibration(ros::NodeHandle &nh,
                       moveit::planning_interface::MoveGroupInterface &move_group,
                       moveit::planning_interface::MoveGroupInterface::Plan &plan,
                       geometry_msgs::Pose pose)
{
    
    bag.open("movement_data.bag", rosbag::bagmode::Write);
    ROS_INFO("Bag is open");
    move_group.setPoseTarget(pose);

    // Plan to the target pose
    bool planning_success = handlePlanError(move_group.plan(plan), "planning");

    if (planning_success)
    {
        // Start execution asynchronously
        
        move_group.asyncExecute(plan);

        ROS_INFO("Robot scanning...");

        ros::Subscriber scan_sub = nh.subscribe("laser_scan", 10, laserScanCallback);
       
        // Monitor the robot's execution
        while (!move_group.getMoveGroupClient().waitForResult(ros::Duration(0.1)))
        {
            ROS_INFO("In async while loop");
            // Write TCP pose data

            geometry_msgs::PoseStamped tcp_pose = move_group.getCurrentPose();
            bag.write("tcp_eff_pose", tcp_pose.header.stamp, tcp_pose);
        }

        scan_sub.shutdown();
        ROS_INFO("Movement finished.");
    }
    else
    {
        ROS_ERROR("Failed to plan movement.");
    }

    // Close the bag file after the movement is complete
    bag.close();

    // ROS_INFO("Recording finished. Bag file saved.");
}




bool computeTrajectory(moveit::planning_interface::MoveGroupInterface &move_group,
                       const geometry_msgs::PoseStamped &goal_pose,
                       moveit::planning_interface::MoveGroupInterface::Plan &plan)
{


    move_group.setPoseTarget(goal_pose, "welding_eff_link");

    // Plan to the target pose
    bool success = handlePlanError(move_group.plan(plan), "planning");

    if (success)
    {
        ROS_INFO("Trajectory planning succeeded.");
    }
    else
    {
        ROS_WARN("Trajectory planning failed.");
    }

    return success;
}

bool robotMovement(moveit::planning_interface::MoveGroupInterface &move_group,
                       const geometry_msgs::Pose &goal_pose,
                       moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    move_group.setPoseTarget(goal_pose);
    ROS_INFO("starting plan");
    
    moveit::core::MoveItErrorCode planning_result = move_group.plan(plan);

    // Check if planning was successful
    if (planning_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Planning successful, executing...");
        
        // Execute the planned trajectory
        moveit::core::MoveItErrorCode execution_result = move_group.execute(plan);
        
        if (execution_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Movement successful.");
            return true;
        }
        else
        {
            ROS_ERROR("Movement execution failed.");
        }
    }
    else
    {
        ROS_ERROR("Planning failed.");
    }

    return false;   
}

bool cartesianMovement(moveit::planning_interface::MoveGroupInterface &move_group,
                        std::vector<geometry_msgs::Pose> &goal_poses,
                       moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    move_group.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    bool avoid_collisions = true;

    // Create a MoveItErrorCodes object
    moveit::core::MoveItErrorCode error_code;

    double fraction = move_group.computeCartesianPath(goal_poses, eef_step, jump_threshold, trajectory, avoid_collisions);
    ROS_INFO_NAMED("ur5e_laser", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // Set the trajectory in the plan
    plan.trajectory_ = trajectory;

    if (handlePlanError(move_group.plan(plan),"planning"))
    {
        move_group.execute(plan);
        return true;
    }
    
    return false;
}

geometry_msgs::Pose offsetMovement(geometry_msgs::Pose &pose, float X, float Y, float Z, float w, float x, float y, float z)
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

// bool sensorCalibration(moveit::planning_interface::MoveGroupInterface &move_group,
//                        moveit::planning_interface::MoveGroupInterface::Plan &plan)
// {
//     std::vector<geometry_msgs::Pose> poses;
//     geometry_msgs::Pose vulcram_pose;

//     // create a fake vulcram pose and lift above
//     vulcram_pose.position.x = 0;
//     vulcram_pose.position.y = 0.4;
//     vulcram_pose.position.z = 0.04;
//     vulcram_pose.orientation.x = 1;
//     vulcram_pose.orientation.y = 0;
//     vulcram_pose.orientation.z = 0;
//     vulcram_pose.orientation.w = 0;
//     poses.push_back(vulcram_pose);

//     geometry_msgs::Pose offset_poses = vulcram_pose;
//     //lower eff into vulcram
//     poses.push_back(offsetMovement(offset_poses, 0, 0, -0.025, 0, 0, 0, 0));
//     // ros::Duration(1.0).sleep(); // Wait before checking again

//     //lift eff from vulcram.
//     poses.push_back(offsetMovement(offset_poses, 0, 0, 0.025, 0, 0, 0, 0));
//     // ros::Duration(1.0).sleep(); // Wait before checking again

//     //move in y direction
//     poses.push_back(offsetMovement(offset_poses, 0, 0.08, 0, 0, 0, 0, 0));
//     // ros::Duration(1.0).sleep(); // Wait before checking again
    
//     //move in -y direction
//     poses.push_back(offsetMovement(offset_poses, 0, -0.160, 0, 0, 0, 0, 0));
//     // ros::Duration(1.0).sleep(); // Wait before checking again

//     //move back to centre
//     poses.push_back(offsetMovement(offset_poses, 0, 0.08, 0, 0, 0, 0, 0));
//     // ros::Duration(1.0).sleep(); // Wait before checking again

//     //move in x direction
//     poses.push_back(offsetMovement(offset_poses, 0.08, 0, 0, 0, 0, 0, 0));
//     // ros::Duration(1.0).sleep(); // Wait before checking again

//     //move in -x direction
//     poses.push_back(offsetMovement(offset_poses, -0.16, 0, 0, 0, 0, 0, 0));
//     // ros::Duration(1.0).sleep(); // Wait before checking again

//     //move back to centre
//     poses.push_back(offsetMovement(offset_poses, 0.08, 0, 0, 0, 0, 0, 0));

//     return cartesianMovement(move_group, poses, plan);
// }

bool sensorCalibration( ros::NodeHandle nh,
                        moveit::planning_interface::MoveGroupInterface &move_group,
                       moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    std::vector<geometry_msgs::Pose> poses;
    geometry_msgs::Pose vulcram_pose;

    // create a fake vulcram pose and lift above
    vulcram_pose.position.x = 0;
    vulcram_pose.position.y = 0.4;
    vulcram_pose.position.z = 0.04;
    vulcram_pose.orientation.x = 1;
    vulcram_pose.orientation.y = 0;
    vulcram_pose.orientation.z = 0;
    vulcram_pose.orientation.w = 0;
    robotMovement(move_group, vulcram_pose, plan);
    ROS_INFO("First Movement Completed.");
    geometry_msgs::Pose offset_poses = vulcram_pose;
    //lower eff into vulcram
    offsetMovement(offset_poses, 0, 0, -0.025, 0, 0, 0, 0);
    robotMovement(move_group, offset_poses, plan);
    // ros::Duration(1.0).sleep(); // Wait before checking again
    
    // //lift eff from vulcram.
    offsetMovement(offset_poses, 0, 0, 0.025, 0, 0, 0, 0);
    robotMovement(move_group, offset_poses, plan);

    // //move in y direction
    offsetMovement(offset_poses, 0.08, 0, 0, 0, 0, 0, 0);
    robotMovement(move_group, offset_poses, plan);
    ros::Duration(1.0).sleep(); // Wait before checking again

    ROS_INFO("Entered sensor calibration before record");
    offsetMovement(offset_poses, -0.16, 0, 0, 0, 0, 0, 0);
    recordCalibration(nh, move_group, plan, offset_poses);

    // return cartesianMovement(move_group, poses, plan);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_plan");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);

    spinner.start(); 

    // Set up the MoveIt! MoveGroup interface for the UR5e robot
    static const std::string PLANNING_GROUP = "manipulator"; // Replace with your robot's planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPlanningTime(5.0);  // 5 seconds


    move_group.setPoseReferenceFrame("base_link"); // Replace with the correct frame in your URDF    
    // ROS_INFO_N("manipulator", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("main loop before sensorCalibration");
    sensorCalibration(node_handle, move_group, my_plan);
    while (ros::ok())
    {
        ROS_WARN("In loop...");
        ros::Duration(1.0).sleep(); // Wait before checking again
    }
    spinner.stop();
    ros::shutdown();
    return 0;
}

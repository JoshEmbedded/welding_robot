#include "headers/calibrationClass.h"

LaserCalibration::LaserCalibration(const std::string &group_name, ros::NodeHandle node_handle)
    : move_group(group_name), nh(node_handle), bag_open(false), sensor_tilt(-1.8579793), scan_sync_sub(nh, "laser_scan", 1),
    joint_sync_sub(nh, "joint_states", 1, ros::TransportHints().tcpNoDelay())
{
    move_group.setWorkspace(-2.0, -2.0, 0.01, 2.0, 2.0, 2.0);
    // Initialize the synchronizer with ApproximateTime policy
    sync = std::make_unique<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), scan_sync_sub, joint_sync_sub);

    // Register the callback
    sync->registerCallback(boost::bind(&LaserCalibration::callback, this, _1, _2));
    scan_sub = nh.subscribe("laser_scan", 10, &LaserCalibration::laserScanCallback, this, ros::TransportHints().tcpNoDelay());
    joint_sub = nh.subscribe("joint_states", 10, &LaserCalibration::jointStateCallback, this, ros::TransportHints().tcpNoDelay());
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
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    if (handlePlanError(move_group.plan(plan), "planning"))
    {
        
        if (handlePlanError(move_group.execute(plan), "execution"))
        {
            return true;
        }
    }
    return false;
}

bool LaserCalibration::jointMovement(const sensor_msgs::JointState joints)
{
    move_group.setJointValueTarget(joints);
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    if (handlePlanError(move_group.plan(plan), "planning"))
    {
        
        if (handlePlanError(move_group.execute(plan), "execution"))
        {
            return true;
        }
    }
    return false;
}

bool LaserCalibration::cartesianMovement(std::vector<geometry_msgs::Pose> &goal_poses)
{
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(goal_poses, 0.001, 0.0, trajectory);
    ROS_INFO("Cartesian path achieved: %.2f%%", fraction * 100.0);
    plan.trajectory_ = trajectory;
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    bool movement = handlePlanError(move_group.execute(plan), "execution");
    return movement;
}

moveit_msgs::RobotTrajectory LaserCalibration::calculateCartesian(geometry_msgs::Pose start, geometry_msgs::Pose end)
{
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> poses;
    poses.push_back(start);
    poses.push_back(end);
    double fraction = move_group.computeCartesianPath(poses, 0.0001, 0.0, trajectory);
    ROS_INFO("Cartesian path computed with %f%% success rate", fraction * 100.0);
    return trajectory;
}

geometry_msgs::Pose LaserCalibration::offsetMovement(geometry_msgs::Pose &pose, float X, float Y, float Z, float roll_offset, float pitch_offset, float yaw_offset)
{
    pose.position.x += X;
    pose.position.y += Y;
    pose.position.z += Z;
    // Extract current RPY angles from the quaternion
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Apply offsets to RPY angles
    roll += roll_offset;
    pitch += pitch_offset;
    yaw += yaw_offset;

    // Convert updated RPY angles back to quaternion
    tf2::Quaternion updated_q;
    updated_q.setRPY(roll, pitch, yaw);

    // Update the pose's orientation with the new quaternion
    pose.orientation.x = updated_q.x();
    pose.orientation.y = updated_q.y();
    pose.orientation.z = updated_q.z();
    pose.orientation.w = updated_q.w();

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

    //Test rotation calibration also works.
    // offsetMovement(start_pose, 0, 0, 0, 0, 0, 0.3);
    if(robotMovement(start_pose)){
        ROS_INFO("First Movement Completed.");
    }
    else{
        return false;
    }
    
    geometry_msgs::Pose offset_pose = start_pose;
    // lower eff into vulcram
    offsetMovement(offset_pose, 0, 0, -0.025, 0, 0, 0);
    robotMovement(offset_pose);
    ros::Duration(1.0).sleep(); // Wait before checking again

    // //lift eff from vulcram.
    offsetMovement(offset_pose, 0, 0, 0.025, 0, 0, 0);
    robotMovement(offset_pose);
    ros::Duration(1.0).sleep(); // Wait before checking again

    // //move in y direction
    offsetMovement(offset_pose, 0.08, 0, 0, 0, 0, 0);
    robotMovement(offset_pose);
    ros::Duration(1.0).sleep(); // Wait before checking again

    // ROS_INFO("Entered sensor calibration before record");
    offsetMovement(offset_pose, -0.18, 0, 0, 0, 0, 0);
    bool record = recordCalibration(offset_pose);
    ros::Duration(1.0).sleep(); // Wait before checking again

    if (!record)
    {
        ROS_ERROR("Robot Scan Failed.");
        return false;
    }

    if (!processScan())
    {
            ROS_ERROR("Unable to calibrate from first pass.");
            ROS_INFO("Scan pass along y-axis");
            robotMovement(start_pose);
            ros::Duration(1.0).sleep(); // Wait before checking again

            offset_pose = start_pose;
            offsetMovement(offset_pose, 0, 0.08, 0, 0, 0, 0);
            robotMovement(offset_pose);
            ros::Duration(1.0).sleep(); // Wait before checking again

            // ROS_INFO("Entered sensor calibration before record");
            offsetMovement(offset_pose, 0, -0.18, 0, 0, 0, 0);
            record = recordCalibration(offset_pose);
            ros::Duration(1.0).sleep(); // Wait before checking again
    }    
    
    if (!record)
    {
        ROS_ERROR("Robot Scan Failed.");
        return false;
    }
    if (!processScan())
    {
        ROS_ERROR("Analysing Bag Failed.");
        return false;
    }

    ROS_INFO("Moving robot to target Joint States");
    jointMovement(calculated_joints);

    sensor_transform = calculateTransform();

    return true;
}

bool LaserCalibration::recordCalibration(geometry_msgs::Pose pose)
{

    // Resolve the package path
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

    move_group.startStateMonitor(); // initalise for faster state request.
    move_group.setPoseTarget(pose);
    bool plan_success = handlePlanError(move_group.plan(plan));
    bool record;
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
        scan_sub.shutdown();
        joint_sub.shutdown(); 
        bag.close();
        ROS_INFO("Bag Closed");
        record = true;
    } catch (const rosbag::BagException& e) {
        ROS_ERROR("Failed to open bag file: %s", e.what());
        record = false;
    }

  
    ros::Duration(0.05).sleep();

    return record;
}



bool LaserCalibration::processScan()
{
    ros::ServiceClient client = nh.serviceClient<alpaka_demo::ProcessBag>("process_bag");

    // Wait for the service to be available
    ROS_INFO("Waiting for the service to be available...");
    client.waitForExistence();
    ROS_INFO("Service available!");

    alpaka_demo::ProcessBag srv;
        
    srv.request.bag_path = bag_path;

    // Call the service
    if (client.call(srv))
    {
        ROS_INFO("Received joint state:");
        if (srv.response.analysed)
        {
            for (size_t i = 0; i < srv.response.joint_state.name.size(); ++i)
            {
                ROS_INFO("  Joint: %s, Position: %f", srv.response.joint_state.name[i].c_str(),
                srv.response.joint_state.position[i]);
            }
            calculated_joints = srv.response.joint_state;
            service_offset.y = srv.response.y;
            service_offset.z = srv.response.x;
        }
        return srv.response.analysed;
    }
    else
    {
        ROS_ERROR("Failed to call service /get_joint_state");
        return false;
    }
}

geometry_msgs::Pose LaserCalibration::calculateTransform()
{

    /// FIND THE TRANSFORM FROM THE TCP

    tcp_transform = move_group.getCurrentPose().pose;

    // ROS_INFO("TCP Transform:");
    // ROS_INFO("  Position:");
    // ROS_INFO("    x: %.3f", tcp_transform.position.x);
    // ROS_INFO("    y: %.3f", tcp_transform.position.y);
    // ROS_INFO("    z: %.3f", tcp_transform.position.z);

    // ROS_INFO("  Orientation:");
    // ROS_INFO("    x: %.3f", tcp_transform.orientation.x);
    // ROS_INFO("    y: %.3f", tcp_transform.orientation.y);
    // ROS_INFO("    z: %.3f", tcp_transform.orientation.z);
    // ROS_INFO("    w: %.3f", tcp_transform.orientation.w);

    sensor_transform.position.x = -tcp_transform.position.x; //finding the transform between sensor & TCP
    sensor_transform.position.y = service_offset.y;
    sensor_transform.position.z = tcp_transform.position.z - service_offset.z;
    sensor_transform.orientation.x = 0;
    sensor_transform.orientation.y = 0;
    sensor_transform.orientation.z = 0;
    sensor_transform.orientation.w = 1;
    

    // Create a tf2 quaternion from the sensor orientation
    tf2::Quaternion sensor_orientation_quat(
        sensor_transform.orientation.x,
        sensor_transform.orientation.y,
        sensor_transform.orientation.z,
        sensor_transform.orientation.w
    );

    // Create a rotation quaternion for -1.8579793 radians around the y-axis
    tf2::Quaternion rotation_quat;
    rotation_quat.setRPY(0, sensor_tilt, 0); // Roll = 0, Pitch (y-axis) = -1.8579793, Yaw = 0

    // Combine the quaternions
    tf2::Quaternion updated_orientation = sensor_orientation_quat * rotation_quat;

    // Normalize the quaternion to ensure it's valid
    updated_orientation.normalize();

    // Update the orientation in sensor_transform
    sensor_transform.orientation.x = updated_orientation.x();
    sensor_transform.orientation.y = updated_orientation.y();
    sensor_transform.orientation.z = updated_orientation.z();
    sensor_transform.orientation.w = updated_orientation.w();

    ROS_INFO("Laser Scanner Offset Transform:");
    ROS_INFO("  Position:");
    ROS_INFO("    x: %.3f", sensor_transform.position.x);
    ROS_INFO("    y: %.3f", sensor_transform.position.y);
    ROS_INFO("    z: %.3f", sensor_transform.position.z);

    ROS_INFO("  Orientation:");
    ROS_INFO("    x: %.3f", sensor_transform.orientation.x);
    ROS_INFO("    y: %.3f", sensor_transform.orientation.y);
    ROS_INFO("    z: %.3f", sensor_transform.orientation.z);
    ROS_INFO("    w: %.3f", sensor_transform.orientation.w);

    // Return Transform between sensor & TCP
    return sensor_transform;   
}
geometry_msgs::Pose LaserCalibration::getSensorTransform()
{
    return sensor_transform;
}

sensor_msgs::LaserScan LaserCalibration::getLaserScan()
{
    return laser_scan;
}

sensor_msgs::JointState LaserCalibration::getJointScan()
{
    return calculated_joints;
}


void LaserCalibration::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_scan = *msg;
    if (bag.isOpen())
    {
        try
        {
            std::lock_guard<std::mutex> lock(bag_mutex);
            bag.write("laser_scan", msg->header.stamp, *msg);
        }
        catch (const rosbag::BagIOException &e)
        {
            ROS_ERROR("Failed to write laser scan to bag: %s", e.what());
        }
    }
}

void LaserCalibration::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    if (bag.isOpen())
    {
        try
        {
            std::lock_guard<std::mutex> lock(bag_mutex);
            bag.write("joint_states", msg->header.stamp, *msg);
        }
        catch (const rosbag::BagIOException &e)
        {
            ROS_ERROR("Failed to write joint states to bag: %s", e.what());
        }
    }
}

void LaserCalibration::callback(const sensor_msgs::LaserScanConstPtr& laser_scan, const sensor_msgs::JointStateConstPtr& joint_state)
{
    // if (bag.isOpen())
    // {
    //     std::lock_guard<std::mutex> lock(bag_mutex); // Ensure thread-safe access to the bag
    //     bag.write("laser_scan", laser_scan->header.stamp, *laser_scan);
    //     bag.write("joint_states", joint_state->header.stamp, *joint_state);
    // }
}

void LaserCalibration::readCalibrationData()
{
    std::lock_guard<std::mutex> lock(bag_mutex); // Ensure thread-safe access to the bag
    bag.open(bag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("laser_scan"));
    topics.push_back(std::string("joint_states"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        // LaserScan message handling
        sensor_msgs::LaserScan::ConstPtr s = m.instantiate<sensor_msgs::LaserScan>();
        if (s != NULL) {
            std::cout << "LaserScan data (ranges):" << std::endl;
            for (size_t j = 0; j < s->ranges.size(); ++j) {
                std::cout << s->ranges[j] << " ";  // Print each range value
            }
            std::cout << std::endl;
        }

        // JointState message handling
        sensor_msgs::JointState::ConstPtr i = m.instantiate<sensor_msgs::JointState>();
        if (i != NULL) {
            std::cout << "JointState data:" << std::endl;
            for (size_t j = 0; j < i->position.size(); ++j) {
                std::cout << i->position[j] << " ";  // Print joint positions
            }
            std::cout << std::endl;
        }
    }
    ros::Duration(1.0).sleep(); // Wait before checking again

    bag.close();
}




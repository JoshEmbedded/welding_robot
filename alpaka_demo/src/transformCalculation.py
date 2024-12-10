import rospy
import tf2_ros
import geometry_msgs.msg


def get_relative_transform():
    # Initialize ROS node
    rospy.init_node('relative_transform_node')

    # Create a Buffer and Listener for tf2
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Wait to allow buffering
    rate = rospy.Rate(1)  # 1 Hz rate loop
    while not rospy.is_shutdown():
        try:
            # Look up the transform
            transform = tf_buffer.lookup_transform(
                target_frame='tcp_eff',
                source_frame='laser_scanner_link',
                time=rospy.Time(0),
                timeout=rospy.Duration(1.0)
            )
            rospy.loginfo("Transform from tcp_eff to laser_link found.")
            rospy.loginfo(transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Could not find transform: %s", e)
        break
        # Sleep until next iteration
        rate.sleep()


if __name__ == "__main__":
    try:
        get_relative_transform()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Node interrupted by user shutdown.")

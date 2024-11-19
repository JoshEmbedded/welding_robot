#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class WeldSeamPoseSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('weld_seam_pose_subscriber', anonymous=True)

        # Initialize the PoseStamped storage
        self.current_pose = PoseStamped()

        # Create a subscriber for the /weld_seam_pose topic
        self.subscriber = rospy.Subscriber(
            '/weld_seam_pose',
            PoseStamped,
            self.pose_callback
        )
        
        self.weld_marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        rospy.loginfo("Weld seam pose subscriber initialized.")

    def pose_callback(self, msg):
        """
        Callback function for the /weld_seam_pose topic.
        """
        # Store the received PoseStamped message
        self.current_pose = msg
        rospy.loginfo(f"Received pose: {self.current_pose}")
        self.create_arrow_marker(self.current_pose)
        
    def create_arrow_marker(self, pose_stamped, marker_id=0, scale=0.01, color=(1.0, 0.0, 0.0, 1.0)):
        """
        Creates an arrow marker for a PoseStamped message.

        :param pose_stamped: PoseStamped message for which the arrow marker is created.
        :type pose_stamped: geometry_msgs.msg.PoseStamped
        :param marker_id: Unique ID for the marker (default: 0).
        :type marker_id: int
        :param scale: Scale factor for the arrow marker (default: 0.1).
                    Represents the diameter of the arrow shaft.
        :type scale: float
        :param color: RGBA tuple for the marker color (default: (1.0, 0.0, 0.0, 1.0) - red).
        :type color: tuple(float, float, float, float)
        :return: A Marker message to visualize the PoseStamped.
        :rtype: visualization_msgs.msg.Marker
        """
        rospy.loginfo("Marker Function Entered")
        marker = Marker()
        
        # Set the marker type to an arrow
        marker.type = Marker.ARROW

        # Set the marker ID and namespace
        marker.id = marker_id
        marker.ns = "arrow_marker"

        # Set the frame ID and timestamp
        marker.header.frame_id = pose_stamped.header.frame_id
        marker.header.stamp = pose_stamped.header.stamp

        # Set the pose of the marker
        marker.pose = pose_stamped.pose

        # Set the scale of the marker
        marker.scale.x = scale * 2.0  # Length of the arrow
        marker.scale.y = scale       # Diameter of the arrow shaft
        marker.scale.z = scale       # Diameter of the arrow head

        # Set the color of the marker
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        # Set lifetime (0 means forever)
        marker.lifetime = rospy.Duration(0)
        
        self.weld_marker_publisher.publish(marker)

        return marker
    
    def run(self):
        """
        Keeps the node running to listen for messages.
        """
        rospy.spin()
 

if __name__ == '__main__':
    try:
        # Create and run the subscriber node
        node = WeldSeamPoseSubscriber()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Weld seam pose subscriber node terminated.")

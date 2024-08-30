#!/usr/bin/env python3

"""
Implementation of the OdomTFBroadcaster class.

Adapted from ROS 2 tutorial https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html

Primary author:
Space-ROS

Docstrings and comments added by:
Azmyin Md. Kamal,
Ph.D. student in MIE, iCORE Lab,
Louisiana State University, Louisiana, USA

Date: August 29th, 2024
Version: 1.0
AI: ChatGPT 4.o

"""

#imports
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

class OdomTFBroadcaster(Node):
    """Publish the coordinate transforms between different frames of the rover."""

    def __init__(self):
        super().__init__('odom_tf_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Setup subscribers
        # Gazebo Ignition publishes [odometry data](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html#details)
        # ! Name changed from /model/curiosity_mars_rover/odometry to ???
        # Subscribe to Odometry topic
        sim_odom_topic_name = "/model/curiosity_mars_rover/odometry" # TODO: make the topic name a parameter instead of hardcoded
        self.subscription = self.create_subscription(Odometry,sim_odom_topic_name,
                                                     self.handle_odometry,1)
        self.subscription  # prevent unused variable warning

    def handle_odometry(self, msg):
        """Process odometry message from simulated world."""
        tf = TransformStamped()
        # Read message content and assign it to
        # Corresponding tf variables
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = msg.header.frame_id
        tf.child_frame_id = msg.child_frame_id
        # Get translation coordinates from the message pose
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        # Get rotation from the message pose 
        tf.transform.rotation.x = msg.pose.pose.orientation.x
        tf.transform.rotation.y = msg.pose.pose.orientation.y
        tf.transform.rotation.z = msg.pose.pose.orientation.z
        tf.transform.rotation.w = msg.pose.pose.orientation.w
        # Send the transformation
        self.tf_broadcaster.sendTransform(tf)

def main():
    """Run main."""
    rclpy.init()
    node = OdomTFBroadcaster()
    node.get_logger().info('Starting odometry_tf_publisher node')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
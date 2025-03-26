#!/usr/bin/env python3

# adapted from ROS 2 tutorial
# https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html

import rclpy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

class OdomTFBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to Odometry topic
        # TODO: make the topic name a parameter instead of hard-coded
        self.subscription = self.create_subscription(
            Odometry,
            f'/model/curiosity_mars_rover/odometry',
            self.handle_odometry,
            1)
        self.subscription  # prevent unused variable warning

    def handle_odometry(self, msg):
        tf = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = msg.header.frame_id
        tf.child_frame_id = msg.child_frame_id

        # get translation coordinates from the message pose
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z

        # get rotation from the message pose 
        tf.transform.rotation.x = msg.pose.pose.orientation.x
        tf.transform.rotation.y = msg.pose.pose.orientation.y
        tf.transform.rotation.z = msg.pose.pose.orientation.z
        tf.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Send the transformation
        self.tf_broadcaster.sendTransform(tf)


def main():
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
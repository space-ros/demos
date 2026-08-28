#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
import time

import sys


class MastArmController(Node):

    def __init__(self, joints):
        super().__init__('mast_joint_trajectory_controller')
        self.get_logger().info("Initializing mast_joint_trajectory_controller node...")

        self.joints_ = joints.split(",")

        # Publisher for JointState
        self.mast_publisher_ = self.create_publisher(JointState, '/mast_joint_state_controller/commands', 10)

        # Services for controlling the mast
        self.mast_open_srv = self.create_service(Empty, 'mast_open', self.mast_open_callback)
        self.mast_close_srv = self.create_service(Empty, 'mast_close', self.mast_close_callback)
        self.mast_rotate_srv = self.create_service(Empty, 'mast_rotate', self.mast_rotate_callback)

        # Retain the current joint positions
        self.current_positions = [0.0, 0.0, 0.0]  # Initialize with default positions
        self.get_logger().info("mast_joint_trajectory_controller node initialized and ready.")

    def mast_open_callback(self, request, response):
        desired_positions = [0.0, -0.5, 0.0]
        self.move_mast(desired_positions, duration=1.0)
        return response

    def mast_close_callback(self, request, response):
        desired_positions = [1.57, -1.57, 0.0]
        self.move_mast(desired_positions, duration=1.0)
        return response

    def mast_rotate_callback(self, request, response):
        # Sequence of positions to rotate the mast
        sequence = [
            ([0.0, -1.57, 0.0], 2.0),
            ([0.0, -3.14, 0.0], 2.0),
            ([0.0, -6.28, 0.0], 2.0)
        ]

        for desired_positions, duration in sequence:
            self.move_mast(desired_positions, duration)

        return response

    def move_mast(self, desired_positions, duration):
        steps = int(duration / 0.01)  # Number of steps based on timer period (0.1s)

        # Calculate the incremental step size for each position
        increments = [
            (desired_positions[i] - self.current_positions[i]) / steps
            for i in range(len(self.current_positions))
        ]

        # Gradually update positions over the specified duration
        for step in range(steps):
            self.current_positions = [
                self.current_positions[i] + increments[i]
                for i in range(len(self.current_positions))
            ]

            # Create JointState message
            joint_state_msg = JointState()
            # joint_state_msg.name = ["mast_p_joint", "mast_02_joint", "mast_cameras_joint"]
            joint_state_msg.name = self.joints_
            joint_state_msg.position = self.current_positions

            # Publish the JointState message
            self.get_logger().info(f"Publishing mast positions: {joint_state_msg}")
            self.mast_publisher_.publish(joint_state_msg)

            # Sleep to simulate time progression
            time.sleep(0.1)  # Sleep to match the timer period (0.1 seconds)


def main(args=None):
    rclpy.init(args=args)

    if len(args) > 1:
        joints =args[1]
        wheel_node = MastArmController(joints)
        wheel_node.get_logger().info("ARGUMENTS:: {}".format(joints))

        rclpy.spin(wheel_node)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        wheel_node.destroy_node()
        
    # else:
    #     wheel_node.get_logger().warn("JOINTS not provided, exiting..")

    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

# from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
import time

import sys

class ArmController(Node):

    def __init__(self, joints):
        super().__init__('arm_joint_trajectory_controller')
        self.get_logger().info("Initializing arm_joint_trajectory_controller node...")

        self.joints_ = joints.split(",")

        # Publisher for JointState
        self.arm_publisher_ = self.create_publisher(JointState, '/arm_joint_position', 10)

        # Services for controlling the arm
        self.open_srv = self.create_service(Empty, 'open_arm', self.open_arm_callback)
        self.close_srv = self.create_service(Empty, 'close_arm', self.close_arm_callback)

        # Initialize joint states
        self.current_positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # Default joint positions
        self.get_logger().info("arm_joint_trajectory_controller node initialized and ready.")

    def open_arm_callback(self, request, response):
        # Define desired positions for opening the arm
        desired_positions = [0.0, -0.5, 3.0, 1.0, 0.0]
        self.move_arm(desired_positions, duration=4.0)
        return response

    def close_arm_callback(self, request, response):
        # Define desired positions for closing the arm
        desired_positions = [-1.57, -0.4, -1.1, -1.57, -1.57]
        self.move_arm(desired_positions, duration=4.0)
        return response

    def move_arm(self, desired_positions, duration):
        # Calculate the number of steps based on the timer period (0.1s)
        steps = int(duration / 0.02)

        # Calculate the incremental step size for each position
        increments = [
            (desired_positions[i] - self.current_positions[i]) / steps
            for i in range(len(self.current_positions))
        ]

        # Gradually update positions over the specified duration
        for step in range(steps):
            # Update the current positions by adding the increments
            self.current_positions = [
                self.current_positions[i] + increments[i]
                for i in range(len(self.current_positions))
            ]

            # Create JointState message
            joint_state_msg = JointState()
            # joint_state_msg.name = ["arm_01_joint", "arm_02_joint", "arm_03_joint", "arm_04_joint", "arm_tools_joint"]
            joint_state_msg.name = self.joints_
            joint_state_msg.position = self.current_positions

            # Publish the JointState message
            self.get_logger().info(f"Publishing arm positions: {joint_state_msg}")
            self.arm_publisher_.publish(joint_state_msg)

            # Sleep to simulate time progression
            time.sleep(0.1)  # Sleep to match the timer period (0.1 seconds)

        # Update the current positions to the desired positions after the movement is complete
        self.current_positions = desired_positions

def main(args=None):
    rclpy.init(args=args)

    if len(args) > 1:
        joints =args[1]
        wheel_node = ArmController(joints)
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
#!/usr/bin/env python3

from pickle import FALSE
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

import rclpy.time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math

import sys, time


class SteerController(Node):

    def __init__(self, joints):
        super().__init__('steer_position_controller')

        self.joints_ = joints.split(",")

        # self.get_logger().info("TESTING STEER...")
        self.steer_publisher_ = self.create_publisher(JointState, '/steer_position', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.curr_vel = Twist()
        self.last_vel = Twist()
        self.should_steer = False

    def vel_callback(self, msg):
        # self.get_logger().info("CMD_VEL callback")
        if abs(self.last_vel.angular.z - self.curr_vel.angular.z) > 0.01 and self.should_steer is False:
            self.last_vel = Twist()
            self.last_vel.linear.x = self.curr_vel.linear.x
            self.last_vel.angular.z = self.curr_vel.angular.z
            self.should_steer = True

        self.curr_vel = msg


    def map_angular_to_steering(self, angular_speed) -> float:
        if abs(angular_speed) < 1e-3:
            return 0.0

        #max 0.6 min -0.6
        angular_speed = min(0.6, max(angular_speed, -0.6))
        return (angular_speed/abs(angular_speed))*(-25 * abs(angular_speed) + 17.5)


    def set_steering(self, turn_rad):
        # Desired steering position using JointState message
        target_steer = JointState()
        
        # target_steer.header.stamp = self.get_clock().now().to_msg()
        target_steer.name = self.joints_

        # Retain the last known positions
        if not hasattr(self, 'current_positions'):
            # Initialize starting positions if not set
            self.current_positions = [0.0, 0.0, 0.0, 0.0]

        if abs(turn_rad) < 1e-3:
            desired_positions = [0.0, 0.0, 0.0, 0.0]
        else:
            R = abs(turn_rad)         #Turning radius

            L = 2.08157     #Chassis Length
            T = 1.53774     #Chassis Width

            alpha_i = math.atan(L/(R - (T/2)))
            alpha_o = math.atan(L/(R + (T/2)))

            if alpha_i > 0.6:
                alpha_i = 0.6

            if alpha_o > 0.6:
                alpha_o = 0.6

            alpha_i = round(alpha_i, 2)
            alpha_o = round(alpha_o, 2)

            if turn_rad > 0.0:
                desired_positions = [alpha_i, -alpha_i, alpha_o, -alpha_o]
            else:
                desired_positions = [-alpha_o, alpha_o, -alpha_i, alpha_i]

        
       # Time duration to reach desired positions
        duration = 1.0  # 1 second
        steps = int(duration / 0.02)  # Number of steps based on timer period

        # Calculate the incremental step size for each position
        increments = [
            (desired_positions[i] - self.current_positions[i]) / steps
            for i in range(len(self.current_positions))
        ]

        for step in range(steps):
            # Update current positions by adding the increments
            self.current_positions = [
                self.current_positions[i] + increments[i]
                for i in range(len(self.current_positions))
            ]
            target_steer.position = self.current_positions
            self.get_logger().info(f"Publishing steering positions: {target_steer}")
            self.steer_publisher_.publish(target_steer)

            # Sleep to simulate time progression
            time.sleep(0.1)  # Adjust the sleep time to match the timer period


    def timer_callback(self):
        
        if self.should_steer:
            steering_val = self.map_angular_to_steering(self.curr_vel.angular.z)
            self.set_steering(steering_val)
            self.should_steer = False


def main(args=None):
    rclpy.init(args=args)

    if len(args) > 1:
        joints =args[1]
        steer_node = SteerController(joints)
        steer_node.get_logger().info("ARGUMENTS:: {}".format(joints))

        rclpy.spin(steer_node)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        steer_node.destroy_node()
        
    # else:
    #     wheel_node.get_logger().warn("JOINTS not provided, exiting..")

    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
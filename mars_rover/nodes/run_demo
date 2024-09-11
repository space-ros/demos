#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
import math
from random import randint
from std_srvs.srv import Empty


# A node for performing combination of motions from different moving parts of the Curiosity rover.
class RunDemo(Node):
    def __init__(self) -> None:
        super().__init__('run_node')
        self.motion_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.forward_srv = self.create_service(Empty, 'move_forward', self.move_forward_callback)
        self.stop_srv = self.create_service(Empty, 'move_stop', self.move_stop_callback)
        self.left_srv = self.create_service(Empty, 'turn_left', self.turn_left_callback)
        self.right_srv = self.create_service(Empty, 'turn_right', self.turn_right_callback)
        self.stopped = True

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.curr_action = Twist()

    def timer_callback(self):
        if (not self.stopped):
            self.motion_publisher_.publish(self.curr_action)

    def move_forward_callback(self, request, response):
        self.get_logger().info("Moving forward")
        action = Twist()
        action.linear.x = 2.0
        self.curr_action = action
        self.stopped = False
        return response

    def move_stop_callback(self, request, response):
        # stop timer from publishing
        self.stopped = True
        self.get_logger().info("Stopping")
        self.curr_action = Twist()
        # publish once to ensure we stop
        self.motion_publisher_.publish(self.curr_action)
        return response

    def turn_left_callback(self, request, response):
        self.get_logger().info("Turning left")
        action = Twist()
        action.linear.x = 1.0
        action.angular.z = 0.4
        self.curr_action = action
        self.stopped = False
        return response

    def turn_right_callback(self, request, response):
        self.get_logger().info("Turning right")
        self.stopped = False
        action = Twist()
        action.linear.x = 1.0
        action.angular.z = -0.4
        self.curr_action = action
        self.stopped = False
        return response


def main(args=None):
    rclpy.init(args=args)

    run_demo = RunDemo()

    rclpy.spin(run_demo)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    run_demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
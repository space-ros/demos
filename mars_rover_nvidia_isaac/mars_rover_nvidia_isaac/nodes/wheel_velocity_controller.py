#!/usr/bin/env python3

from pickle import FALSE
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

# from std_msgs.msg import String, Float64MultiArray
import rclpy.time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math

import sys

class WheelController(Node):
# class WheelController():

    # def __init__(self, joints, node_handle):
    def __init__(self, joints):
        super().__init__('wheel_velocity_controller')

        self.joints_ = joints.split(",")
        # self.nh_ = node_handle


        # self.nh_.get_logger().info("TESTING 123...")
        self.wheel_publisher_ = self.create_publisher(JointState, '/wheel_velocity', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.curr_vel = Twist()
        # self.last_vel = Twist()

    def vel_callback(self, msg):
        # if abs(self.last_vel.angular.z - self.curr_vel.angular.z) > 0.01 and self.should_steer is False:
        #     self.last_vel = Twist()
        #     self.last_vel.linear.x = self.curr_vel.linear.x
        #     self.last_vel.angular.z = self.curr_vel.angular.z
        #     self.should_steer = True

        self.curr_vel = msg

    def set_wheel_common_speed(self, vel):

        target_state = JointState()

        # target_state.header.stamp = self.get_clock().now()
        target_state.name = self.joints_
        target_state.velocity = [vel, vel*1.5, vel, -vel, -vel*1.5, -vel]
        
        
        # self.nh_.get_logger().info(f'Publishing: "{target_state}"')
        self.wheel_publisher_.publish(target_state)

    def timer_callback(self):
        self.set_wheel_common_speed(self.curr_vel.linear.x)


def main(args=None):
    rclpy.init(args=args)

    if len(args) > 1:
        joints =args[1]
        wheel_node = WheelController(joints)
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
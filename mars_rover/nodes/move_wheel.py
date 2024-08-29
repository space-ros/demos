#!/usr/bin/env python3

"""
Implementation of the MoveWheel class.

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

# Imports
# from pickle import FALSE 
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
import math

class MoveWheel(Node):
    """Node to control the wheel velocity, steering, and suspension."""

    def __init__(self):
        super().__init__('wheel_node')
        # Setup publishers
        self.wheel_publisher_ = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.steer_publisher_ = self.create_publisher(JointTrajectory, '/steer_position_controller/joint_trajectory', 10)
        self.suspension_publisher_ = self.create_publisher(Float64MultiArray, '/wheel_tree_position_controller/commands',10)
        timer_period = 0.1  # seconds, 100ms to update once?
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Setup subscribers
        # Not used anywhere in this file,
        # TODO depricate and delete this subscriberm it is not used anywhere
        # self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        # Important instance variables
        self.curr_vel = Twist()
        self.last_vel = Twist()
        self.should_steer = False

    def vel_callback(self, msg):
        """Handle incoming velocity messages and updates steering state."""
        if abs(self.last_vel.angular.z - self.curr_vel.angular.z) > 0.01 and self.should_steer is False:
            self.last_vel = Twist()
            self.last_vel.linear.x = self.curr_vel.linear.x
            self.last_vel.angular.z = self.curr_vel.angular.z
            self.should_steer = True

        self.curr_vel = msg

    def set_wheel_common_speed(self, vel):
        """Set the common speed for all wheels and publishes the command."""
        target_vel = Float64MultiArray()
        target_vel.data = [vel, vel*1.5, vel, -vel, -vel*1.5, -vel]
        # self.get_logger().info(f'Publishing: "{target_vel.data}"')
        self.wheel_publisher_.publish(target_vel)

    def map_angular_to_steering(self, angular_speed) -> float:
        """Map angular speed to a corresponding steering angle."""
        if abs(angular_speed) < 1e-3:
            return 0.0
        angular_speed = min(0.6, max(angular_speed, -0.6)) #max 0.6 min -0.6
        return (angular_speed/abs(angular_speed))*(-25 * abs(angular_speed) + 17.5)

    def set_steering(self, turn_rad):
        """Set the steering angles for the wheels using Ideal Ackerman Steering model."""
        target_steer = JointTrajectory()
        # target_steer.header.stamp = self.get_clock().now().to_msg()
        target_steer.joint_names = ["suspension_steer_F_L_joint", "suspension_steer_B_L_joint", 
                                    "suspension_steer_F_R_joint", "suspension_steer_B_R_joint"]
        steer_point = JointTrajectoryPoint()
        if abs(turn_rad) < 1e-3:
            steer_point.positions = [0.0, 0.0, 0.0, 0.0]
        else:
            R = abs(turn_rad) #Turning radius
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
                steer_point.positions = [alpha_i, -alpha_i, alpha_o, -alpha_o]
            else:
                steer_point.positions = [-alpha_o, alpha_o, -alpha_i, alpha_i]
        steer_point.time_from_start = Duration(sec=1)
        target_steer.points.append(steer_point)
        self.steer_publisher_.publish(target_steer)

    def set_suspension(self, sus_val=None):
        """Set and publish the suspension values for the rover."""
        target_val = Float64MultiArray()
        if sus_val is None:
            target_val.data = [0.3,-0.1,0.3,-0.1] # TODO where are these values coming from?
        self.suspension_publisher_.publish(target_val)

    def timer_callback(self):
        """Periodic callback to update wheel speed, suspension, and steering."""
        self.set_wheel_common_speed(self.curr_vel.linear.x)
        self.set_suspension()
        if self.should_steer:
            steering_val = self.map_angular_to_steering(self.curr_vel.angular.z)
            self.set_steering(steering_val)
            self.should_steer = False

def main(args=None):
    """Run main."""
    rclpy.init(args=args)
    wheel_node = MoveWheel()
    rclpy.spin(wheel_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wheel_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
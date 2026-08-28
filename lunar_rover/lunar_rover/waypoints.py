#!/usr/bin/env python3
# Python demo of Lunar Roving Vehicle Waypoint Following
# Jasper Grant
# August 26th, 2024

from math import sqrt, atan2, pi
# ROS2 Python API import
import rclpy
from rclpy.node import Node
# Messages used for diff drive control import
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Constants for proportional control
KP = [0.2, 0.5]
LINEAR_MINIMUM_SPEED = 0.3
WAYPOINT_DISTANCE_FOR_EQUALITY = 0.1

# Waypoints to follow
WAYPOINTS = [(10, 0), (10, 10), (0, 10), (0, 0)]
# In a practical system, waypoints would be read from a file or other source

class WaypointFollower(Node):

    # init class
    def __init__(self):
        # Super init Node class
        super().__init__('waypoint_follower')
        # Create publisher to control robot from cmd_vel
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel',10)
        # Create subscriber to observe robot odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/lunar_roving_vehicle/odometry',
            self.odom_callback,
            10
        )
        # Start odom as empty odom object
        self.current_odom = Odometry()
        self.waypoints = WAYPOINTS
        self.current_waypoint_index = 0

    # Update model's odom belief from recieved message
    def odom_callback(self, msg):
        self.current_odom = msg
        #self.get_logger().info(f'Currrent Position: {self.current_odom.pose.pose.position.x}, {self.current_odom.pose.pose.position.y}')
        self.navigate_to_waypoint()

    # Calculate distance and angle to the next waypoint
    def calculate_distance_and_angle(self, waypoint):
        self.get_logger().info(f'Waypoint: {waypoint}')
        x = self.current_odom.pose.pose.position.x
        y = self.current_odom.pose.pose.position.y
        self.get_logger().info(f'Current Position: {x}, {y}')
        dx = waypoint[0] - x
        dy = waypoint[1] - y
        #self.get_logger().info(f'dx: {dx}, dy: {dy}')
        distance = sqrt(dx**2 + dy**2)
        angle = atan2(dy, dx)
        # Convert angle to be relative to the robot
        # Get the robot's current orientation from quaternion
        angle = angle - 2 * atan2(self.current_odom.pose.pose.orientation.z, self.current_odom.pose.pose.orientation.w)
        # Normalize angle to be between -pi and pi
        angle = (angle + pi) % (2 * pi) - pi
        self.get_logger().info(f'distance: {distance}, angle: {angle}')
        return distance, angle

    def navigate_to_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached')
            return
        

        waypoint = self.waypoints[self.current_waypoint_index]
        distance, angle = self.calculate_distance_and_angle(waypoint)

        if distance < WAYPOINT_DISTANCE_FOR_EQUALITY:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached.')
            self.current_waypoint_index += 1
            return
        
        msg = Twist()
        # Keep the robot moving forward at a minimum speed
        msg.linear.x = max([KP[0] * distance, LINEAR_MINIMUM_SPEED])

        msg.angular.z = KP[1] * angle
        self.vel_pub.publish(msg)
        


    
def main(args=None):
    # Initialize ROS2 node
    rclpy.init(args=args)
    # Create WaypointFollower object
    waypoint_follower = WaypointFollower()
    # Spin ROS2 node
    rclpy.spin(waypoint_follower)
    # Destroy node on shutdown
    waypoint_follower.destroy_node()
    # Shutdown ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    main()


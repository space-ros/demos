#!/usr/bin/env python3

from pickle import FALSE
import rclpy
from rclpy.node import Node
# from builtin_interfaces.msg import Duration

from sensor_msgs.msg import JointState

import sys

class WheelTreeController(Node):

    def __init__(self, joints):
        super().__init__('wheel_tree_position_controller')

        self.joints_ = joints.split(",")

        self.suspension_publisher_ = self.create_publisher(JointState, '/wheel_tree_position',10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def set_suspension(self, sus_val=None):
        target_val = JointState()
        if sus_val is None:
            target_val.effort = [0.3,-0.1,0.3,-0.1]
            target_val.name = self.joints_
        
        self.suspension_publisher_.publish(target_val)


    def timer_callback(self):
        self.set_suspension()

def main(args=None):
    rclpy.init(args=args)

    if len(args) > 1:
        joints =args[1]
        wheel_node = WheelTreeController(joints)
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
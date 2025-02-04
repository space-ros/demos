#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from std_msgs.msg import String, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty

class MoveArm(Node):

    def __init__(self):
        super().__init__('arm_node')
        self.arm_publisher_ = self.create_publisher(JointTrajectory, '/arm_joint_trajectory_controller/joint_trajectory', 10)
        self.open_srv = self.create_service(Empty, 'open_arm', self.open_arm_callback)
        self.close_srv = self.create_service(Empty, 'close_arm', self.close_arm_callback)


        self.open = True


    def open_arm_callback(self, request, response):
        self.open = True
        traj = JointTrajectory()
        traj.joint_names = ["arm_01_joint", "arm_02_joint", "arm_03_joint", "arm_04_joint", "arm_tools_joint"]
        
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0,-0.5,3.0,1.0,0.0]
        point1.time_from_start = Duration(sec=4)

        traj.points.append(point1)
        self.arm_publisher_.publish(traj)


        return response

    def close_arm_callback(self, request, response):
        self.open = False
        traj = JointTrajectory()
        traj.joint_names = ["arm_01_joint", "arm_02_joint", "arm_03_joint", "arm_04_joint", "arm_tools_joint"]
        
        point1 = JointTrajectoryPoint()
        point1.positions = [-1.57,-0.4,-1.1,-1.57,-1.57]
        point1.time_from_start = Duration(sec=4)

        traj.points.append(point1)
        self.arm_publisher_.publish(traj)

        return response



def main(args=None):
    rclpy.init(args=args)

    arm_node = MoveArm()

    rclpy.spin(arm_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
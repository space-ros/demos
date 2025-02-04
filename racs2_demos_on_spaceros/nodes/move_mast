#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from std_msgs.msg import String, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty

class MastArm(Node):

    def __init__(self):
        super().__init__('mast_node')
        self.mast_publisher_ = self.create_publisher(JointTrajectory, '/mast_joint_trajectory_controller/joint_trajectory', 10)
        self.mast_open_srv = self.create_service(Empty, 'mast_open', self.mast_open_callback)
        self.mast_close_srv = self.create_service(Empty, 'mast_close', self.mast_close_callback)
        self.mast_rotate_srv = self.create_service(Empty, 'mast_rotate', self.mast_rotate_callback)


    def mast_open_callback(self, request, response):
        traj = JointTrajectory()
        traj.joint_names = ["mast_p_joint", "mast_02_joint", "mast_cameras_joint"]
        
        point = JointTrajectoryPoint()
        point.positions = [0.0,-0.5,0.0]
        point.time_from_start = Duration(sec=1)

        traj.points.append(point)
        self.mast_publisher_.publish(traj)
        return response

    def mast_close_callback(self, request, response):
        traj = JointTrajectory()
        traj.joint_names = ["mast_p_joint", "mast_02_joint", "mast_cameras_joint"]
        
        point = JointTrajectoryPoint()
        point.positions = [1.57,-1.57,0.0]
        point.time_from_start = Duration(sec=1)

        traj.points.append(point)
        self.mast_publisher_.publish(traj)
        return response

    def mast_rotate_callback(self, request, response):
        traj = JointTrajectory()
        traj.joint_names = ["mast_p_joint", "mast_02_joint", "mast_cameras_joint"]
        
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0,-1.57,0.0]
        point1.time_from_start = Duration(sec=2)
        traj.points.append(point1)

        point2 = JointTrajectoryPoint()
        point2.positions = [0.0,-3.14,0.0]
        point2.time_from_start = Duration(sec=4)
        traj.points.append(point2)

        point3 = JointTrajectoryPoint()
        point3.positions = [0.0,-6.28,0.0]
        point3.time_from_start = Duration(sec=6)
        traj.points.append(point3)

        self.mast_publisher_.publish(traj)
        return response



def main(args=None):
    rclpy.init(args=args)

    mast_node = MastArm()

    rclpy.spin(mast_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mast_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
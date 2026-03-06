#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
import numpy as np
from scipy.spatial.transform import Rotation as R

class PointCloudTransformer(Node):

    def __init__(self):
        super().__init__('point_cloud_transformer')

        self.declare_parameter('input_topic', '/robot_1/robot_1_camera/points')
        self.declare_parameter('output_topic', '/robot_1/robot_1_camera/points_transformed')
        self.declare_parameter('roll_degrees', 90.0) 
        self.declare_parameter('pitch_degrees', 180.0)  
        self.declare_parameter('yaw_degrees', 90.0)  
        
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        roll_degrees = self.get_parameter('roll_degrees').get_parameter_value().double_value
        pitch_degrees = self.get_parameter('pitch_degrees').get_parameter_value().double_value
        yaw_degrees = self.get_parameter('yaw_degrees').get_parameter_value().double_value

        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pc_callback,
            1
        )
        self.publisher = self.create_publisher(PointCloud2, output_topic, 1)
        
        self.rotation = R.from_euler('xyz', [roll_degrees, pitch_degrees, yaw_degrees], degrees=True)

    def pc_callback(self, msg):
        
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, int(msg.point_step / 4))
        xyz_points = points[:, :3] 
        distances = np.linalg.norm(xyz_points, axis=1)
        valid_points = xyz_points[distances <= 3.0]

        if len(valid_points) == 0:
            return  
        valid_points_rotated = self.rotation.apply(valid_points)

        header = msg.header
        transformed_pc = create_cloud_xyz32(header, valid_points_rotated)
        self.publisher.publish(transformed_pc)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = PointCloudTransformer()
    executor.add_node(node)
    try:
        executor.spin() 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

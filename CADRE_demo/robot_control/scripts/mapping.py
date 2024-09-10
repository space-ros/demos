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
        
        # Parameters
        self.declare_parameter('input_topic', '/robot_1/robot_1_camera/points')
        self.declare_parameter('output_topic', '/robot_1/robot_1_camera/points_transformed')
        self.declare_parameter('roll_degrees', 90.0)  # Roll rotation in degrees
        self.declare_parameter('pitch_degrees', 180.0)  # Pitch rotation in degrees
        self.declare_parameter('yaw_degrees', 90.0)  # Yaw rotation in degrees
        
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
        
        # Rotation setup: roll, pitch, and yaw
        self.rotation = R.from_euler('xyz', [roll_degrees, pitch_degrees, yaw_degrees], degrees=True)

    def pc_callback(self, msg):
        # Extracting point cloud data in a NumPy-compatible way
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, int(msg.point_step / 4))
        
        # Extract the XYZ coordinates and calculate distance
        xyz_points = points[:, :3]  # XYZ are the first three fields
        distances = np.linalg.norm(xyz_points, axis=1)
        
        # Filter points based on distance <= 4.0
        valid_points = xyz_points[distances <= 3.0]

        if len(valid_points) == 0:
            return  # No valid points to process
        
        # Apply rotation to valid points
        valid_points_rotated = self.rotation.apply(valid_points)
        
        # Create and publish the transformed point cloud
        header = msg.header
        transformed_pc = create_cloud_xyz32(header, valid_points_rotated)
        self.publisher.publish(transformed_pc)

def main(args=None):
    rclpy.init(args=args)

    # Use a multithreaded executor for handling multiple callbacks
    executor = MultiThreadedExecutor()

    # Create the node and add it to the executor
    node = PointCloudTransformer()
    executor.add_node(node)

    try:
        executor.spin()  # Spin using the multi-threaded executor
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

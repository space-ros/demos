# Copyright 2024 Blazej Fiderek (xfiderek)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ros_trick_bridge.trick_variable_server_client import TrickVariableServerClient
from std_msgs.msg import Header


class WrenchPublisher(Node):
    def __init__(self):
        super().__init__("wrench_publisher")
        self._trick_host = "localhost"
        self._trick_port = 49765
        self._ndof = 7
        self._trick_variable_names = [
            item
            for i in range(self._ndof)
            for item in [
                f"CanadarmManip.manip.applied_torque[{i}]",
                f"CanadarmManip.manip.friction_torque[{i}]",
            ]
        ]

        # let everything start
        time.sleep(5.0)

        self.get_logger().info("starting trick variable server")
        self._trick_variable_server_client = TrickVariableServerClient(
            host=self._trick_host,
            port=self._trick_port,
            client_tag="wrench_pub",
            trick_variable_names=self._trick_variable_names,
            trick_var_cycle_time=0.1,
            on_receive_callback=self.trick_data_callback,
        )
        self.link_frames = ["B1", "B2", "B3", "B4", "B5", "B6", "EE_SSRMS"]
        self.rot_axes = [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        self.get_logger().info("spawning wrench publishers")
        self.wrench_publishers = [
            publisher
            for link_name in self.link_frames
            for publisher in [
                self.create_publisher(
                    WrenchStamped, f"wrench_topic/{link_name}/total_torque", 10
                ),
                self.create_publisher(
                    WrenchStamped, f"wrench_topic/{link_name}/friction_torque", 10
                ),
            ]
        ]

        self._trick_variable_server_client.start_listening()

    def trick_data_callback(self, trick_data):
        for i in range(self._ndof):
            total_torque_var_name = self._trick_variable_names[2 * i]
            friction_torque_var_name = self._trick_variable_names[2 * i + 1]
            total_torque_val = float(trick_data[total_torque_var_name])
            friction_torque_val = float(trick_data[friction_torque_var_name])
            total_torque_pub = self.wrench_publishers[2 * i]
            friction_torque_pub = self.wrench_publishers[2 * i + 1]
            now = self.get_clock().now().to_msg()

            msg = WrenchStamped()
            msg.header = Header()
            msg.header.frame_id = self.link_frames[i]
            msg.header.stamp = now

            # send total torque
            msg.wrench.torque.x = self.rot_axes[i][0] * total_torque_val
            msg.wrench.torque.y = self.rot_axes[i][1] * total_torque_val
            msg.wrench.torque.z = self.rot_axes[i][2] * total_torque_val
            total_torque_pub.publish(msg)

            # send friction torque
            msg.wrench.torque.x = self.rot_axes[i][0] * friction_torque_val
            msg.wrench.torque.y = self.rot_axes[i][1] * friction_torque_val
            msg.wrench.torque.z = self.rot_axes[i][2] * friction_torque_val
            friction_torque_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WrenchPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

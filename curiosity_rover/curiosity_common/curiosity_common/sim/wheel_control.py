"""This module allows the wheels control between two simulation environments.

### Supoprted Simulation Environments
# - GazeboSim
#    - GazeboSim uses ROS2 Control for joint control.
# - IsaacSim
#   - IsaacSim expects joint states to be published on a topic.
"""

from typing import List

from rclpy.node import Node  # type: ignore

from builtin_interfaces.msg import Duration  # type: ignore
from std_msgs.msg import Float64MultiArray  # type: ignore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # type: ignore
from geometry_msgs.msg import Twist  # type: ignore


class BaseMobileControl:
    """Base class for mobile control."""

    _instance = None
    _node: Node = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, node: Node):
        self._node = node

    def set_wheel_speed(self, mobile_positions: List[float]):
        """Set mobile states."""
        raise NotImplementedError


class GazeboMobileController(BaseMobileControl):
    """Mobile control for GazeboSim."""

    def __init__(self, node: Node):
        super().__init__(node)

        self.wheel_publisher_ = self._node.create_publisher(
            Float64MultiArray,
            "/wheel_velocity_controller/commands",
            10,
        )

        self.suspension_publisher_ = self._node.create_publisher(
            Float64MultiArray,
            "/wheel_tree_position_controller/commands",
            10,
        )

        self.steer_publisher_ = self._node.create_publisher(
            JointTrajectory,
            "/steer_position_controller/joint_trajectory",
            10,
        )

    def set_wheel_speed(
        self,
        mobile_positions: List[float],
    ):
        """Set mobile states."""

        if len(mobile_positions) is not 6:
            raise ValueError("Mobile positions should be 6 floats.")

        msg = Float64MultiArray()
        msg.data = mobile_positions
        self.wheel_publisher_.publish(msg)

    def set_suspension(
        self,
        mobile_positions: List[float],
    ):
        """Set mobile states."""

        if len(mobile_positions) is not 4:
            raise ValueError("Mobile positions should be 4 floats.")

        msg = Float64MultiArray()
        msg.data = mobile_positions
        self.suspension_publisher_.publish(msg)

    def set_steering(self, joint_names: List[str], joint_positions: List[float]):
        """Set joint states."""
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=1)
        point.positions = joint_positions
        trajectory.points.append(point)
        self.steer_publisher_.publish(trajectory)


class IsaacMobileController(BaseMobileControl):
    """Mobile control for IsaacSim."""

    def __init__(self, node: Node):
        super().__init__(node)

        self.wheel_publisher_ = self._node.create_publisher(
            Twist,
            "/curiosity/cmd_vel",
            10,
        )

    def set_cmd_vel(self, msg: Twist):
        """Forward Twist message to IsaacSim."""
        self.wheel_publisher_.publish(msg)

    def set_wheel_speed(
        self,
        mobile_positions: List[float],
    ):
        """Set mobile states."""
        return

    def set_suspension(
        self,
        _: List[float],
    ):
        """Set mobile states."""
        return

    def set_steer(self, _: List[str], __: List[float]):
        """Set joint states."""
        return

"""This module allows the joints control between two simulation environments.

### Supoprted Simulation Environments
 - GazeboSim
    - GazeboSim uses ROS2 Control for joint control.
 - IsaacSim
    - IsaacSim expects joint states to be published on a topic.
"""

from typing import List

from rclpy.node import Node  # type: ignore

from builtin_interfaces.msg import Duration  # type: ignore
from sensor_msgs.msg import JointState  # type: ignore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # type: ignore


class BaseJointControl:
    """Base class for joint control."""

    _instance = None
    _node: Node = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, node: Node):
        self._node = node

    def get_joint_states(self):
        """Get joint states."""
        raise NotImplementedError

    def set_joint_states(self, joint_names: List[str], joint_positions: List[float]):
        """Set joint states."""
        raise NotImplementedError


class GazeboJointController(BaseJointControl):
    """Joint control for GazeboSim."""

    def __init__(self, node: Node):
        super().__init__(node)

        self.arm_publisher_ = self._node.create_publisher(
            JointTrajectory,
            "/canadarm_joint_trajectory_controller/joint_trajectory",
            10,
        )

    def set_joint_states(self, joint_names: List[str], joint_positions: List[float]):
        """Set joint states."""

        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=4)

        trajectory.points.append(point)

        self.arm_publisher_.publish(trajectory)


class IsaacJointController(BaseJointControl):
    """Joint control for IsaacSim."""

    def __init__(self, node: Node):
        super().__init__(node)

        self.arm_publisher_ = self._node.create_publisher(
            JointState,
            "/canadarm2/joint_command",
            10,
        )

    def set_joint_states(self, joint_names: List[str], joint_positions: List[float]):
        """Set joint states."""

        joint_state = JointState()
        joint_state.name = joint_names
        joint_state.position = joint_positions

        self.arm_publisher_.publish(joint_state)

"""Canadarm2 demo launch file."""

from launch import LaunchDescription  # type: ignore
from launch.actions import DeclareLaunchArgument  # type: ignore
from launch.substitutions import LaunchConfiguration  # type: ignore
from launch_ros.actions import Node  # type: ignore


def generate_launch_description():
    """Generate launch description with multiple components."""

    environment_arg = DeclareLaunchArgument(
        "environment",
        default_value="gazebosim",
        description="Environment to run the node in",
    )

    run_move_arm = Node(
        package="canadarm_demo",
        executable="move_arm",
        output="screen",
        parameters=[{"environment": LaunchConfiguration("environment")}],
    )

    return LaunchDescription(
        [
            environment_arg,
            run_move_arm,
        ]
    )

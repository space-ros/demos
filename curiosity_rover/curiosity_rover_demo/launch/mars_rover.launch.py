"""Launch the curiosity rover demo."""

from launch import LaunchDescription  # type: ignore
from launch.actions import DeclareLaunchArgument  # type: ignore
from launch.substitutions import LaunchConfiguration  # type: ignore
from launch_ros.actions import Node, SetParameter  # type: ignore


def generate_launch_description():
    """Launch the curiosity rover demo."""

    environment_arg = DeclareLaunchArgument(
        "environment",
        default_value="gazebosim",
        description="Environment to run the demo in",
    )

    arm_node = Node(
        package="curiosity_rover_demo",
        executable="move_arm",
        output="screen",
        parameters=[{"environment": LaunchConfiguration("environment")}],
    )

    mast_node = Node(
        package="curiosity_rover_demo",
        executable="move_mast",
        output="screen",
        parameters=[{"environment": LaunchConfiguration("environment")}],
    )

    wheel_node = Node(
        package="curiosity_rover_demo",
        executable="move_wheel",
        output="screen",
        parameters=[{"environment": LaunchConfiguration("environment")}],
    )

    run_node = Node(
        package="curiosity_rover_demo", executable="run_demo", output="screen"
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            environment_arg,
            arm_node,
            mast_node,
            wheel_node,
            run_node,
        ]
    )

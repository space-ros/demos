"""Launch the curiosity rover demo."""

from launch import LaunchDescription  # type: ignore
from launch_ros.actions import Node, SetParameter  # type: ignore


def generate_launch_description():
    """Launch the curiosity rover demo."""

    arm_node = Node(
        package="curiosity_rover_demo", executable="move_arm", output="screen"
    )

    mast_node = Node(
        package="curiosity_rover_demo", executable="move_mast", output="screen"
    )

    wheel_node = Node(
        package="curiosity_rover_demo", executable="move_wheel", output="screen"
    )

    run_node = Node(
        package="curiosity_rover_demo", executable="run_demo", output="screen"
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            arm_node,
            mast_node,
            wheel_node,
            run_node,
        ]
    )

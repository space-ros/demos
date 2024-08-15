"""Canadarm2 demo launch file."""

from launch import LaunchDescription  # type: ignore

from launch_ros.actions import Node  # type: ignore


def generate_launch_description():
    """Generate launch description with multiple components."""

    # run_node = Node(
    #    package="canadarm",
    #    executable="move_joint_server",
    #    output='screen'
    # )

    run_move_arm = Node(package="canadarm_demo", executable="move_arm", output="screen")

    return LaunchDescription(
        [
            # run_node,
            run_move_arm,
        ]
    )

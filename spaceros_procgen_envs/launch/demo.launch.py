#!/usr/bin/env -S ros2 launch
"""
Interactive demo showcasing Space ROS ProcGen Environments

This script launches a procedural simulation environment via `../scripts/run_spaceros.py`
while also conveniently initializing a teleoperation node for controlling the robot
via keyboard, together with RViz2 for basic visualization.

A basic set of arguments can be passed to the script to select the task and customize
the simulation environment. For more advanced configurations, please refer to the script itself.

Examples:
    ros2 launch spaceros_procgen_envs demo.launch.py
    ros2 launch spaceros_procgen_envs demo.launch.py task:=sample_collection num_envs:=16
"""

from os import path
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    task = LaunchConfiguration("task")
    num_envs = LaunchConfiguration("num_envs")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    process = [
        ExecuteProcess(
            cmd=[
                PathJoinSubstitution(
                    [EnvironmentVariable("ISAAC_SIM_PYTHON", default_value="python3")]
                ),
                path.join(
                    path.dirname(path.dirname(path.realpath(__file__))),
                    "scripts",
                    "run_spaceros.py",
                ),
                "--task",
                task,
                "--num_envs",
                num_envs,
                "--disable_ui",
            ],
            output="screen",
            shell=True,
            emulate_tty=True,
        )
    ]
    nodes = [
        Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            output="screen",
            arguments=[
                "--ros-args",
                "--log-level",
                log_level,
            ],
            remappings=[("cmd_vel", "/robot/cmd_vel")],
            parameters=[{"use_sim_time": use_sim_time}],
            prefix="xterm -e",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=[
                "--display-config",
                rviz_config,
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(enable_rviz),
        ),
    ]

    return LaunchDescription(declared_arguments + process + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        DeclareLaunchArgument(
            "task",
            default_value="spaceros/perseverance",
            description="Name of the demo/task.",
        ),
        DeclareLaunchArgument(
            "num_envs",
            default_value="1",
            description="Number of environments to simulate.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "enable_rviz", default_value="true", description="Flag to enable RViz2."
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                path.dirname(path.dirname(path.realpath(__file__))),
                "config",
                "rviz",
                "default.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]

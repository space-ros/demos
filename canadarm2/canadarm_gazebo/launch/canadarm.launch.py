"""Canadarm2 Gazebo launch file."""

import os
from launch import LaunchDescription  # type: ignore
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription, SetEnvironmentVariable  # type: ignore
from launch.event_handlers import OnProcessExit  # type: ignore
from launch_ros.actions import Node  # type: ignore
from launch.substitutions import PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory  # type: ignore

import xacro  # type: ignore


def generate_launch_description():
    """Generate launch description with multiple components."""

    canadarm_gazebo_path = get_package_share_directory("canadarm_gazebo")

    simulation_models_path = get_package_share_directory("canadarm_description")

    env_gz_plugin = SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', 
        os.pathsep.join(
            [
                os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", default=""),
                os.environ.get("LD_LIBRARY_PATH", default="")
            ]
    ))
    env_gz_resource = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", 
        os.pathsep.join(
            [
                os.environ.get("GZ_SIM_RESOURCE_PATH", default=""),
                canadarm_gazebo_path + "/models",
            ]
    ))


    urdf_model_path = os.path.join(
        simulation_models_path,
        "models",
        "urdf",
        "SSRMS_Canadarm2.urdf.xacro",
    )
    leo_model = os.path.join(canadarm_gazebo_path, "worlds", "simple.world")

    doc = xacro.process_file(
        urdf_model_path, mappings={"xyz": "1.0 0.0 1.5", "rpy": "3.1416 0.0 0.0"}
    )
    robot_description = {"robot_description": doc.toxml()}

    start_world = IncludeLaunchDescription(
            PathJoinSubstitution([get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
            launch_arguments = [
               ('gz_args', [
                   leo_model,
                   ' -r',
                   ' -v 4' 
               ])
            ]   
    )    
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "canadarm",
            "-topic",
            robot_description,
        ],
        output="screen",
    )

    # Control
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_canadarm_joint_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "canadarm_joint_trajectory_controller",
        ],
        output="screen",
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription(
        [   
            env_gz_plugin,
            env_gz_resource,
            start_world,
            robot_state_publisher,
            spawn,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_canadarm_joint_controller],
                )
            ),
            gz_sim_bridge
        ]
    )


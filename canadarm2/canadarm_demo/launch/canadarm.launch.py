from http.server import executable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnExecutionComplete
import os
from os import environ

from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():
    # ld = LaunchDescription()

    canadarm_demos_path = get_package_share_directory('canadarm')
    simulation_models_path = get_package_share_directory('simulation')

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_RESOURCE_PATH':
           ':'.join([environ.get('IGN_GAZEBO_RESOURCE_PATH', default=''), canadarm_demos_path])}


    urdf_model_path = os.path.join(simulation_models_path, 'models', 'canadarm', 'urdf', 'SSRMS_Canadarm2.urdf.xacro')
    leo_model = os.path.join(canadarm_demos_path, 'worlds', 'simple.world')


    doc = xacro.process_file(urdf_model_path, mappings={'xyz' : '1.0 0.0 1.5', 'rpy': '3.1416 0.0 0.0'})
    robot_description = {'robot_description': doc.toxml()}


    #run_node = Node(
    #    package="canadarm",
    #    executable="move_joint_server",
    #    output='screen'
    #)

    run_move_arm = Node(
        package="canadarm",
        executable="move_arm",
        output='screen'
    )


    start_world = ExecuteProcess(
        cmd=['ign gazebo', leo_model, '-r'],
        output='screen',
        additional_env=env,
        shell=True
    )


    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description])

    spawn = Node(
        package='ros_ign_gazebo', executable='create',
        arguments=[
            '-name', 'canadarm',
            '-topic', robot_description,
        ],
        output='screen'
    )


    # Control
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_canadarm_joint_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'canadarm_joint_trajectory_controller'],
        output='screen'
    )



    return LaunchDescription([
        start_world,
        robot_state_publisher,
        spawn,
        #run_node,
        run_move_arm,

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
    ])

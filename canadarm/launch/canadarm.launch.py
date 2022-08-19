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

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')])}


    mars_rover_demos_path = os.path.join(
        get_package_share_directory('canadarm'))

    urdf_model_path = os.path.join(mars_rover_demos_path, 'urdf/SSRMS_Canadarm2.urdf')
    sdf_model_path = os.path.join(mars_rover_demos_path, 'sdf/model.sdf')
    mars_world_model = os.path.join(FindPackageShare(package='canadarm').find('canadarm'), 'worlds/simple.world')


    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}


    run_node = Node(
        package="canadarm",
        executable="move_joint_server",
        output='screen'
    )

    start_world = ExecuteProcess(
        cmd=['ign gazebo', mars_world_model, '-r'],
        output='screen',
        additional_env=env,
        shell=True
    )


    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[])

    spawn = Node(
        package='ros_ign_gazebo', executable='create',
        arguments=[
            '-name', 'canadarm',
            '-x','1.0',
            '-z','-7.8',
            '-y','0.0',
            '-string', doc.toxml(),
            '-allow_renaming', 'true'
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
             'canadarm_joint_controller'],
        output='screen'
    )



    return LaunchDescription([
        start_world,
        robot_state_publisher,
        spawn,
        run_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=start_world,
                on_exit=[spawn],
            )
        ),
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
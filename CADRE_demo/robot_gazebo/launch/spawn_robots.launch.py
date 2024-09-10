#!/usr/bin/python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_gazebo = get_package_share_directory('robot_gazebo')

    # Declare arguments for world file
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_robot_gazebo, 'worlds', 'empty_world.world'),
        description='Path to the SDF world file'
    )

    # Declare arguments for Robot 1
    robot_name_1_arg = DeclareLaunchArgument('robot_name_1', default_value='robot_1')
    robot_file_1_arg = DeclareLaunchArgument('robot_file_1', default_value='robot.xacro')
    x_spawn_1_arg = DeclareLaunchArgument('x_spawn_1', default_value='0.0')
    y_spawn_1_arg = DeclareLaunchArgument('y_spawn_1', default_value='4.5')
    z_spawn_1_arg = DeclareLaunchArgument('z_spawn_1', default_value='2.2')
    roll_spawn_1_arg = DeclareLaunchArgument('roll_spawn_1', default_value='0.0')
    pitch_spawn_1_arg = DeclareLaunchArgument('pitch_spawn_1', default_value='0.0')
    yaw_spawn_1_arg = DeclareLaunchArgument('yaw_spawn_1', default_value='1.57')

    # Declare arguments for Robot 2
    robot_name_2_arg = DeclareLaunchArgument('robot_name_2', default_value='robot_2')
    robot_file_2_arg = DeclareLaunchArgument('robot_file_2', default_value='robot1.xacro')
    x_spawn_2_arg = DeclareLaunchArgument('x_spawn_2', default_value='4.5')
    y_spawn_2_arg = DeclareLaunchArgument('y_spawn_2', default_value='0.0')
    z_spawn_2_arg = DeclareLaunchArgument('z_spawn_2', default_value='2.5')
    roll_spawn_2_arg = DeclareLaunchArgument('roll_spawn_2', default_value='0.0')
    pitch_spawn_2_arg = DeclareLaunchArgument('pitch_spawn_2', default_value='0.0')
    yaw_spawn_2_arg = DeclareLaunchArgument('yaw_spawn_2', default_value='0.0')

    # Include Gazebo launch description
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    # Define nodes for Robot 1
    robot_1_nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=[LaunchConfiguration('robot_name_1'), '_robot_state_publisher'],
            parameters=[{
                'robot_description': LaunchConfiguration('robot_file_1')
            }],
            remappings=[
                ('/robot_description', [LaunchConfiguration('robot_name_1'), '_robot_description']),
                ('/joint_states', [LaunchConfiguration('robot_name_1'), '/joint_states'])
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name=[LaunchConfiguration('robot_name_1'), '_joint_state_broadcaster'],
            namespace=LaunchConfiguration('robot_name_1'),
            arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name=[LaunchConfiguration('robot_name_1'), '_forward_velocity_controller'],
            namespace=LaunchConfiguration('robot_name_1'),
            arguments=['forward_velocity_controller', '--controller-manager', 'controller_manager'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=[LaunchConfiguration('robot_name_1'), '_spawn_entity'],
            arguments=[
                '-entity', LaunchConfiguration('robot_name_1'),
                '-x', LaunchConfiguration('x_spawn_1'),
                '-y', LaunchConfiguration('y_spawn_1'),
                '-z', LaunchConfiguration('z_spawn_1'),
                '-R', LaunchConfiguration('roll_spawn_1'),
                '-P', LaunchConfiguration('pitch_spawn_1'),
                '-Y', LaunchConfiguration('yaw_spawn_1'),
                '-topic', [LaunchConfiguration('robot_name_1'), '_robot_description']
            ],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LaunchConfiguration('robot_name_1'), '_static_transform_publisher'],
            arguments=['0', '0', '0', '0', '0', '0', 'world', [LaunchConfiguration('robot_name_1'), '_odom']],
            output='screen'
        )
    ]

    # Define nodes for Robot 2
    robot_2_nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=[LaunchConfiguration('robot_name_2'), '_robot_state_publisher'],
            parameters=[{
                'robot_description': LaunchConfiguration('robot_file_2')
            }],
            remappings=[
                ('/robot_description', [LaunchConfiguration('robot_name_2'), '_robot_description']),
                ('/joint_states', [LaunchConfiguration('robot_name_2'), '/joint_states'])
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name=[LaunchConfiguration('robot_name_2'), '_joint_state_broadcaster'],
            namespace=LaunchConfiguration('robot_name_2'),
            arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name=[LaunchConfiguration('robot_name_2'), '_forward_velocity_controller'],
            namespace=LaunchConfiguration('robot_name_2'),
            arguments=['forward_velocity_controller', '--controller-manager', 'controller_manager'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=[LaunchConfiguration('robot_name_2'), '_spawn_entity'],
            arguments=[
                '-entity', LaunchConfiguration('robot_name_2'),
                '-x', LaunchConfiguration('x_spawn_2'),
                '-y', LaunchConfiguration('y_spawn_2'),
                '-z', LaunchConfiguration('z_spawn_2'),
                '-R', LaunchConfiguration('roll_spawn_2'),
                '-P', LaunchConfiguration('pitch_spawn_2'),
                '-Y', LaunchConfiguration('yaw_spawn_2'),
                '-topic', [LaunchConfiguration('robot_name_2'), '_robot_description']
            ],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[LaunchConfiguration('robot_name_2'), '_static_transform_publisher'],
            arguments=['0', '0', '0', '0', '0', '0', 'world', [LaunchConfiguration('robot_name_2'), '_odom']],
            output='screen'
        )
    ]

    return LaunchDescription([
        world_arg,
        robot_name_1_arg,
        robot_file_1_arg,
        x_spawn_1_arg,
        y_spawn_1_arg,
        z_spawn_1_arg,
        roll_spawn_1_arg,
        pitch_spawn_1_arg,
        yaw_spawn_1_arg,
        robot_name_2_arg,
        robot_file_2_arg,
        x_spawn_2_arg,
        y_spawn_2_arg,
        z_spawn_2_arg,
        roll_spawn_2_arg,
        pitch_spawn_2_arg,
        yaw_spawn_2_arg,
        gazebo_launch,
        *robot_1_nodes,
        *robot_2_nodes
    ])

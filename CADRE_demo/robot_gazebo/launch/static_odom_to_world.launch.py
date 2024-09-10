#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):

    entity_name = LaunchConfiguration('robot_name').perform(context)
    odom_frame_name = entity_name + "_odom"
    name_node = odom_frame_name + "_static_transform_publisher"

    st_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name_node,
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', odom_frame_name],
            parameters=[{"use_sim_time": True}]
    )

    return [st_pub]

def generate_launch_description(): 

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='robot')

    return LaunchDescription([
        robot_name_arg,
        OpaqueFunction(function = launch_setup)
        ])

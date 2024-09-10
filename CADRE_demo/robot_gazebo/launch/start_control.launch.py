#!/usr/bin/python3

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction



def launch_setup(context, *args, **kwargs):

    robot_name = LaunchConfiguration('robot_name').perform(context)
    spawn_controller_1_name = robot_name + "_spawn_controller_joint_state_broadcaster"
    spawn_controller_2_name = robot_name + "_spawn_controller_forward_velocity_controller"
    controller_manager_name = "controller_manager"



    spawn_controller_1 = Node(
        package="controller_manager",
        executable="spawner",
        name=spawn_controller_1_name,
        namespace=robot_name,
        arguments=["joint_state_broadcaster", "--controller-manager", controller_manager_name],
        output="screen"
    )
    
    spawn_controller_2 = Node(
        package="controller_manager",
        executable="spawner",
        name=spawn_controller_2_name,
        namespace=robot_name,
        arguments=["forward_velocity_controller", "--controller-manager", controller_manager_name],
        output="screen"
    )

    return [spawn_controller_1, spawn_controller_2]


def generate_launch_description(): 

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='robot')

    return LaunchDescription([
        robot_name_arg,
        OpaqueFunction(function = launch_setup)
        ])
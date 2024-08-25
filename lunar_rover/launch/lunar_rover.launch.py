from http.server import executable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnExecutionComplete
import os
from os import environ

from ament_index_python.packages import get_package_share_directory

import xacro



# . ../spaceros_ws/install/setup.bash && . ../depends_ws/install/setup.bash
# rm -rf build install log && colcon build && . install/setup.bash

def generate_launch_description():

    lunar_rover_demos_path = get_package_share_directory('lunar_rover')

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_RESOURCE_PATH':
           ':'.join([environ.get('IGN_GAZEBO_RESOURCE_PATH', default=''), lunar_rover_demos_path])}
    
    lunar_world_model = os.path.join(lunar_rover_demos_path, 'worlds', 'moon.world')

    start_world = ExecuteProcess(
        cmd=['ign gazebo', lunar_world_model, '-r'],
        output='screen',
        additional_env=env,
        shell=True
    )

    # Set right parameters here
    ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/model/lunar_roving_vehicle/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            ],
            output='screen')

 
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        start_world,
        ros_gz_bridge,
    ])

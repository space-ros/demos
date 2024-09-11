import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get directories
    pkg_ugv_sim = get_package_share_directory('custom_gz_plugins')

    # Set robot name and bridge config
    robot_name = 'buggy'
    bridge_config = os.path.join(pkg_ugv_sim, 'config', robot_name + '_bridge.yaml')

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'vehicle_dust_demo.sdf'],
        output='screen'
    )

    # Launch the ROS-GZ bridge
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        ros_gz_bridge
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get directories
    custom_pkg = get_package_share_directory('custom_gz_plugins')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set robot name and bridge config
    robot_name = 'ingenuity'
    world_name = 'drone_dust_demo'
    bridge_config = os.path.join(custom_pkg, 'config', robot_name + '_bridge.yaml')

    world_path = os.path.join(custom_pkg, 'worlds', world_name + '.sdf')

    # launch GZ Sim with empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args' : world_path + " -r"
        }.items()          
    )

    # Launch the ROS-GZ bridge
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    return LaunchDescription([
        # gz_sim,
        ros_gz_bridge
    ])
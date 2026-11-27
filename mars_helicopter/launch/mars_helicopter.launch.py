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



# . ../spaceros_ws/install/setup.bash && . ../depends_ws/install/setup.bash
# rm -rf build install log && colcon build && . install/setup.bash

def generate_launch_description():

    mars_helicopter_demos_path = get_package_share_directory('mars_helicopter')
    mars_helicopter_models_path = get_package_share_directory('simulation')

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_RESOURCE_PATH':
           ':'.join([environ.get('IGN_GAZEBO_RESOURCE_PATH', default=''), mars_helicopter_demos_path])}
    
    urdf_model_path = os.path.join(mars_helicopter_models_path, 'models', 'ingenuity', 'urdf', 'model.urdf')
    mars_world_model = os.path.join(mars_helicopter_demos_path, 'worlds', 'jezero_crater.world')

    #doc = xacro.process_file(urdf_model_path)
    #robot_description = {'robot_description': doc.toxml()}

    # hover_node = Node(
    #     package="mars_helicopter",
    #     executable="teleop_helicopter",
    #     output='screen'
    # )

    start_world = ExecuteProcess(
        cmd=['ign gazebo', mars_world_model, '-r'],
        output='screen',
        additional_env=env,
        shell=True
    )

    gz_ros2_bridge_yaml = os.path.join(mars_helicopter_demos_path, 'config', 'gz_ros2_bridge.yaml')

    # ROS 2 to Ignition bridge for joint states and TF
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gz_ros2_bridge_yaml}],
        output='screen'
    )
    

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        start_world,
        bridge_node,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
    ])
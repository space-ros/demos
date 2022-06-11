from http.server import executable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from os import environ

def generate_launch_description():
    ld = LaunchDescription()

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')])}


    urdf_model_path = os.path.join(FindPackageShare(package='mars_rover').find('mars_rover'), 'urdf/curiosity_mars_rover.xacro')
    mars_world_model = os.path.join(FindPackageShare(package='mars_rover').find('mars_rover'), 'worlds/mars_curiosity.world')

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=urdf_model_path,
        description="mars rover urdf"
    )

    test_node = Node(
        package="mars_rover",
        executable="test_node",
        parameters=[
            {"robot_description": Command(['xacro ', LaunchConfiguration('model')])}
        ]
    )

    start_world = ExecuteProcess(
        cmd=['ign gazebo', mars_world_model],
        output='screen',
        additional_env=env,
        shell=True
    )

    params = {'use_sim_time': True, 'robot_description': Command(['xacro ', LaunchConfiguration('model')])}
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
            '-name', 'curiosity_mars_rover',
            '-x','1.0',
            '-z','0.0',
            '-y','0.0',
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    ld.add_action(model_arg)
    ld.add_action(test_node)
    ld.add_action(start_world)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn)

    return ld
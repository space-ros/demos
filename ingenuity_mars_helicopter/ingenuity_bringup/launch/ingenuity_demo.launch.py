import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Paths
    package_path = get_package_share_directory('ingenuity_description')  # Replace 'simulation' with your package name
    urdf_file = os.path.join(package_path,'urdf', 'ingenuity_description.urdf')
    sdf_file_path = os.path.join(package_path, 'sdf', 'ingenuity_world.sdf')

    # Load URDF
    urdf_content = ''
    with open(urdf_file, 'r') as file:
        urdf_content = file.read()

    params = {'robot_description': urdf_content}
    print(urdf_content)

    # Nodes
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('ingenuity_description'),
        'rviz',
        'ingenuity_config.rviz'
    )

    # Create a Node action to start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    gz_ros2_bridge_yaml = os.path.join(get_package_share_directory('ingenuity_bringup'), 'config', 'gz_ros2_bridge.yaml')

    # ROS 2 to Ignition bridge for joint states and TF
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gz_ros2_bridge_yaml}],
        output='screen'
    )

    # Launch Description
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('gz_args', [f' -r -v 4 {sdf_file_path}'])]),
        
        node_robot_state_publisher,
        bridge_node,
        rviz_node,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
    ])

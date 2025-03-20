from http.server import executable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnExecutionComplete
import os
from os import environ

from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():
    # ld = LaunchDescription()

    canadarm_demos_path = get_package_share_directory('canadarm')
    canadarm_models_path = get_package_share_directory('simulation')
    
    sim_resource_path = os.pathsep.join(
            [
                environ.get("GZ_SIM_RESOURCE_PATH", default=""),
                os.path.join( canadarm_models_path, 'models' )
            ]
    )
    env_gz_sim = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', sim_resource_path)

    plugin_path = os.pathsep.join(
            [
                environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", default=""),
                environ.get("LD_LIBRARY_PATH", default=""),
            ]
    )
    env_gz_plugin = SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path)

    urdf_model_path = os.path.join(canadarm_models_path, 'models', 'canadarm', 'urdf', 'SSRMS_Canadarm2.urdf.xacro')
    leo_model = os.path.join(canadarm_demos_path, 'worlds', 'simple.sdf')


    doc = xacro.process_file(urdf_model_path, mappings={'xyz' : '1.0 0.0 1.5', 'rpy': '3.1416 0.0 0.0'})
    robot_description_content = doc.toxml()
    robot_description = {'robot_description': robot_description_content}


    #run_node = Node(
    #    package="canadarm",
    #    executable="move_joint_server",
    #    output='screen'
    #)

    run_move_arm = Node(
        package="canadarm",
        executable="move_arm",
        output='screen'
    )

    gz_launch = IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
            launch_arguments = [
               ('gz_args', [
                   leo_model,
                   ' -r',
                   ' -v 4' 
               ])
            ]   
    )

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description,
            {"use_sim_time": True}])
    
    spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_canadarm',
            output='screen',
            arguments=[
              "-string", 
              robot_description_content, 
              "-name", 'canadarm', 
              "-allow_renaming", "true",
          ] 
        )      


    # Control
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen'
    )
        
    canadarm_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["canadarm_joint_trajectory_controller", "-c", "/controller_manager"],
        output='screen',
    )

    # Make the /clock topic available in ROS
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),    
        env_gz_sim,
        env_gz_plugin,
        gz_launch,
        robot_state_publisher,
        spawn,
        ##run_node,
        run_move_arm,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[canadarm_joint_controller_spawner],
            )
        ),
        gz_sim_bridge
    ])

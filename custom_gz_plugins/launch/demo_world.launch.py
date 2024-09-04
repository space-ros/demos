from http.server import executable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnExecutionComplete
import os
from os import environ

from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():
    mars_rover_demos_path = get_package_share_directory('mars_rover')
    mars_rover_models_path = get_package_share_directory('simulation')

    env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'GZ_SIM_RESOURCE_PATH':
           ':'.join([mars_rover_demos_path, mars_rover_models_path + '/models'])}

    urdf_model_path = os.path.join(mars_rover_models_path, 'models', 'curiosity_path',
        'urdf', 'curiosity_mars_rover.xacro.urdf')

    doc = xacro.process_file(urdf_model_path)
    robot_description = {'robot_description': doc.toxml()}

    declare_world_name_arg = DeclareLaunchArgument('world_name', default_value='mars_gale_crater_patch1.sdf',
                                                    description='Name of the world file to launch in Gazebo')
    world_name = LaunchConfiguration('world_name')

    arm_node = Node(
        package="mars_rover",
        executable="move_arm",
        output='screen'
    )

    mast_node = Node(
        package="mars_rover",
        executable="move_mast",
        output='screen'
    )

    wheel_node = Node(
        package="mars_rover",
        executable="move_wheel",
        output='screen'
    )

    run_node = Node(
        package="mars_rover",
        executable="run_demo",
        output='screen'
    )

    start_world = ExecuteProcess(
        cmd=['gz sim', world_name, '-r'],
        output='screen',
        additional_env=env,
        shell=True
    )

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description])

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-name', 'curiosity_path',
            '-topic', robot_description,
        ],
        output='screen'
    )

    ## Control Components

    component_state_msg = '{name: "GazeboSimSystem", target_state: {id: 3, label: ""}}'

    ## a hack to resolve current bug
    set_hardware_interface_active = ExecuteProcess(
        cmd=['ros2', 'service', 'call', 
            'controller_manager/set_hardware_component_state',
            'controller_manager_msgs/srv/SetHardwareComponentState',
            component_state_msg]
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_joint_trajectory_controller'],
        output='screen'
    )

    load_mast_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'mast_joint_trajectory_controller'],
        output='screen'
    )

    load_wheel_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'wheel_velocity_controller'],
        output='screen'
    )

    load_steer_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'steer_position_controller'],
        output='screen'
    )

    load_suspension_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'wheel_tree_position_controller'],
        output='screen'
    )

    return LaunchDescription([
        declare_world_name_arg,
        start_world,
        robot_state_publisher,
        spawn,
        arm_node,
        mast_node,
        wheel_node,
        run_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[set_hardware_interface_active],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=set_hardware_interface_active,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_joint_traj_controller,
                        load_mast_joint_traj_controller,
                        load_wheel_joint_traj_controller,
                        load_steer_joint_traj_controller,
                        load_suspension_joint_traj_controller],
            )
        ),
    ])
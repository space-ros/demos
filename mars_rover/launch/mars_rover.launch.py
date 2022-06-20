from http.server import executable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnExecutionComplete
import os
from os import environ

def generate_launch_description():
    # ld = LaunchDescription()

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

    arm_node = Node(
        package="mars_rover",
        executable="move_arm",
        parameters=[
            {"robot_description": Command(['xacro ', LaunchConfiguration('model')])}
        ],
        output='screen'
    )

    mast_node = Node(
        package="mars_rover",
        executable="move_mast",
        parameters=[
            {"robot_description": Command(['xacro ', LaunchConfiguration('model')])}
        ],
        output='screen'
    )

    wheel_node = Node(
        package="mars_rover",
        executable="move_wheel",
        parameters=[
            {"robot_description": Command(['xacro ', LaunchConfiguration('model')])}
        ],
        output='screen'
    )

    start_world = ExecuteProcess(
        cmd=['ign gazebo', mars_world_model, '-r'],
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
            '-z','-7.0',
            '-y','0.0',
            '-topic', '/robot_description'
        ],
        output='screen'
    )


    ## Control Components

    component_state_msg = '{name: "IgnitionSystem", target_state: {id: 3, label: ""}}'

    ## a hack to resolve current bug
    set_hardware_interface_active = ExecuteProcess(
        cmd=['ros2', 'service', 'call', 
            'controller_manager/set_hardware_component_state',
            'controller_manager_msgs/srv/SetHardwareComponentState',
            component_state_msg]
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'arm_joint_trajectory_controller'],
        output='screen'
    )

    load_mast_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'mast_joint_trajectory_controller'],
        output='screen'
    )

    load_wheel_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'wheel_velocity_controller'],
        output='screen'
    )

    load_steer_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'steer_position_controller'],
        output='screen'
    )

    load_suspension_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'wheel_tree_position_controller'],
        output='screen'
    )




    return LaunchDescription([
        model_arg,
        start_world,
        robot_state_publisher,
        spawn,
        arm_node,
        mast_node,
        wheel_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=start_world,
                on_exit=[spawn],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[set_hardware_interface_active],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=set_hardware_interface_active,
                on_exit=[load_joint_state_broadcaster,
                        load_arm_joint_traj_controller,
                        load_mast_joint_traj_controller,
                        load_wheel_joint_traj_controller,
                        load_steer_joint_traj_controller,
                        load_suspension_joint_traj_controller],
            )
        ),
    ])
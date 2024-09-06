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

    lunar_pole_exploration_rover_demos_path = get_package_share_directory('lunar_pole_exploration_rover')
    lunar_pole_exploration_rover_models_path = get_package_share_directory('simulation')

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_RESOURCE_PATH':
           ':'.join([environ.get('IGN_GAZEBO_RESOURCE_PATH', default=''), lunar_pole_exploration_rover_demos_path])}

    urdf_model_path = os.path.join(lunar_pole_exploration_rover_models_path, 'models', 'lunar_pole_exploration_rover',
        'urdf', 'lunar_pole_exploration_rover.xacro')
    lunar_pole_world_model = os.path.join(lunar_pole_exploration_rover_demos_path, 'worlds', 'lunar_pole.world')

    doc = xacro.process_file(urdf_model_path)
    robot_description = {'robot_description': doc.toxml()}

    mast_node = Node(
        package="lunar_pole_exploration_rover",
        executable="move_mast",
        output='screen'
    )

    wheel_node = Node(
        package="lunar_pole_exploration_rover",
        executable="move_wheel",
        output='screen'
    )

    run_node = Node(
        package="lunar_pole_exploration_rover",
        executable="run_demo",
        output='screen'
    )

    odom_node = Node(
        package="lunar_pole_exploration_rover",
        executable="odom_tf_publisher",
        output='screen'
    )

    start_world = ExecuteProcess(
        cmd=['ign gazebo', lunar_pole_world_model, '-r'],
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

    ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                'aft_cam_left/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '/aft_cam_right/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '/nav_cam_left/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '/nav_cam_right/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '/haz_cam_left_front/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '/haz_cam_left_rear/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '/haz_cam_right_front/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '/haz_cam_right_rear/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '/model/lunar_pole_exploration_rover/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/model/lunar_pole_exploration_rover/left_solar_panel/solar_panel_output@std_msgs/msg/Float32@ignition.msgs.Float',
                '/model/lunar_pole_exploration_rover/right_solar_panel/solar_panel_output@std_msgs/msg/Float32@ignition.msgs.Float',
                '/model/lunar_pole_exploration_rover/rear_solar_panel/solar_panel_output@std_msgs/msg/Float32@ignition.msgs.Float',
                '/model/lunar_pole_exploration_rover/battery/rechargeable_battery/state@sensor_msgs/msg/BatteryState@ignition.msgs.BatteryState',
            ],
            output='screen')

    navcam_left_image_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['navcam_left/image_raw', 'navcam_left/image_raw'],
            output='screen')

    navcam_right_image_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['navcam_right/image_raw', 'navcam_right/image_raw'],
            output='screen')

    aftcam_left_image_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['aftcam_left/image_raw', 'aftcam_left/image_raw'],
            output='screen')

    aftcam_right_image_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['aftcam_right/image_raw', 'aftcam_right/image_raw'],
            output='screen')

    hazcam_left_front_image_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['hazcam_left_front/image_raw', 'hazcam_left_front/image_raw'],
            output='screen')

    hazcam_left_rear_image_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['hazcam_left_rear/image_raw', 'hazcam_left_rear/image_raw'],
            output='screen')

    hazcam_right_front_image_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['hazcam_right_front/image_raw', 'hazcam_right_front/image_raw'],
            output='screen')

    hazcam_right_rear_image_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['hazcam_right_rear/image_raw', 'hazcam_right_rear/image_raw'],
            output='screen')

    spawn = Node(
        package='ros_ign_gazebo', executable='create',
        arguments=[
            '-name', 'lunar_pole_exploration_rover',
            '-topic', robot_description,
            '-z', '0.0',
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
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_mast_camera_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'mast_camera_joint_trajectory_controller'],
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


    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        start_world,
        robot_state_publisher,
        spawn,
        mast_node,
        wheel_node,
        run_node,
        odom_node,
        ros_gz_bridge,
        navcam_left_image_bridge,
        navcam_right_image_bridge,
        aftcam_left_image_bridge,
        aftcam_right_image_bridge,
        hazcam_left_front_image_bridge,
        hazcam_left_rear_image_bridge,
        hazcam_right_front_image_bridge,
        hazcam_right_rear_image_bridge,

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
                on_exit=[load_mast_camera_joint_traj_controller,
                         load_wheel_joint_traj_controller,
                         load_steer_joint_traj_controller],
            )
        ),
    ])

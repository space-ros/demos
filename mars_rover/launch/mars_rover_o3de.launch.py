"""
Bring up nodes, hardware interfaces to control a Curiosity Rover in Open 3D Engine.

Author:
Azmyin Md. Kamal,
Ph.D. student in MIE,
Louisiana State University,
Louisiana, USA

Date: August 29th, 2024
Version: 1.0

AI: ChatGPT 4.o

"""

# Imports
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node, SetParameter
from launch.event_handlers import OnProcessExit
import os
from os import environ
import xacro

def generate_launch_description():
    """Generate launch description to bringup curiosity rover simulation."""
    # Define paths to mars_rover and simulation packages
    mars_rover_demos_path = get_package_share_directory('mars_rover')
    mars_rover_models_path = get_package_share_directory('simulation')
    # Define 
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_RESOURCE_PATH':
           ':'.join([environ.get('IGN_GAZEBO_RESOURCE_PATH', default=''), mars_rover_demos_path])}
    
    urdf_model_path = os.path.join(mars_rover_models_path, 'models', 'curiosity_path',
        'urdf', 'curiosity_mars_rover.xacro')
    
    mars_world_model = os.path.join(mars_rover_demos_path, 'worlds', 'mars_curiosity.world')
    
    doc = xacro.process_file(urdf_model_path)
    robot_description = {'robot_description': doc.toxml()}
    
    # Arm
    # arm_node = Node(
    #     package="mars_rover",
    #     executable="move_arm",
    #     output='screen'
    # )
    # Sensor mast
    # mast_node = Node(
    #     package="mars_rover",
    #     executable="move_mast",
    #     output='screen'
    # )
    # Wheel groups
    wheel_node = Node(
        package="mars_rover",
        executable="move_wheel",
        output='screen'
    )
    # Node to take teleop
    teleop_rover_node = Node(
        package="mars_rover",
        executable="teleop_rover",
        output='screen'
    )
    # Publish odometry
    # odom_node = Node(
    #     package="mars_rover",
    #     executable="odom_tf_publisher",
    #     output='screen'
    # )

    # Fire up Gazebo Ignition
    start_world = ExecuteProcess(
        cmd=['ign gazebo', mars_world_model, '-r'],
        output='screen',
        additional_env=env,
        shell=True
    )
    # Publish various joint positions and link orientation
    # This is crucial for simulation
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description])
    # Bridge communication between ROS 2 and the Ignition Gazebo simulation
    ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/model/curiosity_mars_rover/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            ],
            output='screen')
    # Connect image messages from Gazebo to ROS2
    image_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/image_raw', '/image_raw'],
            output='screen')
    # Spawn curiosity rover into the world.
    spawn = Node(
        package='ros_ign_gazebo', executable='create',
        arguments=[
            '-name', 'curiosity_mars_rover',
            '-topic', robot_description,
            '-z', '-7.5'
        ],
        output='screen'
    )

    # Control Components
    component_state_msg = '{name: "IgnitionSystem", target_state: {id: 3, label: ""}}'
    
    # TODO what bug was resolved with this hack?
    # Set hardware_interface state to active
    set_hardware_interface_active = ExecuteProcess(
        cmd=['ros2', 'service', 'call', 
            'controller_manager/set_hardware_component_state',
            'controller_manager_msgs/srv/SetHardwareComponentState',
            component_state_msg]
    )
    
    # Controller to broadcast all joint states
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    # Controller for arm
    load_arm_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_joint_trajectory_controller'],
        output='screen'
    )
    
    # Controller for sensor mast
    load_mast_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'mast_joint_trajectory_controller'],
        output='screen'
    )
    
    # Controller for wheel
    load_wheel_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'wheel_velocity_controller'],
        output='screen'
    )
    # Controller for steering
    load_steer_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'steer_position_controller'],
        output='screen'
    )
    # Controller for suspension joints
    load_suspension_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'wheel_tree_position_controller'],
        output='screen'
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        #start_world,
        #robot_state_publisher,
        #spawn,
        # arm_node,
        # mast_node,
        wheel_node,
        teleop_rover_node, # Renamed from run_node
        # odom_node,
        #ros_gz_bridge,
        #image_bridge,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[set_hardware_interface_active],
            )
        ),
        # After the hardware interface is activated, 
        # the joint state broadcaster is loaded.
        RegisterEventHandler(
            OnProcessExit(
                target_action=set_hardware_interface_active,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        # After the joint state broadcaster is loaded, 
        # all trajectory controllers (arm, mast, wheel, steer, suspension) are loaded and activated.
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[
                        # load_arm_joint_traj_controller,
                        # load_mast_joint_traj_controller,
                        load_wheel_joint_traj_controller,
                        load_steer_joint_traj_controller,
                        load_suspension_joint_traj_controller],
            )
        ),
    ])

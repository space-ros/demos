from http.server import executable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription, SetEnvironmentVariable
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

    mars_rover_demos_path = get_package_share_directory('mars_rover')
    mars_rover_models_path = get_package_share_directory('simulation')

    sim_resource_path = os.pathsep.join(
            [
                environ.get("GZ_SIM_RESOURCE_PATH", default=""),
                os.path.join( mars_rover_models_path, 'models' )
            ]
    )
    env_gz_sim = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', sim_resource_path)

#    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
#           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
#                     environ.get('LD_LIBRARY_PATH', default='')]),
#          } 
    
    urdf_model_path = os.path.join(mars_rover_models_path, 'models', 'curiosity_path',
        'urdf', 'curiosity_mars_rover.xacro')
    mars_world_model = os.path.join(mars_rover_demos_path, 'worlds', 'mars_curiosity.sdf')

    doc = xacro.process_file(urdf_model_path)
    robot_description = {'robot_description': doc.toxml()}

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

    odom_node = Node(
        package="mars_rover",
        executable="odom_tf_publisher",
        output='screen'
    )

    gz_launch = IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
            launch_arguments = [
               ('gz_args', [
                   mars_world_model,
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
            parameters=[robot_description])
    
    ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/curiosity_mars_rover/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
#                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            ],
            output='screen')
            
    image_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/image_raw', '/image_raw'],
            output='screen')

    spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_mars_rover',
            arguments=[
              "-topic", 
              robot_description, 
              "-name", 'curiosity_mars_rover', 
              "-allow_renaming", "true",
              "-z", '-7.5',
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

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    load_arm_joint_traj_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['arm_joint_trajectory_controller', "--controller-manager", "/controller_manager"],
        output='screen'
    )

    load_mast_joint_traj_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mast_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    load_wheel_joint_traj_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['wheel_velocity_controller', "--controller-manager", "/controller_manager"],
        output='screen'
    )

    load_steer_joint_traj_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['steer_position_controller', "--controller-manager", "/controller_manager"],
        output='screen'
    )

    load_suspension_joint_traj_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['wheel_tree_position_controller', "--controller-manager", "/controller_manager"],
        output='screen'
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        env_gz_sim,
        gz_launch,
        robot_state_publisher,
        spawn,
        arm_node,
        mast_node,
        wheel_node,
        run_node,
        odom_node,
        ros_gz_bridge,
#        image_bridge,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
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
                        load_suspension_joint_traj_controller
],
            )
        ),
    ])

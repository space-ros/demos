"""Mars Rover Demo Launch File."""

import os

from http.server import executable
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.substitutions import (
    TextSubstitution,
    PathJoinSubstitution,
    LaunchConfiguration,
    Command,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnExecutionComplete

from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():
    """Launch the mars rover demo."""

    curiosity_gazebo_path = get_package_share_directory("curiosity_gazebo")
    curiosity_rover_models_path = get_package_share_directory("curiosity_description")

    env_gz_plugin = SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', 
        os.pathsep.join(
            [
                os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", default=""),
                os.environ.get("LD_LIBRARY_PATH", default="")
            ]
    ))
    env_gz_resource = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", 
        os.pathsep.join(
            [
                os.environ.get("GZ_SIM_RESOURCE_PATH", default=""),
                curiosity_gazebo_path + "/models",
                curiosity_rover_models_path
            ]
    ))

    urdf_model_path = os.path.join(
        curiosity_rover_models_path,
        "models",
        "urdf",
        "curiosity_mars_rover.xacro",
    )
    mars_world_model = os.path.join(
        curiosity_gazebo_path, "worlds", "mars_curiosity.world"
    )

    doc = xacro.process_file(urdf_model_path)
    robot_description = {"robot_description": doc.toxml()}

    odom_node = Node(
        package="curiosity_gazebo",
        executable="odom_tf_publisher",
        name="odom_tf_publisher",
        output="screen",
    )

    start_world = IncludeLaunchDescription(
            PathJoinSubstitution([get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
            launch_arguments = [
               ('gz_args', [
                   mars_world_model,
                   ' -r',
                   ' -v 4' 
               ])
            ]   
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/curiosity_mars_rover/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        ],
        output="screen",
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/image_raw", "/image_raw"],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "curiosity_mars_rover",
            "-topic",
            robot_description,
            "-z",
            "-7.5",
        ],
        output="screen",
    )

    ## Control Components

    component_state_msg = '{name: "GazeboSystem", target_state: {id: 3, label: ""}}'

    ## a hack to resolve current bug
    set_hardware_interface_active = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "controller_manager/set_hardware_component_state",
            "controller_manager_msgs/srv/SetHardwareComponentState",
            component_state_msg,
        ]
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_arm_joint_traj_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "arm_joint_trajectory_controller",
        ],
        output="screen",
    )

    load_mast_joint_traj_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "mast_joint_trajectory_controller",
        ],
        output="screen",
    )

    load_wheel_joint_traj_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "wheel_velocity_controller",
        ],
        output="screen",
    )

    load_steer_joint_traj_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "steer_position_controller",
        ],
        output="screen",
    )

    load_suspension_joint_traj_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "wheel_tree_position_controller",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            env_gz_plugin,
            env_gz_resource,
            SetParameter(name="use_sim_time", value=True),
            start_world,
            robot_state_publisher,
            spawn,
            odom_node,
            ros_gz_bridge,
            image_bridge,
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
                    on_exit=[
                        load_arm_joint_traj_controller,
                        load_mast_joint_traj_controller,
                        load_wheel_joint_traj_controller,
                        load_steer_joint_traj_controller,
                        load_suspension_joint_traj_controller,
                    ],
                )
            ),
        ]
    )


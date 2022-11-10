from http.server import executable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # ld = LaunchDescription()

    canadarm_demos_path = get_package_share_directory('canadarm')
    simulation_models_path = get_package_share_directory('simulation')

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_RESOURCE_PATH':
           ':'.join([canadarm_demos_path])}


    urdf_model_path = os.path.join(simulation_models_path, 'models', 'canadarm', 'urdf', 'SSRMS_Canadarm2.urdf')
    leo_model = os.path.join(canadarm_demos_path, 'worlds', 'simple.world')

    # doc = xacro.process_file(urdf_model_path)
    # robot_description = {'robot_description': doc.toxml()}
    # planning_context
    moveit_config = (
        MoveItConfigsBuilder("canadarm")
        .robot_description(file_path=urdf_model_path)
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # run_node = Node(
    #     package="canadarm",
    #     executable="move_joint_server",
    #     output='screen'
    # )
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )


    start_world = ExecuteProcess(
        cmd=['ign gazebo', leo_model, '-r'],
        output='screen',
        additional_env=env,
        shell=True
    )

    # RViz
    rviz_config = os.path.join(canadarm_demos_path, "config", "canadarm_config_demo.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ]
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "Base_Joint"],
    )

    # Publish TF
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[moveit_config.robot_description])

    spawn = Node(
        package='ros_ign_gazebo', executable='create',
        arguments=[
            '-name', 'canadarm',
            '-topic', moveit_config.robot_description,
        ],
        output='screen'
    )


    # Control
    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_canadarm_joint_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'canadarm_joint_controller'],
    #     output='screen'
    # )
    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        canadarm_demos_path,
        "config",
        "canadarm_control.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "canadarm_joint_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]


    return LaunchDescription(
        [
            start_world,
            rviz_node,
            static_tf,
            robot_state_publisher,
            spawn,
            run_move_group_node,
            ros2_control_node,
            # run_node,

            # RegisterEventHandler(
            #     OnProcessExit(
            #         target_action=spawn,
            #         on_exit=[load_joint_state_broadcaster],
            #     )
            # ),
            # RegisterEventHandler(
            #     OnProcessExit(
            #         target_action=load_joint_state_broadcaster,
            #         on_exit=[load_canadarm_joint_controller],
            #     )
            # ),
        ]
        + load_controllers
    )

import launch
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    AndSubstitution,
    PathJoinSubstitution
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import os


def generate_launch_description():
    # robot_model = launch_ros.substitutions.FindPackageShare(package="leo_description").find("leo_description")
    pkg_share = FindPackageShare("lunarsim_gz_worlds")
    gz_models_path = PathJoinSubstitution([pkg_share, "models"])


    use_sim_time = LaunchConfiguration("use_sim_time")
    use_localization = LaunchConfiguration("use_localization")
    log_level = LaunchConfiguration("log_level")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration('world')

    world_path = PathJoinSubstitution([pkg_share, "world", world])

    # gazebo have to be executed with shell=False, or test_launch won't terminate it
    #   see: https://github.com/ros2/launch/issues/545
    # This code is form taken ros_gz_sim and modified to work with shell=False
    #   see: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/launch/gz_sim.launch.py.in
    gz_env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
           'GZ_PARTITION': os.environ.get('GZ_PARTITION', default='')}
    gazebo = [
        ExecuteProcess(
            condition=launch.conditions.IfCondition(headless),
            cmd=['ruby', FindExecutable(name="gz"), 'sim',  '-r', '-v', gz_verbosity, '-s', '--headless-rendering', world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        ),
        ExecuteProcess(
            condition=launch.conditions.UnlessCondition(headless),
            cmd=['ruby', FindExecutable(name="gz"), 'sim',  '-r', '-v', gz_verbosity, world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        )
    ]
    
    return launch.LaunchDescription(
        [
            # SetEnvironmentVariable(
            #     name="IGN_GAZEBO_RESOURCE_PATH",
            #     value=gz_models_path,
            # ),
            # SetEnvironmentVariable(
            #     name="IGN_GAZEBO_MODEL_PATH",
            #     value=gz_models_path,
            # ),
            DeclareLaunchArgument(
                name="headless",
                default_value="False",
                description="Start GZ in hedless mode and don't start RViz (overrides use_rviz)",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            DeclareLaunchArgument(
                "gz_verbosity",
                default_value="3",
                description="Verbosity level for Ignition Gazebo (0~4).",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value="",
                description="Extra args for Gazebo (ie. '-s' for running headless)",
            ),
            DeclareLaunchArgument(
                name="log_level",
                default_value="info",
                description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
            ),
            DeclareLaunchArgument(
                'world',
                default_value="dem_moon.sdf",
                description="The world file to be used from the worlds directory"
            )
        ] + gazebo
    )
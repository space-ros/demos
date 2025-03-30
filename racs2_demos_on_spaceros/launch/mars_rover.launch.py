"""Launch the curiosity rover demo."""

from launch import LaunchDescription  # type: ignore
from launch_ros.actions import Node, SetParameter  # type: ignore

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution



def generate_launch_description():
    """Launch the curiosity rover demo."""

    arm_node = Node(
        package="curiosity_rover_demo", executable="move_arm", output="screen"
    )

    mast_node = Node(
        package="curiosity_rover_demo", executable="move_mast", output="screen"
    )

    wheel_node = Node(
        package="curiosity_rover_demo", executable="move_wheel", output="screen"
    )

    run_node = Node(
        package="curiosity_rover_demo", executable="run_demo", output="screen"
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("curiosity_gazebo"),
                "launch",
                "curiosity_gazebo.launch.py"
            ])
        )
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            gazebo_launch, 
            arm_node,
            mast_node,
            wheel_node,
            # run_node,
        ]
    )
    


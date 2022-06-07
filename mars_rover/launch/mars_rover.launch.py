from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    ld = LaunchDescription()


    urdf_model_path = os.path.join(FindPackageShare(package='mars_rover').find('mars_rover'), 'urdf/curiosity_mars_rover.xacro')

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=urdf_model_path,
        description="mars rover urdf"
    )

    test_node = Node(
        package="mars_rover",
        executable="test_node",
        parameters=[
            {"robot_description": Command(['xacro ', LaunchConfiguration('model')])}
        ]
    )

    ld.add_action(model_arg)
    ld.add_action(test_node)

    return ld
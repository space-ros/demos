from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    ld = LaunchDescription()


    # model_arg = DeclareLaunchArgument(
    #     "model", default_value=TextSubstitution(text="$(find curiosity_mars_rover_description)/urdf/curiosity_mars_rover.xacro")
    # )
    urdf_model_path = os.path.join(get_package_share_directory('mars_rover'), 'urdf','curiosity_mars_rover.xacro')

    model = xacro.parse(open(urdf_model_path))
    xacro.process_doc(model)


    test_node = Node(
        package="mars_rover",
        executable="test_node",
        parameters=[
            {"robot_description": model.toxml()}
        ]
    )

    ld.add_action(test_node)

    return ld
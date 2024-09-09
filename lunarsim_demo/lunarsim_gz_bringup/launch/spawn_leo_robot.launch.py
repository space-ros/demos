# Copyright 2023 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import xacro


def spawn_robot(context: LaunchContext, namespace: LaunchConfiguration):
    pkg_project_description = get_package_share_directory("leo_description")

    lunarsim_gz_bringup = get_package_share_directory("lunarsim_gz_bringup")
    robot_ns = context.perform_substitution(namespace)

    robot_desc = xacro.process(
        os.path.join(
            pkg_project_description,
            "urdf",
            "leo_sim.urdf.xacro",
        ),
        mappings={"robot_ns": robot_ns},
    )

    if robot_ns == "":
        robot_gazebo_name = "leo_rover"
        node_name_prefix = ""
    else:
        robot_gazebo_name = "leo_rover_" + robot_ns
        node_name_prefix = robot_ns + "_"

    # Launch robot state publisher node
    robot_state_publisher = Node(
        namespace=robot_ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}, 
            {"robot_description": robot_desc},
        ]
    )

    # Spawn a robot inside a simulation
    leo_rover = Node(
        namespace=robot_ns,
        package="ros_gz_sim",
        executable="create",
        name="ros_gz_sim_create",
        output="both",
        arguments=[
        "-topic",
            "robot_description",
            "-name",
            robot_gazebo_name,
            "-x",
            LaunchConfiguration('x_pose'),
            "-y",
            LaunchConfiguration('y_pose'),
            "-z",
            LaunchConfiguration('z_pose'),
        ],
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=node_name_prefix + "parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(lunarsim_gz_bringup, "config", "leo_ros_gz_bridge.yaml"),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )
    

    # Launch key_teleop.launch.xml from leo_teleop package
    key_teleop_launch =  IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('leo_teleop'),
                 'launch', 'key_teleop.launch.xml']
            )
        )
    )

    return [
        robot_state_publisher,
        leo_rover,
        topic_bridge,
        key_teleop_launch
    ]


def generate_launch_description():
    name_argument = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )

    x_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0',
        description="The x_pose of the rover's starting position"
    )
    
    y_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0',
        description="The y_pose of the rover's starting position"
    )
    
    z_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='560',
        description="The z_pose of the rover's starting position"
    )

    namespace = LaunchConfiguration("robot_ns")

    return LaunchDescription(
        [ x_arg,
          y_arg,
          z_arg,
          name_argument, 
          OpaqueFunction(function=spawn_robot, args=[namespace])
        ]
    )

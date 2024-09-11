# ROS2
from launch import LaunchDescription
from launch_ros.actions import Node
import launch.logging

from ament_index_python.packages import get_package_share_directory

# Internal
from mars_rover_nvidia_isaac.isaac_controller_loader import ControllerLoader

# System
from pathlib import Path

def generate_launch_description():

    logger = launch.logging.get_logger('launch_logger')

    package_share_path = get_package_share_directory("mars_rover_nvidia_isaac")
    param_file_path = Path(package_share_path, 'config',  'isaac_mars_rover_control.yaml')

    controller_loader_config_file = Path(package_share_path, 'config', 'isaac_controller_loader_config.yaml')

    logger.info("Creating cocntroller loader")
    controller_loader = ControllerLoader(param_file_path, controller_loader_config_file)

    # node.get_logger().info("LOADING Following controllers...")
    logger.info("Getting Controllers to load")
    controllers = controller_loader.get_loaded_controllers()
    
    nodes = []
    for name in controllers:
        status, joints = controller_loader.get_joints_for_controller(name)
        if status == "SUPPORTED":
            if joints is None:
                logger.info("No Joints provided for {}. Skipping...".format(name))
            else:
                logger.info("LAUNCHING Controller:: {} for Joints:: {}".format(name, joints))
                node = Node(package='mars_rover_nvidia_isaac',
                            executable=name+'.py',
                            name=name,
                            arguments=[joints],
                            output='screen'
                            )
                nodes.append(node)
        elif status == "UNSUPPORTED":
            logger.info("UNSUPPORTED Controller:: {}. Skipping...".format(name))

    run_node = Node(
        package="mars_rover_nvidia_isaac",
        executable="run_isaac_demo.py",
        output='screen'
    )
    
    nodes.append(run_node)

    rviz_teleop_pkg_path = get_package_share_directory('mars_rover_teleop')
    rviz_config_file = Path(rviz_teleop_pkg_path, 'config', 'rviz_with_teleop.rviz')
    logger.info("RVIZ config file:: {}".format(rviz_config_file))
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config_file)]  # '-d' tells RViz to use the provided config file
    )

    nodes.append(rviz_node)

    

    return LaunchDescription(nodes)
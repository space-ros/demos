from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    ld = LaunchDescription()

    moveit_config = MoveItConfigsBuilder(
        "SSRMS_Canadarm2", package_name="trick_canadarm_moveit_config"
    ).to_moveit_configs()

    ld.add_action(generate_moveit_rviz_launch(moveit_config))
    return ld

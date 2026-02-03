# Copyright 2024 Blazej Fiderek (xfiderek)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events.matchers import matches_action
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    ld = LaunchDescription()

    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="ROS2 namespace"
    )
    node_name_arg = DeclareLaunchArgument(
        "node_name", default_value="ros_trick_bridge", description="Node name"
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="The bridge is a lifecycle node. Set this argument to true to transit to active on startup (start the simulation)",
    )
    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value=[
            FindPackageShare("ros_trick_bridge"),
            "/params/ros_trick_bridge_example_params.yaml",
        ],
        description="ROS Trick Bridge config",
    )

    ros_trick_bridge_lifecycle_node = LifecycleNode(
        package="ros_trick_bridge",
        executable="ros_trick_bridge_node",
        name=LaunchConfiguration("node_name"),
        namespace=LaunchConfiguration("namespace"),
        parameters=[LaunchConfiguration("param_file")],
        output="both",
        emulate_tty=True,
    )

    ros_trick_emit_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ros_trick_bridge_lifecycle_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(PythonExpression([LaunchConfiguration("autostart")])),
    )
    ros_trick_emit_activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ros_trick_bridge_lifecycle_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ),
        condition=IfCondition(PythonExpression([LaunchConfiguration("autostart")])),
    )
    ros_trick_inactive_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ros_trick_bridge_lifecycle_node,
            goal_state="inactive",
            entities=[
                ros_trick_emit_activate_event,
            ],
        ),
        condition=IfCondition(PythonExpression([LaunchConfiguration("autostart")])),
    )

    # Add logging for debugging
    ld.add_action(LogInfo(msg="Launching ROS Trick Bridge Lifecycle Node"))
    ld.add_action(namespace_arg)
    ld.add_action(node_name_arg)
    ld.add_action(autostart_arg)
    ld.add_action(param_file_arg)
    ld.add_action(ros_trick_bridge_lifecycle_node)
    ld.add_action(ros_trick_inactive_state_handler)
    ld.add_action(ros_trick_emit_configure_event)
    ld.add_action(LogInfo(msg="Emitting configure event"))

    # Add event handlers for logging state transitions
    ld.add_action(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=ros_trick_bridge_lifecycle_node,
                start_state="unconfigured",
                goal_state="configuring",
                entities=[
                    LogInfo(msg="Transitioning from 'unconfigured' to 'configuring'")
                ],
            )
        )
    )

    ld.add_action(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=ros_trick_bridge_lifecycle_node,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    LogInfo(msg="Transitioning from 'configuring' to 'inactive'")
                ],
            )
        )
    )

    ld.add_action(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=ros_trick_bridge_lifecycle_node,
                start_state="inactive",
                goal_state="activating",
                entities=[LogInfo(msg="Transitioning from 'inactive' to 'activating'")],
            )
        )
    )

    ld.add_action(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=ros_trick_bridge_lifecycle_node,
                start_state="activating",
                goal_state="active",
                entities=[LogInfo(msg="Transitioning from 'activating' to 'active'")],
            )
        )
    )

    return ld

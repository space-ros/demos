from os import path

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg

import spaceros_procgen_envs.core.assets as asset_utils
import spaceros_procgen_envs.utils.math as math_utils
from spaceros_procgen_envs.core.actions import (
    WheeledRoverActionCfg,
    WheeledRoverActionGroupCfg,
)


def perseverance_cfg(
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    asset_name: str = "robot",
    action_scale: float = 1.0,
    **kwargs,
) -> asset_utils.WheeledRoverCfg:
    frame_base = "body"
    regex_drive_joints = "drive_joint.*"
    regex_steer_joints = "steer_joint.*"

    return asset_utils.WheeledRoverCfg(
        ## Model
        asset_cfg=ArticulationCfg(
            spawn=sim_utils.UsdFileCfg(
                usd_path=path.join(
                    path.dirname(path.realpath(__file__)), "perseverance.usdc"
                ),
                activate_contact_sensors=True,
                collision_props=sim_utils.CollisionPropertiesCfg(
                    contact_offset=0.02, rest_offset=0.005
                ),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    max_linear_velocity=1.5,
                    max_angular_velocity=1000.0,
                    max_depenetration_velocity=1.0,
                    disable_gravity=False,
                ),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False,
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=4,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(),
            actuators={
                "drive_joints": ImplicitActuatorCfg(
                    joint_names_expr=[regex_drive_joints],
                    velocity_limit=40.0,
                    effort_limit=150.0,
                    damping=25000.0,
                    stiffness=0.0,
                ),
                "steer_joints": ImplicitActuatorCfg(
                    joint_names_expr=[regex_steer_joints],
                    velocity_limit=2.0,
                    effort_limit=400.0,
                    damping=200.0,
                    stiffness=500.0,
                ),
                "rocker_joints": ImplicitActuatorCfg(
                    joint_names_expr=["suspension_joint_rocker.*"],
                    velocity_limit=5.0,
                    effort_limit=2500.0,
                    damping=400.0,
                    stiffness=4000.0,
                ),
                "bogie_joints": ImplicitActuatorCfg(
                    joint_names_expr=["suspension_joint_bogie.*"],
                    velocity_limit=4.0,
                    effort_limit=500.0,
                    damping=25.0,
                    stiffness=200.0,
                ),
            },
        ).replace(prim_path=prim_path, **kwargs),
        ## Actions
        action_cfg=WheeledRoverActionGroupCfg(
            WheeledRoverActionCfg(
                asset_name=asset_name,
                wheelbase=(2.26, 2.14764),
                wheelbase_mid=2.39164,
                wheel_radius=0.26268,
                steering_joint_names=[
                    "steer_joint_front_left",
                    "steer_joint_front_right",
                    "steer_joint_rear_left",
                    "steer_joint_rear_right",
                ],
                drive_joint_names=[
                    "drive_joint_front_left",
                    "drive_joint_front_right",
                    "drive_joint_rear_left",
                    "drive_joint_rear_right",
                    "drive_joint_mid_left",
                    "drive_joint_mid_right",
                ],
                scale=action_scale,
            )
        ),
        ## Frames
        frame_base=asset_utils.FrameCfg(
            prim_relpath=frame_base,
        ),
        frame_camera_front=asset_utils.FrameCfg(
            prim_relpath=f"{frame_base}/camera_front",
            offset=asset_utils.TransformCfg(
                # translation=(-0.3437, -0.8537, 1.9793),  # Left Navcam
                translation=(-0.7675, -0.8537, 1.9793),  # Right Navcam
                rotation=math_utils.quat_from_rpy(0.0, 15.0, -90.0),
            ),
        ),
        ## Joints
        regex_drive_joints=regex_drive_joints,
        regex_steer_joints=regex_steer_joints,
    )

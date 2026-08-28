from os import path

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.controllers import DifferentialIKControllerCfg
from omni.isaac.lab.envs.mdp import DifferentialInverseKinematicsActionCfg
from torch import pi

import spaceros_procgen_envs.core.assets as asset_utils
import spaceros_procgen_envs.utils.math as math_utils
from spaceros_procgen_envs.core.actions import ManipulatorTaskSpaceActionCfg
from spaceros_procgen_envs.core.envs import mdp


def canadarm3_large_cfg(
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    asset_name: str = "robot",
    use_relative_mode: bool = True,
    action_scale: float = 0.1,
    **kwargs,
) -> asset_utils.ManipulatorCfg:
    frame_base = "canadarm3_large_0"
    frame_ee = "canadarm3_large_7"
    regex_joints_arm = "canadarm3_large_joint_[1-7]"
    regex_joints_hand = "canadarm3_large_joint_7"

    return asset_utils.ManipulatorCfg(
        ## Model
        asset_cfg=ArticulationCfg(
            spawn=sim_utils.UsdFileCfg(
                usd_path=path.join(
                    path.dirname(path.realpath(__file__)), "canadarm3_large.usdc"
                ),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=True,
                    max_depenetration_velocity=5.0,
                ),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False,
                    solver_position_iteration_count=12,
                    solver_velocity_iteration_count=1,
                ),
                activate_contact_sensors=True,
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                joint_pos={
                    "canadarm3_large_joint_1": 50.0 * pi / 180.0,
                    "canadarm3_large_joint_2": 0.0 * pi / 180.0,
                    "canadarm3_large_joint_3": 55.0 * pi / 180.0,
                    "canadarm3_large_joint_4": 75.0 * pi / 180.0,
                    "canadarm3_large_joint_5": -30.0 * pi / 180.0,
                    "canadarm3_large_joint_6": 0.0 * pi / 180.0,
                    "canadarm3_large_joint_7": 0.0 * pi / 180.0,
                },
            ),
            actuators={
                "joints": ImplicitActuatorCfg(
                    joint_names_expr=["canadarm3_large_joint_[1-7]"],
                    effort_limit=2500.0,
                    velocity_limit=5.0,
                    stiffness=40000.0,
                    damping=25000.0,
                ),
            },
            soft_joint_pos_limit_factor=1.0,
        ).replace(prim_path=prim_path, **kwargs),
        ## Actions
        action_cfg=ManipulatorTaskSpaceActionCfg(
            arm=mdp.DifferentialInverseKinematicsActionCfg(
                asset_name=asset_name,
                joint_names=[regex_joints_arm],
                body_name=frame_ee,
                controller=DifferentialIKControllerCfg(
                    command_type="pose",
                    use_relative_mode=use_relative_mode,
                    ik_method="dls",
                ),
                scale=action_scale,
                body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(
                    pos=(0.0, 0.0, 0.0)
                ),
            ),
            hand=mdp.BinaryJointPositionActionCfg(
                asset_name=asset_name,
                joint_names=[regex_joints_hand],
                close_command_expr={regex_joints_hand: 0.0},
                open_command_expr={regex_joints_hand: 0.0},
            ),
        ),
        ## Frames
        frame_base=asset_utils.FrameCfg(
            prim_relpath=frame_base,
        ),
        frame_ee=asset_utils.FrameCfg(
            prim_relpath=frame_ee,
            offset=asset_utils.TransformCfg(
                translation=(0.0, 0.0, 0.1034),
            ),
        ),
        frame_camera_base=asset_utils.FrameCfg(
            prim_relpath=f"{frame_base}/camera_base",
            offset=asset_utils.TransformCfg(
                translation=(0.06, 0.0, 0.15),
                rotation=math_utils.quat_from_rpy(0.0, -10.0, 0.0),
            ),
        ),
        frame_camera_wrist=asset_utils.FrameCfg(
            prim_relpath=f"{frame_ee}/camera_wrist",
            offset=asset_utils.TransformCfg(
                translation=(0.0, 0.0, -0.45),
                rotation=math_utils.quat_from_rpy(0.0, 90.0, 180.0),
            ),
        ),
        ## Links
        regex_links_arm="canadarm3_large_[0-6]",
        regex_links_hand="canadarm3_large_7",
        ## Joints
        regex_joints_arm=regex_joints_arm,
        regex_joints_hand=regex_joints_hand,
    )

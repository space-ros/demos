from omni.isaac.lab.controllers import DifferentialIKControllerCfg
from omni.isaac.lab.envs.mdp import DifferentialInverseKinematicsActionCfg
from torch import pi

import spaceros_procgen_envs.core.assets as asset_utils
import spaceros_procgen_envs.utils.math as math_utils
from spaceros_procgen_envs.core.actions import ManipulatorTaskSpaceActionCfg
from spaceros_procgen_envs.core.envs import mdp


def franka_cfg(
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    asset_name: str = "robot",
    use_relative_mode: bool = True,
    action_scale: float = 0.1,
    **kwargs,
) -> asset_utils.ManipulatorCfg:
    from omni.isaac.lab_assets.franka import FRANKA_PANDA_HIGH_PD_CFG

    asset_cfg = FRANKA_PANDA_HIGH_PD_CFG.copy()
    asset_cfg.spawn.activate_contact_sensors = True
    asset_cfg.spawn.rigid_props.disable_gravity = True
    asset_cfg.spawn.articulation_props.enabled_self_collisions = False
    asset_cfg.spawn.articulation_props.solver_position_iteration_count = 12
    asset_cfg.spawn.articulation_props.solver_velocity_iteration_count = 1
    asset_cfg.actuators["panda_shoulder"].stiffness *= 10.0
    asset_cfg.actuators["panda_shoulder"].damping *= 10.0
    asset_cfg.actuators["panda_forearm"].stiffness *= 10.0
    asset_cfg.actuators["panda_forearm"].damping *= 10.0
    asset_cfg.init_state = asset_utils.ArticulationCfg.InitialStateCfg(
        joint_pos={
            "panda_joint1": 0.0,
            "panda_joint2": -(pi / 8.0),
            "panda_joint3": 0.0,
            "panda_joint4": -(pi - (pi / 8.0)),
            "panda_joint5": 0.0,
            "panda_joint6": pi - (pi / 4.0),
            "panda_joint7": (pi / 4.0),
            "panda_finger_joint.*": 0.04,
        },
    )
    asset_cfg = asset_cfg.replace(prim_path=prim_path, **kwargs)

    frame_base = "panda_link0"
    frame_ee = "panda_hand"
    regex_joints_arm = "panda_joint.*"
    regex_joints_hand = "panda_finger_joint.*"

    return asset_utils.ManipulatorCfg(
        ## Model
        asset_cfg=asset_cfg,
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
                    pos=(0.0, 0.0, 0.107)
                ),
            ),
            hand=mdp.BinaryJointPositionActionCfg(
                asset_name=asset_name,
                joint_names=[regex_joints_hand],
                close_command_expr={regex_joints_hand: 0.0},
                open_command_expr={regex_joints_hand: 0.04},
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
                translation=(0.07, 0.0, 0.05),
                rotation=math_utils.quat_from_rpy(0.0, -60.0, 180.0),
            ),
        ),
        ## Links
        regex_links_arm="panda_link[1-7]",
        regex_links_hand="panda_(leftfinger|rightfinger)",
        ## Joints
        regex_joints_arm=regex_joints_arm,
        regex_joints_hand=regex_joints_hand,
    )

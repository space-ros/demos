from os import path

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from torch import pi

import spaceros_procgen_envs.core.assets as asset_utils
import spaceros_procgen_envs.utils.math as math_utils
from spaceros_procgen_envs.core.actions import (
    MultiCopterActionCfg,
    MultiCopterActionGroupCfg,
)


def ingenuity_cfg(
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    asset_name: str = "robot",
    action_scale: float = 4.0,
    **kwargs,
) -> asset_utils.MultiCopterCfg:
    frame_base = "body"
    regex_links_rotors = "rotor_[1-2]"
    regex_joints_rotors = "rotor_joint_[1-2]"

    return asset_utils.MultiCopterCfg(
        ## Model
        asset_cfg=ArticulationCfg(
            spawn=sim_utils.UsdFileCfg(
                usd_path=path.join(
                    path.dirname(path.realpath(__file__)), "ingenuity.usdc"
                ),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False,
                    solver_position_iteration_count=8,
                    solver_velocity_iteration_count=1,
                ),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=True,
                    max_depenetration_velocity=5.0,
                ),
                activate_contact_sensors=True,
            ),
            init_state=ArticulationCfg.InitialStateCfg(),
            actuators={
                "rotors": ImplicitActuatorCfg(
                    joint_names_expr=[regex_joints_rotors],
                    velocity_limit=2500 / 60 * 2 * pi,  # 2500 RPM
                    effort_limit=7.5,
                    stiffness=0.0,
                    damping=1000.0,
                ),
            },
            soft_joint_pos_limit_factor=0.0,
        ).replace(prim_path=prim_path, **kwargs),
        ## Actions
        action_cfg=MultiCopterActionGroupCfg(
            flight=MultiCopterActionCfg(
                asset_name=asset_name,
                frame_base=frame_base,
                regex_joints_rotors=regex_joints_rotors,
                nominal_rpm={
                    "rotor_joint_1": 2500.0,
                    "rotor_joint_2": -2500.0,
                },
                tilt_magnitude=0.2,
                scale=action_scale,
            )
        ),
        ## Frames
        frame_base=asset_utils.FrameCfg(
            prim_relpath=frame_base,
        ),
        frame_camera_bottom=asset_utils.FrameCfg(
            prim_relpath=f"{frame_base}/camera_bottom",
            offset=asset_utils.TransformCfg(
                translation=(0.045, 0.0, 0.1275),
                rotation=math_utils.quat_from_rpy(0.0, 90.0, 0.0),
            ),
        ),
        ## Links
        regex_links_rotors=regex_links_rotors,
        ## Joints
        regex_joints_rotors=regex_joints_rotors,
    )

from typing import TYPE_CHECKING, Dict, Tuple

import omni.isaac.lab.utils.math as math_utils
import torch
from omni.isaac.core.prims.xform_prim_view import XFormPrimView
from omni.isaac.lab.managers import SceneEntityCfg
from pxr import Usd

from spaceros_procgen_envs.core.assets import Articulation

if TYPE_CHECKING:
    from spaceros_procgen_envs.core.envs import BaseEnv


def reset_xform_orientation_uniform(
    env: "BaseEnv",
    env_ids: torch.Tensor,
    orientation_distribution_params: Dict[str, Tuple[float, float]],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> Usd.Prim:
    asset: XFormPrimView = env.scene[asset_cfg.name]

    range_list = [
        orientation_distribution_params.get(key, (0.0, 0.0))
        for key in ["roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, device=asset._device)
    rand_samples = math_utils.sample_uniform(
        ranges[:, 0], ranges[:, 1], (len(env_ids), 3), device=asset._device
    )

    orientations = math_utils.quat_from_euler_xyz(
        rand_samples[:, 0], rand_samples[:, 1], rand_samples[:, 2]
    )

    asset.set_world_poses(orientations=orientations)


def reset_joints_by_offset(
    env: "BaseEnv",
    env_ids: torch.Tensor,
    position_range: tuple[float, float],
    velocity_range: tuple[float, float],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Reset the robot joints with offsets around the default position and velocity by the given ranges.

    This function samples random values from the given ranges and biases the default joint positions and velocities
    by these values. The biased values are then set into the physics simulation.
    """
    # Extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]

    # Get default joint state
    joint_pos = asset.data.default_joint_pos[env_ids].clone()
    joint_vel = asset.data.default_joint_vel[env_ids].clone()

    # Bias these values randomly
    joint_pos += math_utils.sample_uniform(
        *position_range, joint_pos.shape, joint_pos.device
    )
    joint_vel += math_utils.sample_uniform(
        *velocity_range, joint_vel.shape, joint_vel.device
    )

    # Clamp joint pos to limits
    joint_pos_limits = asset.data.soft_joint_pos_limits[env_ids]
    joint_pos = joint_pos.clamp_(joint_pos_limits[..., 0], joint_pos_limits[..., 1])
    # Clamp joint vel to limits
    joint_vel_limits = asset.data.soft_joint_vel_limits[env_ids]
    joint_vel = joint_vel.clamp_(-joint_vel_limits, joint_vel_limits)

    # Set into the physics simulation
    joint_indices = asset.find_joints(asset_cfg.joint_names)[0]
    asset.write_joint_state_to_sim(
        joint_pos[:, joint_indices],
        joint_vel[:, joint_indices],
        joint_ids=joint_indices,
        env_ids=env_ids,
    )

from os import path
from typing import Any, Dict

import spaceros_procgen_envs.core.assets as asset_utils
import spaceros_procgen_envs.core.sim as sim_utils
import spaceros_procgen_envs.utils.math as math_utils


def gateway_cfg(
    *,
    prim_path: str = "{ENV_REGEX_NS}/vehicle",
    spawn_kwargs: Dict[str, Any] = {},
    **kwargs,
) -> asset_utils.VehicleCfg:
    if spawn_kwargs.get("collision_props") is None:
        spawn_kwargs["collision_props"] = sim_utils.CollisionPropertiesCfg()

    return asset_utils.VehicleCfg(
        ## Model
        asset_cfg=asset_utils.AssetBaseCfg(
            prim_path=prim_path,
            spawn=sim_utils.UsdFileCfg(
                usd_path=path.join(
                    path.dirname(path.realpath(__file__)), "gateway.usdc"
                ),
                **spawn_kwargs,
            ),
            **kwargs,
        ),
        ## Frames
        frame_manipulator_base=asset_utils.FrameCfg(
            prim_relpath="base",
            offset=asset_utils.TransformCfg(
                translation=(0.0, 0.0, 0.0),
            ),
        ),
        frame_camera_base=asset_utils.FrameCfg(
            prim_relpath="camera_base",
            offset=asset_utils.TransformCfg(
                translation=(0.21, 0.0, 0.0),
                rotation=math_utils.quat_from_rpy(0.0, 45.0, 0.0),
            ),
        ),
        frame_cargo_bay=asset_utils.FrameCfg(
            prim_relpath="cargo_bay",
            offset=asset_utils.TransformCfg(
                translation=(-0.6, 0.0, 0.3),
            ),
        ),
        ## Properties
        height=0.0,
    )

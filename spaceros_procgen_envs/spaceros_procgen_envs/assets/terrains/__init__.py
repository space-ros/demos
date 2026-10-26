from typing import Any, Dict, Optional, Tuple

import spaceros_procgen_envs.core.envs as env_utils
import spaceros_procgen_envs.core.sim as sim_utils
from spaceros_procgen_envs.core.assets import AssetBaseCfg

from .lunar_surface_procgen import LunarSurfaceProcgenCfg
from .martian_surface_procgen import MartianSurfaceProcgenCfg


def terrain_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    size: Tuple[float, float] = (10.0, 10.0),
    num_assets: int = 1,
    prim_path: str = "{ENV_REGEX_NS}/terrain",
    spawn_kwargs: Dict[str, Any] = {},
    procgen_kwargs: Dict[str, Any] = {},
    **kwargs,
) -> Optional[AssetBaseCfg]:
    spawn: Optional[sim_utils.SpawnerCfg] = None

    match env_cfg.assets.terrain.variant:
        case env_utils.AssetVariant.PRIMITIVE:
            spawn = sim_utils.GroundPlaneCfg(size=size, **spawn_kwargs)

        case env_utils.AssetVariant.PROCEDURAL:
            if spawn_kwargs.get("collision_props") is None:
                spawn_kwargs["collision_props"] = sim_utils.CollisionPropertiesCfg()
            usd_file_cfg = sim_utils.UsdFileCfg(
                usd_path="IGNORED",
                **spawn_kwargs,
            )

            match env_cfg.scenario:
                case env_utils.Scenario.MOON:
                    spawn = LunarSurfaceProcgenCfg(
                        num_assets=num_assets,
                        usd_file_cfg=usd_file_cfg,
                        seed=env_cfg.seed,
                        detail=env_cfg.detail,
                    )

                case env_utils.Scenario.MARS:
                    spawn = MartianSurfaceProcgenCfg(
                        num_assets=num_assets,
                        usd_file_cfg=usd_file_cfg,
                        seed=env_cfg.seed,
                        detail=env_cfg.detail,
                    )
                case _:
                    return None

            # Set height to 10% of the average planar size
            scale = (*size, (size[0] + size[1]) / 20.0)
            for node_cfg in spawn.geometry_nodes.values():
                if node_cfg.get("scale") is not None:
                    node_cfg["scale"] = scale

            for key, value in procgen_kwargs.items():
                for node_cfg in spawn.geometry_nodes.values():
                    if node_cfg.get(key) is not None:
                        node_cfg[key] = value
                    elif hasattr(spawn, key):
                        setattr(spawn, key, value)

    if spawn is None:
        return None
    return AssetBaseCfg(
        prim_path=prim_path,
        spawn=spawn,
        **kwargs,
    )

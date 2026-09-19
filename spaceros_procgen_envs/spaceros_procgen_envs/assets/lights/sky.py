from os import path
from typing import Any, Dict, Optional

from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

import spaceros_procgen_envs.core.envs as env_utils
import spaceros_procgen_envs.core.sim as sim_utils
from spaceros_procgen_envs.core.assets import AssetBaseCfg

HDR_DIR = path.join(path.dirname(path.realpath(__file__)), "hdr")


def sky_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    prim_path: str = "/World/sky",
    spawn_kwargs: Dict[str, Any] = {},
    **kwargs,
) -> Optional[AssetBaseCfg]:
    texture_file: Optional[str] = None

    match env_cfg.scenario:
        case env_utils.Scenario.EARTH:
            texture_file = (
                f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
            )
        case env_utils.Scenario.MARS:
            texture_file = path.join(HDR_DIR, "martian_sky_day.hdr")
        case env_utils.Scenario.ORBIT:
            texture_file = path.join(HDR_DIR, "low_lunar_orbit.jpg")

    if texture_file is None:
        return None
    return AssetBaseCfg(
        prim_path=prim_path,
        spawn=sim_utils.DomeLightCfg(
            intensity=0.25 * env_cfg.scenario.light_intensity,
            texture_file=texture_file,
            **spawn_kwargs,
        ),
        **kwargs,
    )

from typing import Any, Dict, Optional

import spaceros_procgen_envs.core.assets as asset_utils
import spaceros_procgen_envs.core.envs as env_utils

from .construction_rover import construction_rover_cfg
from .gateway import gateway_cfg


def vehicle_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    prim_path: str = "{ENV_REGEX_NS}/vehicle",
    spawn_kwargs: Dict[str, Any] = {},
    **kwargs,
) -> Optional[asset_utils.VehicleCfg]:
    vehicle_cfg: Optional[asset_utils.VehicleCfg] = None
    match env_cfg.assets.vehicle.variant:
        case env_utils.AssetVariant.NONE:
            return None

        case _:
            match env_cfg.scenario:
                case env_utils.Scenario.MOON | env_utils.Scenario.MARS:
                    vehicle_cfg = construction_rover_cfg(
                        prim_path=prim_path, spawn_kwargs=spawn_kwargs, **kwargs
                    )
                case env_utils.Scenario.ORBIT:
                    vehicle_cfg = gateway_cfg(
                        prim_path=prim_path, spawn_kwargs=spawn_kwargs, **kwargs
                    )

    return vehicle_cfg

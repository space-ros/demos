from typing import Tuple

import spaceros_procgen_envs.core.envs as env_utils


def contrastive_color_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
) -> Tuple[float, float, float]:
    match env_cfg.scenario:
        case (
            env_utils.Scenario.ASTEROID
            | env_utils.Scenario.MOON
            | env_utils.Scenario.ORBIT
        ):
            return (0.8, 0.8, 0.8)
        case env_utils.Scenario.EARTH | env_utils.Scenario.MARS:
            return (0.2, 0.2, 0.2)
        case _:
            return (0.7071, 0.7071, 0.7071)

import spaceros_procgen_envs.core.assets as asset_utils
import spaceros_procgen_envs.core.envs as env_utils

from .canadarm3 import canadarm3_large_cfg  # noqa: F401
from .franka import franka_cfg
from .ingenuity import ingenuity_cfg
from .perseverance import perseverance_cfg


def rover_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    **kwargs,
) -> asset_utils.MobileRobotCfg:
    return perseverance_cfg(prim_path=prim_path, **kwargs)


def manipulator_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    **kwargs,
) -> asset_utils.ManipulatorCfg:
    return franka_cfg(prim_path=prim_path, **kwargs)


def aerial_robot_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    **kwargs,
) -> asset_utils.AerialRobotCfg:
    return ingenuity_cfg(prim_path=prim_path, **kwargs)

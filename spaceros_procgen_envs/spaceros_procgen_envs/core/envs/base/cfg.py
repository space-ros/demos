from os import path

from omni.isaac.lab.envs import DirectRLEnvCfg
from omni.isaac.lab.utils import configclass

import spaceros_procgen_envs.core.envs as env_utils
from spaceros_procgen_envs.paths import SPACEROS_DEMO_CONFIG_DIR


@configclass
class BaseEnvCfg(DirectRLEnvCfg):
    """
    Extended version of :class:`omni.isaac.lab.envs.DirectRLEnvCfg`.
    """

    ## Updated defaults
    # Disable UI window by default
    ui_window_class_type: type | None = None
    # Redundant: spaces are automatically extracted
    num_actions: int = 0
    # Redundant: spaces are automatically extracted
    num_observations: int = 0

    ## Environment
    env_cfg: env_utils.EnvironmentConfig = env_utils.EnvironmentConfig.extract(
        cfg_path=path.join(SPACEROS_DEMO_CONFIG_DIR, "env.yaml")
    )

    ## Misc
    # Flag that enables the timeout for the environment
    enable_truncation: bool = True

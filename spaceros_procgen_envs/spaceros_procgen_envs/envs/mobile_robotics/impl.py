from spaceros_procgen_envs.core.assets import Articulation
from spaceros_procgen_envs.core.envs import BaseEnv

from .cfg import BaseMobileRoboticsEnvCfg


class BaseMobileRoboticsEnv(BaseEnv):
    cfg: BaseMobileRoboticsEnvCfg

    def __init__(self, cfg: BaseMobileRoboticsEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get handles to scene assets
        self._robot: Articulation = self.scene["robot"]

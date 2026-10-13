from spaceros_procgen_envs.core.assets import Articulation
from spaceros_procgen_envs.core.envs import BaseEnv

from .cfg import BaseAerialRoboticsEnvCfg


class BaseAerialRoboticsEnv(BaseEnv):
    cfg: BaseAerialRoboticsEnvCfg

    def __init__(self, cfg: BaseAerialRoboticsEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get handles to scene assets
        self._robot: Articulation = self.scene["robot"]

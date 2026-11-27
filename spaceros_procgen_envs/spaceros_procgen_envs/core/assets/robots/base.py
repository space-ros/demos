from typing import Any, Union

from spaceros_procgen_envs.core.assets import (
    ArticulationCfg,
    AssetCfg,
    FrameCfg,
    RigidObjectCfg,
)


class RobotCfg(AssetCfg):
    asset_cfg: Union[ArticulationCfg, RigidObjectCfg]

    ## Actions
    action_cfg: Any

    ## Frames
    frame_base: FrameCfg

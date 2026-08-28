from dataclasses import MISSING
from dataclasses import MISSING as DELAYED_CFG
from typing import Callable, List, Literal, Optional, Tuple

from omni.isaac.lab.utils import configclass
from pxr import Usd

import spaceros_procgen_envs.core.sim as sim_utils

from . import impl


@configclass
class MultiAssetCfg(sim_utils.SpawnerCfg):
    """Configuration parameters for loading multiple assets randomly"""

    func: Callable[..., Usd.Prim] = impl.spawn_multi_asset_sequential

    assets_cfg: List[sim_utils.SpawnerCfg] = MISSING
    """List of asset configurations to spawn"""


@configclass
class MultiShapeCfg(MultiAssetCfg):
    assets_cfg: List[sim_utils.SpawnerCfg] = DELAYED_CFG

    size: Tuple[float, float, float] = MISSING
    """Size of cuboid"""

    radius: Optional[float] = None
    """Radius of sphere|cylinder|capsule|cone (default: self.size[0])"""

    height: Optional[float] = None
    """Height of cylinder|capsule|cone (default: self.size[1])"""

    axis: Literal["X", "Y", "Z"] = "Z"
    """Axis of cylinder|capsule|cone"""

    shape_cfg: sim_utils.ShapeCfg = sim_utils.ShapeCfg()
    """Additional configuration applied to all shapes"""

    def __post_init__(self):
        if self.radius is None:
            self.radius = self.size[0]
        if self.height is None:
            self.height = self.size[1]

        # Extract ShapeCfg kwargs while ignoring private attributes and func
        shape_cfg_kwargs = {
            k: v
            for k, v in self.shape_cfg.__dict__.items()
            if not k.startswith("_") and k != "func"
        }

        self.assets_cfg = [
            sim_utils.CuboidCfg(size=self.size, **shape_cfg_kwargs),
            sim_utils.SphereCfg(radius=self.radius, **shape_cfg_kwargs),
            sim_utils.CylinderCfg(
                radius=self.radius,
                height=self.height,
                axis=self.axis,
                **shape_cfg_kwargs,
            ),
            sim_utils.CapsuleCfg(
                radius=self.radius,
                height=self.height,
                axis=self.axis,
                **shape_cfg_kwargs,
            ),
            sim_utils.ConeCfg(
                radius=self.radius,
                height=self.height,
                axis=self.axis,
                **shape_cfg_kwargs,
            ),
        ]

from collections.abc import Callable

from omni.isaac.lab.sim import UsdFileCfg as __UsdFileCfg
from omni.isaac.lab.utils import configclass

from spaceros_procgen_envs.core.sim.schemas import MeshCollisionPropertiesCfg

from . import impl


@configclass
class UsdFileCfg(__UsdFileCfg):
    """
    Extended version of :class:`omni.isaac.lab.sim.UsdFileCfg`.
    """

    func: Callable = impl.spawn_from_usd

    mesh_collision_props: MeshCollisionPropertiesCfg | None = None

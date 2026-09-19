from typing import TYPE_CHECKING, Tuple

import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.core.utils.stage as stage_utils
from pxr import PhysxSchema, Usd, UsdPhysics

from spaceros_procgen_envs.core.sim import clone

if TYPE_CHECKING:
    from . import cfg

from spaceros_procgen_envs.core.sim.spawners.from_files import (
    spawn_from_usd as __spawn_from_usd,
)


@clone
def spawn_from_usd(
    prim_path: str,
    cfg: "cfg.UsdFileCfg",
    translation: Tuple[float, float, float] | None = None,
    orientation: Tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    """
    Extended version of :class:`omni.isaac.lab.sim.spawners.from_files.spawn_from_usd`.
    """

    # Get stage
    stage: Usd.Stage = stage_utils.get_current_stage()
    if not stage.ResolveIdentifierToEditTarget(cfg.usd_path):
        raise FileNotFoundError(f"USD file not found at path: '{cfg.usd_path}'.")

    # Create prim if it doesn't exist
    if not prim_utils.is_prim_path_valid(prim_path):
        prim_utils.create_prim(
            prim_path,
            usd_path=cfg.usd_path,
            translation=translation,
            orientation=orientation,
            scale=cfg.scale,
        )
    # Get prim
    prim: Usd.Prim = stage.GetPrimAtPath(prim_path)

    # Define missing APIs
    _define_missing_apis(prim, cfg)

    # Apply mesh collision properties
    if cfg.mesh_collision_props is not None:
        cfg.mesh_collision_props.func(prim_path, cfg.mesh_collision_props, stage)

    return __spawn_from_usd(prim_path, cfg, translation, orientation)


def _define_missing_apis(prim: Usd.Prim, cfg: "cfg.UsdFileCfg"):
    if cfg.rigid_props is not None and not UsdPhysics.RigidBodyAPI(prim):
        UsdPhysics.RigidBodyAPI.Apply(prim)

    if cfg.collision_props is not None and not UsdPhysics.CollisionAPI(prim):
        UsdPhysics.CollisionAPI.Apply(prim)

    if cfg.mass_props is not None and not UsdPhysics.MassAPI(prim):
        UsdPhysics.MassAPI.Apply(prim)

    if cfg.articulation_props is not None and not UsdPhysics.ArticulationRootAPI(prim):
        UsdPhysics.ArticulationRootAPI.Apply(prim)

    if cfg.fixed_tendons_props is not None:
        if not PhysxSchema.PhysxTendonAxisAPI(prim):
            PhysxSchema.PhysxTendonAxisAPI.Apply(prim)
        if not PhysxSchema.PhysxTendonAxisRootAPI(prim):
            PhysxSchema.PhysxTendonAxisRootAPI.Apply(prim)

    if cfg.joint_drive_props is not None and not UsdPhysics.DriveAPI(prim):
        UsdPhysics.DriveAPI.Apply(prim)

    if cfg.deformable_props is not None and not PhysxSchema.PhysxDeformableBodyAPI(
        prim
    ):
        PhysxSchema.PhysxDeformableBodyAPI.Apply(prim)

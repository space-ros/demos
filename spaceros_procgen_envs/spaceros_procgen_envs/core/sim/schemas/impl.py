from typing import TYPE_CHECKING

import carb
import omni.isaac.core.utils.stage as stage_utils
from omni.physx.scripts import utils as physx_utils
from pxr import PhysxSchema, Usd, UsdPhysics

from spaceros_procgen_envs.core.sim import apply_nested

if TYPE_CHECKING:
    from . import cfg


@apply_nested
def set_mesh_collision_properties(
    prim_path: str,
    cfg: "cfg.MeshCollisionPropertiesCfg",
    stage: Usd.Stage | None = None,
) -> bool:
    """

    Args:
        prim_path: The prim path of parent.
        cfg: The configuration for the collider.
        stage: The stage where to find the prim. Defaults to None, in which case the
            current stage is used.

    Returns:
        True if the properties were successfully set, False otherwise.
    """

    # Apply mesh collision approximation
    if cfg.mesh_approximation is not None:
        if stage is None:
            stage: Usd.Stage = stage_utils.get_current_stage()
        prim: Usd.Prim = stage.GetPrimAtPath(prim_path)

        if physx_utils.hasSchema(prim, "CollisionAPI"):
            carb.log_warn("CollisionAPI is already defined")
            return

        def isPartOfRigidBody(currPrim):
            if currPrim.HasAPI(UsdPhysics.RigidBodyAPI):
                return True

            currPrim = currPrim.GetParent()

            return isPartOfRigidBody(currPrim) if currPrim.IsValid() else False

        if cfg.mesh_approximation == "none" and isPartOfRigidBody(prim):
            carb.log_warn(
                f"setCollider: {prim.GetPath()} is a part of a rigid body. Resetting approximation shape from none (trimesh) to convexHull"
            )
            cfg.mesh_approximation = "convexHull"

        collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
        PhysxSchema.PhysxCollisionAPI.Apply(prim)
        collisionAPI.CreateCollisionEnabledAttr().Set(True)

        api = physx_utils.MESH_APPROXIMATIONS.get(
            cfg.mesh_approximation, 0
        )  # None is a valid value
        if api == 0:
            carb.log_warn(
                f"setCollider: invalid approximation type {cfg.mesh_approximation} provided for {prim.GetPath()}. Falling back to convexHull."
            )
            cfg.mesh_approximation = "convexHull"
            api = physx_utils.MESH_APPROXIMATIONS[cfg.mesh_approximation]
        approximation_api = api.Apply(prim) if api is not None else None
        if cfg.mesh_approximation == "sdf" and cfg.sdf_resolution:
            approximation_api.CreateSdfResolutionAttr().Set(cfg.sdf_resolution)

        meshcollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(prim)

        meshcollisionAPI.CreateApproximationAttr().Set(cfg.mesh_approximation)

    return True

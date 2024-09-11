from collections.abc import Callable
from typing import Literal

from omni.isaac.lab.utils import configclass

from . import impl


@configclass
class MeshCollisionPropertiesCfg:
    func: Callable = impl.set_mesh_collision_properties

    mesh_approximation: (
        Literal[
            "none",
            "convexHull",
            "convexDecomposition",
            "meshSimplification",
            "convexMeshSimplification",
            "boundingCube",
            "boundingSphere",
            "sphereFill",
            "sdf",
        ]
        | None
    ) = None
    """Collision approximation to use for the collision shape."""

    sdf_resolution: int = 128
    """Resolution of the SDF grid used for collision approximation."""

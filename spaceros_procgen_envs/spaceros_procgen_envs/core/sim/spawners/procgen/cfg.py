from dataclasses import MISSING
from os import path
from typing import Any, Callable, Dict, List, Optional

import platformdirs
from omni.isaac.lab.utils import configclass
from pxr import Usd

import spaceros_procgen_envs.core.sim as sim_utils
from spaceros_procgen_envs.paths import SPACEROS_DEMO_SCRIPTS_DIR

from . import impl


@configclass
class ProceduralAssetCfg(sim_utils.SpawnerCfg):
    """
    Configuration for generating procedural assets.
    """

    func: Callable[..., Usd.Prim] = MISSING

    seed: int = 0
    """
    Initial seed used for generating the assets.
    """

    num_assets: int = 4
    """
    Number of assets to generate.
    """

    cache_dir: str = platformdirs.user_cache_dir("spaceros_procgen_envs")
    """
    Directory to cache the generated assets.
    """

    overwrite_min_age: int = -1
    """
    Number of seconds after which to overwrite the cached assets. Disabled if negative.
    """


@configclass
class BlenderNodesAssetCfg(ProceduralAssetCfg):
    """
    Configuration for generating assets using Blender (geometry) nodes.
    """

    func: Callable[..., Usd.Prim] = impl.spawn_blender_procgen_assets

    blender_bin: str = "blender"
    blender_args: List[str] = [
        "--factory-startup",
        "--background",
        "--offline-mode",
        "--enable-autoexec",
        "--python-exit-code",
        "1",
    ]
    bpy_script: str = path.join(
        SPACEROS_DEMO_SCRIPTS_DIR, "blender", "procgen_assets.py"
    )

    # Output
    name: str = MISSING
    autorun_scripts: List[str] = MISSING

    # Export
    ext: str = ".usdz"
    export_kwargs: Dict[str, Any] = {}

    # Geometry
    geometry_nodes: Dict[str, Dict[str, Any]] = MISSING
    decimate_angle_limit: Optional[float] = None
    decimate_face_count: Optional[int] = None

    # Material
    material: Optional[str] = None
    detail: float = 1.0
    texture_resolution: int = 1024

    # Prim
    usd_file_cfg: sim_utils.UsdFileCfg = sim_utils.UsdFileCfg(usd_path="IGNORED")

    def __post_init__(self):
        super().__post_init__()

        if not self.ext.startswith("."):
            self.ext = f".{self.ext}"

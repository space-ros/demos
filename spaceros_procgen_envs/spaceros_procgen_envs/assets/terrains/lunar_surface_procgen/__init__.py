from os import path
from typing import Any, Dict, List

from omni.isaac.lab.utils import configclass

import spaceros_procgen_envs.core.sim as sim_utils
from spaceros_procgen_envs.utils.path import abs_listdir


@configclass
class LunarSurfaceProcgenCfg(sim_utils.BlenderNodesAssetCfg):
    name: str = "lunar_surface"
    autorun_scripts: List[str] = abs_listdir(
        path.join(path.dirname(path.realpath(__file__)), "nodes")
    )
    geometry_nodes: Dict[str, Dict[str, Any]] = {
        "LunarTerrain": {
            "density": 0.1,
            "scale": (10.0, 10.0, 1.0),
            "flat_area_size": 0.0,
            "rock_mesh_boolean": False,
        }
    }
    material = "LunarSurface"
    texture_resolution = 2048

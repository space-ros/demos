from os import path
from typing import Any, Dict, List, Optional

from omni.isaac.lab.utils import configclass

import spaceros_procgen_envs.core.sim as sim_utils
from spaceros_procgen_envs.utils.path import abs_listdir


@configclass
class LunarRockProcgenCfg(sim_utils.BlenderNodesAssetCfg):
    name: str = "lunar_rock"
    autorun_scripts: List[str] = abs_listdir(
        path.join(path.dirname(path.realpath(__file__)), "nodes")
    )

    # Geometry
    geometry_nodes: Dict[str, Dict[str, Any]] = {
        "LunarRock": {
            "detail": 5,  # Level of the subdivision (resolution of the mesh)
            "scale": [0.1, 0.1, 0.075],  # Metric scale of the mesh
            "scale_std": [0.01, 0.01, 0.005],  # Standard deviation of the scale
            "horizontal_cut": False,  # Flag to enable horizontal cut
            "horizontal_cut_offset": 0.0,  # Offset of the horizontal cut with respect to the mesh center
        }
    }
    decimate_face_count: Optional[int] = None
    decimate_angle_limit: Optional[float] = None

    # Material
    material: str = "LunarRock"
    texture_resolution: int = 1024

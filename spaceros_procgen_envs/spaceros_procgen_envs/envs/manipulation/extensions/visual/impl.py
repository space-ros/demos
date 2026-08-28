from typing import Dict, Tuple

import torch
from omni.isaac.lab.scene import InteractiveScene
from omni.isaac.lab.sensors import Camera

from spaceros_procgen_envs.utils import image_proc
from spaceros_procgen_envs.utils import string as string_utils

from .cfg import VisualManipulationEnvExtCfg


class VisualManipulationEnvExt:
    ## Subclass requirements
    common_step_counter: int
    scene: InteractiveScene
    cfg: VisualManipulationEnvExtCfg

    def __init__(self, cfg: VisualManipulationEnvExtCfg, **kwargs):
        ## Extract camera sensors from the scene
        self.__cameras: Dict[
            str,  # Name of the output image
            Tuple[
                Camera,  # Camera sensor
                Tuple[float, float],  # Depth range
            ],
        ] = {
            f"image_{string_utils.sanitize_camera_name(key)}": (
                sensor,
                getattr(cfg.scene, key).spawn.clipping_range,
            )
            for key, sensor in self.scene._sensors.items()
            if type(sensor) == Camera
        }

    def _get_observations(self) -> Dict[str, torch.Tensor]:
        observation = {}
        for image_name, (sensor, depth_range) in self.__cameras.items():
            observation.update(
                image_proc.construct_observation(
                    **image_proc.extract_images(sensor),
                    depth_range=depth_range,
                    image_name=image_name,
                )
            )
        return observation

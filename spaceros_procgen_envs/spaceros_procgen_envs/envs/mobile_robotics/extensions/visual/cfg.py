from dataclasses import MISSING
from typing import Tuple

from omni.isaac.lab.envs import ViewerCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors import CameraCfg
from omni.isaac.lab.sensors.camera.camera_cfg import PinholeCameraCfg
from omni.isaac.lab.utils import configclass

import spaceros_procgen_envs.core.assets as asset_utils
import spaceros_procgen_envs.utils.math as math_utils


@configclass
class VisualMobileRoboticsEnvExtCfg:
    ## Subclass requirements
    agent_rate: int = MISSING
    scene: InteractiveSceneCfg = MISSING
    viewer: ViewerCfg = MISSING
    robot_cfg: asset_utils.WheeledRoverCfg = MISSING

    ## Enabling flags
    enable_camera_scene: bool = True
    enable_camera_front: bool = True

    ## Resolution
    camera_resolution: Tuple[int, int] = (640, 480)
    camera_framerate: int = 0  # 0 matches the agent rate

    def __post_init__(self):
        ## Scene
        self.scene.env_spacing += 4.0

        ## Sensors
        framerate = (
            self.camera_framerate if self.camera_framerate > 0 else self.agent_rate
        )
        # Scene camera
        if self.enable_camera_scene:
            self.scene.camera_scene = CameraCfg(
                prim_path=f"{self.scene.robot.prim_path}/{self.robot_cfg.frame_base.prim_relpath}/camera_scene",
                offset=CameraCfg.OffsetCfg(
                    convention="world",
                    pos=(0.0, 7.5, 5.0),
                    rot=math_utils.quat_from_rpy(0.0, 30.0, -90.0),
                ),
                spawn=PinholeCameraCfg(
                    clipping_range=(0.01, 20.0 - 0.01),
                ),
                width=self.camera_resolution[0],
                height=self.camera_resolution[1],
                update_period=framerate,
                data_types=["rgb", "distance_to_camera"],
            )

        # Front camera
        if self.enable_camera_front:
            self.scene.camera_front = CameraCfg(
                prim_path=f"{self.scene.robot.prim_path}/{self.robot_cfg.frame_camera_front.prim_relpath}",
                offset=CameraCfg.OffsetCfg(
                    convention="world",
                    pos=self.robot_cfg.frame_camera_front.offset.translation,
                    rot=self.robot_cfg.frame_camera_front.offset.rotation,
                ),
                spawn=PinholeCameraCfg(
                    focal_length=5.0,
                    horizontal_aperture=12.0,
                    clipping_range=(0.01, 20.0 - 0.01),
                ),
                width=self.camera_resolution[0],
                height=self.camera_resolution[1],
                update_period=framerate,
                data_types=["rgb", "distance_to_camera"],
            )

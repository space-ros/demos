from dataclasses import MISSING
from typing import Optional, Tuple

from omni.isaac.lab.envs import ViewerCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors import CameraCfg
from omni.isaac.lab.sensors.camera.camera_cfg import PinholeCameraCfg
from omni.isaac.lab.utils import configclass

import spaceros_procgen_envs.core.assets as asset_utils
import spaceros_procgen_envs.utils.math as math_utils


@configclass
class VisualManipulationEnvExtCfg:
    ## Subclass requirements
    agent_rate: int = MISSING
    scene: InteractiveSceneCfg = MISSING
    viewer: ViewerCfg = MISSING
    robot_cfg: asset_utils.ManipulatorCfg = MISSING
    vehicle_cfg: Optional[asset_utils.VehicleCfg] = None

    ## Enabling flags
    enable_camera_scene: bool = True
    enable_camera_base: bool = True
    enable_camera_wrist: bool = True

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
                prim_path="{ENV_REGEX_NS}/camera_scene",
                offset=CameraCfg.OffsetCfg(
                    convention="world",
                    pos=(1.2, 0.0, 0.8),
                    rot=math_utils.quat_from_rpy(0.0, 30.0, 180.0),
                ),
                spawn=PinholeCameraCfg(
                    clipping_range=(0.01, 4.0 - 0.01),
                ),
                width=self.camera_resolution[0],
                height=self.camera_resolution[1],
                update_period=framerate,
                data_types=["rgb", "distance_to_camera"],
            )

        # Robot base camera
        if self.enable_camera_base:
            camera_base_kwargs = {
                "spawn": PinholeCameraCfg(
                    focal_length=5.0,
                    horizontal_aperture=12.0,
                    clipping_range=(0.001, 2.5 - 0.001),
                ),
                "width": self.camera_resolution[0],
                "height": self.camera_resolution[1],
                "update_period": framerate,
                "data_types": ["rgb", "distance_to_camera"],
            }
            if self.vehicle_cfg and self.vehicle_cfg.frame_camera_base:
                self.scene.camera_base = CameraCfg(
                    prim_path=f"{self.scene.vehicle.prim_path}/{self.vehicle_cfg.frame_camera_base.prim_relpath}",
                    offset=CameraCfg.OffsetCfg(
                        convention="world",
                        pos=self.vehicle_cfg.frame_camera_base.offset.translation,
                        rot=self.vehicle_cfg.frame_camera_base.offset.rotation,
                    ),
                    **camera_base_kwargs,
                )
            else:
                self.scene.camera_base = CameraCfg(
                    prim_path=f"{self.scene.robot.prim_path}/{self.robot_cfg.frame_camera_base.prim_relpath}",
                    offset=CameraCfg.OffsetCfg(
                        convention="world",
                        pos=self.robot_cfg.frame_camera_base.offset.translation,
                        rot=self.robot_cfg.frame_camera_base.offset.rotation,
                    ),
                    **camera_base_kwargs,
                )

        # Wrist camera
        if self.enable_camera_wrist:
            self.scene.camera_wrist = CameraCfg(
                prim_path=f"{self.scene.robot.prim_path}/{self.robot_cfg.frame_camera_wrist.prim_relpath}",
                offset=CameraCfg.OffsetCfg(
                    convention="world",
                    pos=self.robot_cfg.frame_camera_wrist.offset.translation,
                    rot=self.robot_cfg.frame_camera_wrist.offset.rotation,
                ),
                spawn=PinholeCameraCfg(
                    focal_length=10.0,
                    horizontal_aperture=16.0,
                    clipping_range=(0.001, 1.5 - 0.001),
                ),
                width=self.camera_resolution[0],
                height=self.camera_resolution[1],
                update_period=framerate,
                data_types=["rgb", "distance_to_camera"],
            )

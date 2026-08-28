from typing import Any

from spaceros_procgen_envs.core.actions import WheeledRoverActionGroupCfg
from spaceros_procgen_envs.core.assets import FrameCfg

from . import RobotCfg


class MobileRobotCfg(RobotCfg):
    ## Actions
    action_cfg: Any


class WheeledRoverCfg(MobileRobotCfg):
    ## Actions
    action_cfg: WheeledRoverActionGroupCfg

    ## Frames
    frame_camera_front: FrameCfg

    ## Joints
    regex_drive_joints: str
    regex_steer_joints: str

from spaceros_procgen_envs.core.actions import ManipulatorTaskSpaceActionCfg
from spaceros_procgen_envs.core.assets import FrameCfg

from . import RobotCfg


class ManipulatorCfg(RobotCfg):
    ## Actions
    action_cfg: ManipulatorTaskSpaceActionCfg

    ## Frames
    frame_ee: FrameCfg
    frame_camera_base: FrameCfg
    frame_camera_wrist: FrameCfg

    ## Links
    regex_links_arm: str
    regex_links_hand: str

    ## Joints
    regex_joints_arm: str
    regex_joints_hand: str

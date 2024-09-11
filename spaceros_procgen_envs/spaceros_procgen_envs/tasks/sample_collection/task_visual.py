from typing import Dict

import torch
from omni.isaac.lab.utils import configclass

from spaceros_procgen_envs.envs import (
    VisualManipulationEnvExt,
    VisualManipulationEnvExtCfg,
)

from .task import Task, TaskCfg


@configclass
class VisualTaskCfg(TaskCfg, VisualManipulationEnvExtCfg):
    def __post_init__(self):
        TaskCfg.__post_init__(self)
        VisualManipulationEnvExtCfg.__post_init__(self)


class VisualTask(Task, VisualManipulationEnvExt):
    cfg: VisualTaskCfg

    def __init__(self, cfg: VisualTaskCfg, **kwargs):
        Task.__init__(self, cfg, **kwargs)
        VisualManipulationEnvExt.__init__(self, cfg, **kwargs)

    def _get_observations(self) -> Dict[str, torch.Tensor]:
        return {
            **Task._get_observations(self),
            **VisualManipulationEnvExt._get_observations(self),
        }

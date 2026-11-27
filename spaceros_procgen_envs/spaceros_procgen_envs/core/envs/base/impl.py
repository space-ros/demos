from typing import Sequence

import gymnasium
import numpy as np
import torch
from omni.isaac.lab.envs import DirectRLEnv
from omni.isaac.lab.managers import ActionManager

from . import BaseEnvCfg


class __PostInitCaller(type):
    def __call__(cls, *args, **kwargs):
        obj = type.__call__(cls, *args, **kwargs)
        obj.__post_init__()
        return obj


class BaseEnv(DirectRLEnv, metaclass=__PostInitCaller):
    """
    Extended version of :class:`omni.isaac.lab.envs.DirectRLEnv`.
    """

    def __init__(self, cfg: BaseEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        if self.cfg.actions:
            self.action_manager = ActionManager(self.cfg.actions, self)
            self.cfg.num_actions = self.action_manager.total_action_dim
            print("[INFO] Action Manager: ", self.action_manager)

    def __post_init__(self):
        self._update_gym_env_spaces()

    def close(self):
        if not self._is_closed:
            if self.cfg.actions:
                del self.action_manager

        super().close()

    def _reset_idx(self, env_ids: Sequence[int]):
        if self.cfg.actions:
            self.action_manager.reset(env_ids)

        if self.cfg.events:
            self.event_manager.reset(env_ids)

        super()._reset_idx(env_ids)

    def _pre_physics_step(self, actions: torch.Tensor):
        if self.cfg.actions:
            self.action_manager.process_action(actions)
        else:
            super()._pre_physics_step(actions)

    def _apply_action(self):
        if self.cfg.actions:
            self.action_manager.apply_action()
        else:
            super()._apply_action()

    def _update_gym_env_spaces(self):
        # Action space
        self.single_action_space = gymnasium.spaces.Box(
            low=-np.inf, high=np.inf, shape=(self.cfg.num_actions,)
        )
        self.action_space = gymnasium.vector.utils.batch_space(
            self.single_action_space, self.num_envs
        )

        # Observation space
        self.single_observation_space = gymnasium.spaces.Dict({})
        for (
            obs_key,
            obs_buf,
        ) in self._get_observations().items():
            self.single_observation_space[obs_key] = gymnasium.spaces.Box(
                low=-np.inf, high=np.inf, shape=obs_buf.shape[1:]
            )
        self.observation_space = gymnasium.vector.utils.batch_space(
            self.single_observation_space, self.num_envs
        )

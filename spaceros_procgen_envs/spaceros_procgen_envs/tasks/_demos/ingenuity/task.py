from typing import Dict, Sequence, Tuple

import torch
from omni.isaac.lab.utils import configclass

import spaceros_procgen_envs.core.envs as env_utils
from spaceros_procgen_envs.envs import BaseAerialRoboticsEnv, BaseAerialRoboticsEnvCfg

##############
### Config ###
##############


@configclass
class TaskCfg(BaseAerialRoboticsEnvCfg):
    def __post_init__(self):
        if self.env_cfg.scenario != env_utils.Scenario.MARS:
            print("[WARN] Ingenuity is on Mars (scenario updated)")
            self.env_cfg.scenario = env_utils.Scenario.MARS

        super().__post_init__()


############
### Task ###
############


class Task(BaseAerialRoboticsEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Pre-compute metrics used in hot loops
        self._max_episode_length = self.max_episode_length

        ## Initialize the intermediate state
        self._update_intermediate_state()

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

    def _get_dones(self) -> Tuple[torch.Tensor, torch.Tensor]:
        # Note: This assumes that `_get_dones()` is called before `_get_rewards()` and `_get_observations()` in `step()`
        self._update_intermediate_state()

        if not self.cfg.enable_truncation:
            self._truncations = torch.zeros_like(self._truncations)

        return self._terminations, self._truncations

    def _get_rewards(self) -> torch.Tensor:
        return self._rewards

    def _get_observations(self) -> Dict[str, torch.Tensor]:
        return {
            "robot_base_pose_w": torch.cat(
                [
                    self._robot.data.body_pos_w[:, 0],
                    self._robot.data.body_quat_w[:, 0],
                ],
                dim=-1,
            )
        }

    ########################
    ### Helper Functions ###
    ########################

    def _update_intermediate_state(self):
        ## Compute other intermediate states
        (
            self._remaining_time,
            self._rewards,
            self._terminations,
            self._truncations,
        ) = _compute_intermediate_state(
            current_action=self.action_manager.action,
            previous_action=self.action_manager.prev_action,
            episode_length_buf=self.episode_length_buf,
            max_episode_length=self._max_episode_length,
        )


#############################
### TorchScript functions ###
#############################


@torch.jit.script
def _compute_intermediate_state(
    *,
    current_action: torch.Tensor,
    previous_action: torch.Tensor,
    episode_length_buf: torch.Tensor,
    max_episode_length: int,
) -> Tuple[
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
]:
    ## Intermediate states
    # Time
    remaining_time = 1 - (episode_length_buf / max_episode_length).unsqueeze(-1)

    ## Rewards
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.05
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.sum(
        torch.square(current_action - previous_action), dim=1
    )

    # Total reward
    rewards = torch.sum(
        torch.stack(
            [
                penalty_action_rate,
            ],
            dim=-1,
        ),
        dim=-1,
    )

    ## Termination and truncation
    truncations = episode_length_buf > (max_episode_length - 1)
    terminations = torch.zeros_like(truncations)

    return (
        remaining_time,
        rewards,
        terminations,
        truncations,
    )

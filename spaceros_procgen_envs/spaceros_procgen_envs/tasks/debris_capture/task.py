from typing import Any, Dict, List, Optional, Sequence, Tuple

import torch
from omni.isaac.lab.managers import EventTermCfg, SceneEntityCfg
from omni.isaac.lab.sensors import ContactSensor, ContactSensorCfg
from omni.isaac.lab.utils import configclass

import spaceros_procgen_envs.core.envs as env_utils
import spaceros_procgen_envs.utils.math as math_utils
from spaceros_procgen_envs import assets
from spaceros_procgen_envs.core.assets import AssetCfg, RigidObject, RigidObjectCfg
from spaceros_procgen_envs.envs import (
    BaseManipulationEnv,
    BaseManipulationEnvCfg,
    BaseManipulationEnvEventCfg,
    mdp,
)

##############
### Config ###
##############


class DebrisCfg(AssetCfg):
    class Config:
        arbitrary_types_allowed = True  # Due to EventTermCfg

    ## Model
    asset_cfg: RigidObjectCfg

    ## Randomization
    state_randomizer: EventTermCfg


@configclass
class TaskCfg(BaseManipulationEnvCfg):
    ## Environment
    episode_length_s: float = 10.0

    ## Task
    is_finite_horizon: bool = False

    ## Events
    @configclass
    class EventCfg(BaseManipulationEnvEventCfg):
        ## Object
        reset_rand_object_state: Optional[EventTermCfg] = None

    events = EventCfg()

    def __post_init__(self):
        if self.env_cfg.scenario != env_utils.Scenario.ORBIT:
            print("[WARN] Debris is usually captured in Orbit (scenario updated)")
            self.env_cfg.scenario = env_utils.Scenario.ORBIT
        self.env_cfg.assets.terrain.variant = env_utils.AssetVariant.NONE

        super().__post_init__()

        ## Simulation
        self.sim.gravity = (0.0, 0.0, 0.0)

        ## Scene
        self.object_cfg = self._object_cfg(
            self.env_cfg,
            num_assets=self.scene.num_envs,
            init_state=RigidObjectCfg.InitialStateCfg(pos=(1.0, 0.0, 0.5)),
            spawn_kwargs={
                "activate_contact_sensors": True,
            },
        )
        self.scene.object = self.object_cfg.asset_cfg

        ## Sensors
        self.scene.contacts_robot_hand_obj = ContactSensorCfg(
            prim_path=f"{self.scene.robot.prim_path}/{self.robot_cfg.regex_links_hand}",
            update_period=0.0,
            # Note: This causes error 'Filter pattern did not match the correct number of entries'
            #       However, it seems to function properly anyway...
            filter_prim_paths_expr=[self.scene.object.prim_path],
        )

        ## Events
        self.events.reset_rand_object_state = self.object_cfg.state_randomizer

    ########################
    ### Helper Functions ###
    ########################

    @staticmethod
    def _object_cfg(
        env_cfg: env_utils.EnvironmentConfig,
        *,
        num_assets: int,
        prim_path: str = "{ENV_REGEX_NS}/sample",
        asset_cfg: SceneEntityCfg = SceneEntityCfg("object"),
        spawn_kwargs: Dict[str, Any] = {},
        **kwargs,
    ) -> DebrisCfg:
        return DebrisCfg(
            asset_cfg=assets.object_of_interest_from_env_cfg(
                env_cfg,
                num_assets=num_assets,
                prim_path=prim_path,
                spawn_kwargs=spawn_kwargs,
                **kwargs,
            ),
            state_randomizer=EventTermCfg(
                func=mdp.reset_root_state_uniform,
                mode="reset",
                params={
                    "asset_cfg": asset_cfg,
                    "pose_range": {
                        "x": (-0.25, 0.25),
                        "y": (-0.25, 0.25),
                        "z": (-0.25, 0.25),
                        "roll": (-torch.pi, torch.pi),
                        "pitch": (-torch.pi, torch.pi),
                        "yaw": (-torch.pi, torch.pi),
                    },
                    "velocity_range": {
                        "x": (-0.2 - 0.05, -0.2 + 0.05),
                        "y": (-0.05, 0.05),
                        "z": (-0.05, 0.05),
                        "roll": (-torch.pi, torch.pi),
                        "pitch": (-torch.pi, torch.pi),
                        "yaw": (-torch.pi, torch.pi),
                    },
                },
            ),
        )


############
### Task ###
############


class Task(BaseManipulationEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get handles to scene assets
        self._contacts_robot_hand_obj: ContactSensor = self.scene[
            "contacts_robot_hand_obj"
        ]
        self._object: RigidObject = self.scene["object"]

        ## Pre-compute metrics used in hot loops
        self._robot_arm_joint_indices, _ = self._robot.find_joints(
            self.cfg.robot_cfg.regex_joints_arm
        )
        self._robot_hand_joint_indices, _ = self._robot.find_joints(
            self.cfg.robot_cfg.regex_joints_hand
        )
        self._max_episode_length = self.max_episode_length
        self._obj_com_offset = self._object.data._root_physx_view.get_coms().to(
            self.device
        )

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
            "robot_joint_pos": self._robot.data.joint_pos,
            "robot_ee_pose_w": torch.cat(
                [
                    self._tf_robot_ee.data.target_pos_w[:, 0, :],
                    self._tf_robot_ee.data.target_quat_w[:, 0, :],
                ],
                dim=-1,
            ),
            "robot_hand_wrench": (
                self._robot.root_physx_view.get_link_incoming_joint_force()[
                    :, self._robot_hand_joint_indices
                ]
            ).mean(dim=1),
        }

    ########################
    ### Helper Functions ###
    ########################

    def _update_intermediate_state(self):
        ## Extract intermediate states
        self._robot_ee_pos_wrt_base = self._tf_robot_ee.data.target_pos_source[:, 0, :]
        self._obj_vel_w = self._object.data.root_vel_w

        ## Compute other intermediate states
        (
            self._remaining_time,
            self._robot_joint_pos_arm,
            self._robot_joint_pos_hand,
            self._robot_ee_rotmat_wrt_base,
            self._obj_com_pos_wrt_robot_ee,
            self._obj_com_rotmat_wrt_robot_ee,
            self._rewards,
            self._terminations,
            self._truncations,
        ) = _compute_intermediate_state(
            current_action=self.action_manager.action,
            previous_action=self.action_manager.prev_action,
            episode_length_buf=self.episode_length_buf,
            max_episode_length=self._max_episode_length,
            robot_arm_joint_indices=self._robot_arm_joint_indices,
            robot_hand_joint_indices=self._robot_hand_joint_indices,
            joint_pos=self._robot.data.joint_pos,
            soft_joint_pos_limits=self._robot.data.soft_joint_pos_limits,
            robot_ee_pos_w=self._tf_robot_ee.data.target_pos_w[:, 0, :],
            robot_ee_quat_w=self._tf_robot_ee.data.target_quat_w[:, 0, :],
            robot_ee_quat_wrt_base=self._tf_robot_ee.data.target_quat_source[:, 0, :],
            robot_arm_contact_net_forces=self._contacts_robot.data.net_forces_w,
            robot_hand_obj_contact_force_matrix=self._contacts_robot_hand_obj.data.force_matrix_w,
            obj_pos_w=self._object.data.root_pos_w,
            obj_quat_w=self._object.data.root_quat_w,
            obj_com_offset=self._obj_com_offset,
            obj_vel_w=self._obj_vel_w,
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
    robot_arm_joint_indices: List[int],
    robot_hand_joint_indices: List[int],
    joint_pos: torch.Tensor,
    soft_joint_pos_limits: torch.Tensor,
    robot_ee_pos_w: torch.Tensor,
    robot_ee_quat_w: torch.Tensor,
    robot_ee_quat_wrt_base: torch.Tensor,
    robot_arm_contact_net_forces: torch.Tensor,
    robot_hand_obj_contact_force_matrix: torch.Tensor,
    obj_pos_w: torch.Tensor,
    obj_quat_w: torch.Tensor,
    obj_com_offset: torch.Tensor,
    obj_vel_w: torch.Tensor,
) -> Tuple[
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
]:
    ## Intermediate states
    # Time
    remaining_time = 1 - (episode_length_buf / max_episode_length).unsqueeze(-1)

    # Robot joint positions
    joint_pos_normalized = math_utils.scale_transform(
        joint_pos,
        soft_joint_pos_limits[:, :, 0],
        soft_joint_pos_limits[:, :, 1],
    )
    robot_joint_pos_arm, robot_joint_pos_hand = (
        joint_pos_normalized[:, robot_arm_joint_indices],
        joint_pos_normalized[:, robot_hand_joint_indices],
    )

    # End-effector pose (position and '6D' rotation)
    robot_ee_rotmat_wrt_base = math_utils.matrix_from_quat(robot_ee_quat_wrt_base)

    # Transformation | Object origin -> Object CoM
    obj_com_pos_w, obj_com_quat_w = math_utils.combine_frame_transforms(
        t01=obj_pos_w,
        q01=obj_quat_w,
        t12=obj_com_offset[:, :3],
        q12=obj_com_offset[:, 3:],
    )

    # Transformation | End-effector -> Object CoM
    obj_com_pos_wrt_robot_ee, obj_com_quat_wrt_robot_ee = (
        math_utils.subtract_frame_transforms(
            t01=robot_ee_pos_w,
            q01=robot_ee_quat_w,
            t02=obj_com_pos_w,
            q02=obj_com_quat_w,
        )
    )
    obj_com_rotmat_wrt_robot_ee = math_utils.matrix_from_quat(obj_com_quat_wrt_robot_ee)

    ## Rewards
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.05
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.sum(
        torch.square(current_action - previous_action), dim=1
    )

    # Penalty: Undesired robot arm contacts
    WEIGHT_UNDERSIRED_ROBOT_ARM_CONTACTS = -0.1
    THRESHOLD_UNDERSIRED_ROBOT_ARM_CONTACTS = 10.0
    penalty_undersired_robot_arm_contacts = WEIGHT_UNDERSIRED_ROBOT_ARM_CONTACTS * (
        torch.max(torch.norm(robot_arm_contact_net_forces, dim=-1), dim=1)[0]
        > THRESHOLD_UNDERSIRED_ROBOT_ARM_CONTACTS
    )

    # Reward: Distance | End-effector <--> Object
    WEIGHT_DISTANCE_EE_TO_OBJ = 1.0
    TANH_STD_DISTANCE_EE_TO_OBJ = 0.25
    reward_distance_ee_to_obj = WEIGHT_DISTANCE_EE_TO_OBJ * (
        1.0
        - torch.tanh(
            torch.norm(obj_com_pos_wrt_robot_ee, dim=-1) / TANH_STD_DISTANCE_EE_TO_OBJ
        )
    )

    # Reward: Object grasped
    WEIGHT_OBJ_GRASPED = 4.0
    THRESHOLD_OBJ_GRASPED = 5.0
    reward_obj_grasped = WEIGHT_OBJ_GRASPED * (
        torch.mean(
            torch.max(torch.norm(robot_hand_obj_contact_force_matrix, dim=-1), dim=-1)[
                0
            ],
            dim=1,
        )
        > THRESHOLD_OBJ_GRASPED
    )

    # Penalty: Object velocity (linear)
    WEIGHT_OBJ_VEL_LINEAR = -2.0
    penalty_obj_vel_linear = WEIGHT_OBJ_VEL_LINEAR * torch.norm(
        obj_vel_w[:, :3], dim=-1
    )

    # Penalty: Object velocity (angular)
    WEIGHT_OBJ_VEL_ANGULAR = -1.0 / (2.0 * torch.pi)
    penalty_obj_vel_angular = WEIGHT_OBJ_VEL_ANGULAR * torch.norm(
        obj_vel_w[:, 3:], dim=-1
    )

    # Total reward
    rewards = torch.sum(
        torch.stack(
            [
                penalty_action_rate,
                penalty_undersired_robot_arm_contacts,
                reward_distance_ee_to_obj,
                reward_obj_grasped,
                penalty_obj_vel_linear,
                penalty_obj_vel_angular,
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
        robot_joint_pos_arm,
        robot_joint_pos_hand,
        robot_ee_rotmat_wrt_base,
        obj_com_pos_wrt_robot_ee,
        obj_com_rotmat_wrt_robot_ee,
        rewards,
        terminations,
        truncations,
    )

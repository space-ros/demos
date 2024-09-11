from typing import Any, Dict, List, Optional, Sequence, Tuple

import torch
from omni.isaac.core.prims.xform_prim_view import XFormPrimView
from omni.isaac.lab.managers import EventTermCfg, SceneEntityCfg
from omni.isaac.lab.sensors import ContactSensor, ContactSensorCfg
from omni.isaac.lab.utils import configclass
from pydantic import NonNegativeInt

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


class PegCfg(AssetCfg):
    class Config:
        arbitrary_types_allowed = True  # Due to EventTermCfg

    ## Model
    asset_cfg: RigidObjectCfg

    ## Geometry
    offset_pos_ends: Tuple[
        Tuple[float, float, float],
        Tuple[float, float, float],
    ]

    ## Rotational symmetry of the peg represented as integer
    #  0: Circle (infinite symmetry)
    #  1: No symmetry (exactly one fit)
    #  n: n-fold symmetry (360/n deg between each symmetry)
    rot_symmetry_n: NonNegativeInt = 1

    ## Randomization
    state_randomizer: Optional[EventTermCfg] = None


class HoleCfg(AssetCfg):
    ## Geometry
    offset_pos_bottom: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    offset_pos_entrance: Tuple[float, float, float]


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
        super().__post_init__()

        ## Scene
        self.object_cfg, self.target_cfg = self._peg_and_hole_cfg(
            self.env_cfg,
            num_assets=self.scene.num_envs,
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.02)),
            spawn_kwargs_peg={
                "activate_contact_sensors": True,
            },
        )
        self.scene.object = self.object_cfg.asset_cfg
        self.scene.target = self.target_cfg.asset_cfg

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
    def _peg_and_hole_cfg(
        env_cfg: env_utils.EnvironmentConfig,
        *,
        size: Tuple[float, float] = (0.05, 0.05, 0.05),
        num_assets: int,
        prim_path_peg: str = "{ENV_REGEX_NS}/peg",
        prim_path_hole: str = "{ENV_REGEX_NS}/hole",
        asset_cfg_peg: SceneEntityCfg = SceneEntityCfg("object"),
        spawn_kwargs_peg: Dict[str, Any] = {},
        spawn_kwargs_hole: Dict[str, Any] = {},
        **kwargs,
    ) -> Tuple[PegCfg, HoleCfg]:
        pose_range_peg = {
            "x": (-0.25 - 0.025, -0.25 + 0.025),
            "y": (-0.05, 0.05),
            "z": (-0.01, -0.01),
            "roll": (torch.pi / 2, torch.pi / 2),
            "yaw": (
                torch.pi / 2 - torch.pi / 16,
                torch.pi / 2 + torch.pi / 16,
            ),
        }
        rot_symmetry_n = 4
        offset_pos_ends = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.2))
        offset_pos_entrance = (0.0, 0.0, 0.02)

        peg_cfg, hole_cfg = assets.peg_in_hole_from_env_cfg(
            env_cfg,
            size=size,
            num_assets=num_assets,
            prim_path_peg=prim_path_peg,
            prim_path_hole=prim_path_hole,
            spawn_kwargs_peg=spawn_kwargs_peg,
            spawn_kwargs_hole=spawn_kwargs_hole,
            **kwargs,
        )

        return PegCfg(
            asset_cfg=peg_cfg,
            offset_pos_ends=offset_pos_ends,
            rot_symmetry_n=rot_symmetry_n,
            state_randomizer=EventTermCfg(
                func=mdp.reset_root_state_uniform,
                mode="reset",
                params={
                    "asset_cfg": asset_cfg_peg,
                    "pose_range": pose_range_peg,
                    "velocity_range": {},
                },
            ),
        ), HoleCfg(
            asset_cfg=hole_cfg,
            offset_pos_entrance=offset_pos_entrance,
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
        self._target: XFormPrimView = self.scene["target"]

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

        ## Initialize buffers
        self._initial_obj_height_w = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )
        self._peg_offset_pos_ends = torch.tensor(
            self.cfg.object_cfg.offset_pos_ends, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1, 1)
        self._peg_rot_symmetry_n = torch.tensor(
            self.cfg.object_cfg.rot_symmetry_n, dtype=torch.int32, device=self.device
        ).repeat(self.num_envs)
        self._hole_offset_pos_bottom = torch.tensor(
            self.cfg.target_cfg.offset_pos_bottom,
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1)
        self._hole_offset_pos_entrance = torch.tensor(
            self.cfg.target_cfg.offset_pos_entrance,
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1)

        ## Initialize the intermediate state
        self._update_intermediate_state()

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        # Update the initial height of the objects
        self._initial_obj_height_w[env_ids] = self._object.data.root_pos_w[env_ids, 2]

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
        self._robot_hand_wrench = (
            self._robot.root_physx_view.get_link_incoming_joint_force()[
                :, self._robot_hand_joint_indices
            ]
        )

        ## Compute other intermediate states
        target_pos_w, target_quat_w = self._target.get_world_poses()
        (
            self._remaining_time,
            self._robot_joint_pos_arm,
            self._robot_joint_pos_hand,
            self._robot_ee_rotmat_wrt_base,
            self._obj_com_pos_wrt_robot_ee,
            self._obj_com_rotmat_wrt_robot_ee,
            self._hole_entrance_pos_wrt_peg_ends,
            self._hole_bottom_pos_wrt_peg_ends,
            self._hole_rotmat_wrt_peg,
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
            target_pos_w=target_pos_w,
            target_quat_w=target_quat_w,
            initial_obj_height_w=self._initial_obj_height_w,
            peg_offset_pos_ends=self._peg_offset_pos_ends,
            peg_rot_symmetry_n=self._peg_rot_symmetry_n,
            hole_offset_pos_bottom=self._hole_offset_pos_bottom,
            hole_offset_pos_entrance=self._hole_offset_pos_entrance,
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
    target_pos_w: torch.Tensor,
    target_quat_w: torch.Tensor,
    initial_obj_height_w: torch.Tensor,
    peg_offset_pos_ends: torch.Tensor,
    peg_rot_symmetry_n: torch.Tensor,
    hole_offset_pos_bottom: torch.Tensor,
    hole_offset_pos_entrance: torch.Tensor,
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

    # Transformation | Object origin -> Peg ends
    _peg_end0_pos_w, _ = math_utils.combine_frame_transforms(
        t01=obj_pos_w,
        q01=obj_quat_w,
        t12=peg_offset_pos_ends[:, 0],
    )
    _peg_end1_pos_w, _ = math_utils.combine_frame_transforms(
        t01=obj_pos_w,
        q01=obj_quat_w,
        t12=peg_offset_pos_ends[:, 1],
    )
    peg_ends_pos_w = torch.stack([_peg_end0_pos_w, _peg_end1_pos_w], dim=1)

    # Transformation | Target origin -> Hole entrance
    hole_entrance_pos_w, _ = math_utils.combine_frame_transforms(
        t01=target_pos_w,
        q01=target_quat_w,
        t12=hole_offset_pos_entrance,
    )

    # Transformation | Target origin -> Hole bottom
    hole_bottom_pos_w, _ = math_utils.combine_frame_transforms(
        t01=target_pos_w,
        q01=target_quat_w,
        t12=hole_offset_pos_bottom,
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

    # Transformation | Peg ends -> Hole entrance
    _hole_entrance_pos_wrt_peg_end0, hole_quat_wrt_peg = (
        math_utils.subtract_frame_transforms(
            t01=peg_ends_pos_w[:, 0],
            q01=obj_quat_w,
            t02=hole_entrance_pos_w,
            q02=target_quat_w,
        )
    )
    _hole_entrance_pos_wrt_peg_end1, _ = math_utils.subtract_frame_transforms(
        t01=peg_ends_pos_w[:, 1],
        q01=obj_quat_w,
        t02=hole_entrance_pos_w,
    )
    hole_entrance_pos_wrt_peg_ends = torch.stack(
        [_hole_entrance_pos_wrt_peg_end0, _hole_entrance_pos_wrt_peg_end1], dim=1
    )
    hole_rotmat_wrt_peg = math_utils.matrix_from_quat(hole_quat_wrt_peg)

    # Transformation | Peg ends -> Hole bottom
    _hole_bottom_pos_wrt_peg_end0, _ = math_utils.subtract_frame_transforms(
        t01=peg_ends_pos_w[:, 0],
        q01=obj_quat_w,
        t02=hole_bottom_pos_w,
    )
    _hole_bottom_pos_wrt_peg_end1, _ = math_utils.subtract_frame_transforms(
        t01=peg_ends_pos_w[:, 1],
        q01=obj_quat_w,
        t02=hole_bottom_pos_w,
    )
    hole_bottom_pos_wrt_peg_ends = torch.stack(
        [_hole_bottom_pos_wrt_peg_end0, _hole_bottom_pos_wrt_peg_end1], dim=1
    )

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

    # Reward: Object lifted
    WEIGHT_OBJ_LIFTED = 8.0
    HEIGHT_OFFSET_OBJ_LIFTED = 0.3
    HEIGHT_SPAN_OBJ_LIFTED = 0.25
    TAHN_STD_HEIGHT_OBJ_LIFTED = 0.05
    obj_target_height_offset = (
        torch.abs(obj_com_pos_w[:, 2] - initial_obj_height_w - HEIGHT_OFFSET_OBJ_LIFTED)
        - HEIGHT_SPAN_OBJ_LIFTED
    ).clamp(min=0.0)
    reward_obj_lifted = WEIGHT_OBJ_LIFTED * (
        1.0 - torch.tanh(obj_target_height_offset / TAHN_STD_HEIGHT_OBJ_LIFTED)
    )

    # Reward: Alignment | Peg -> Hole | Primary Z axis
    WEIGHT_ALIGN_PEG_TO_HOLE_PRIMARY = 8.0
    TANH_STD_ALIGN_PEG_TO_HOLE_PRIMARY = 0.5
    _peg_to_hole_primary_axis_similarity = torch.abs(hole_rotmat_wrt_peg[:, 2, 2])
    reward_align_peg_to_hole_primary = WEIGHT_ALIGN_PEG_TO_HOLE_PRIMARY * (
        1.0
        - torch.tanh(
            (1.0 - _peg_to_hole_primary_axis_similarity)
            / TANH_STD_ALIGN_PEG_TO_HOLE_PRIMARY
        )
    )

    # Reward: Alignment | Peg -> Hole | Secondary XY axes (affected by primary via power)
    WEIGHT_ALIGN_PEG_TO_HOLE_SECONDARY = 4.0
    TANH_STD_ALIGN_PEG_TO_HOLE_SECONDARY = 0.2
    _peg_to_hole_yaw = torch.atan2(
        hole_rotmat_wrt_peg[:, 0, 1], hole_rotmat_wrt_peg[:, 0, 0]
    )
    _symmetry_step = 2 * torch.pi / peg_rot_symmetry_n
    _peg_to_hole_yaw_symmetric_directional = _peg_to_hole_yaw % _symmetry_step
    # Note: Lines above might result in NaN/inf when `peg_rot_symmetry_n=0` (infinite circular symmetry)
    #       However, the following `torch.where()` will handle this case
    _peg_to_hole_yaw_symmetric_normalized = torch.where(
        peg_rot_symmetry_n <= 0,
        0.0,
        torch.min(
            _peg_to_hole_yaw_symmetric_directional,
            _symmetry_step - _peg_to_hole_yaw_symmetric_directional,
        )
        / (_symmetry_step / 2.0),
    )
    reward_align_peg_to_hole_secondary = WEIGHT_ALIGN_PEG_TO_HOLE_SECONDARY * (
        1.0
        - torch.tanh(
            _peg_to_hole_yaw_symmetric_normalized.pow(
                _peg_to_hole_primary_axis_similarity
            )
            / TANH_STD_ALIGN_PEG_TO_HOLE_SECONDARY
        )
    )

    # Reward: Distance | Peg -> Hole entrance
    WEIGHT_DISTANCE_PEG_TO_HOLE_ENTRANCE = 16.0
    TANH_STD_DISTANCE_PEG_TO_HOLE_ENTRANCE = 0.025
    reward_distance_peg_to_hole_entrance = WEIGHT_DISTANCE_PEG_TO_HOLE_ENTRANCE * (
        1.0
        - torch.tanh(
            torch.min(torch.norm(hole_entrance_pos_wrt_peg_ends, dim=-1), dim=1)[0]
            / TANH_STD_DISTANCE_PEG_TO_HOLE_ENTRANCE
        )
    )

    # Reward: Distance | Peg -> Hole bottom
    WEIGHT_DISTANCE_PEG_TO_HOLE_BOTTOM = 128.0
    TANH_STD_DISTANCE_PEG_TO_HOLE_BOTTOM = 0.005
    reward_distance_peg_to_hole_bottom = WEIGHT_DISTANCE_PEG_TO_HOLE_BOTTOM * (
        1.0
        - torch.tanh(
            torch.min(torch.norm(hole_bottom_pos_wrt_peg_ends, dim=-1), dim=1)[0]
            / TANH_STD_DISTANCE_PEG_TO_HOLE_BOTTOM
        )
    )

    # Total reward
    rewards = torch.sum(
        torch.stack(
            [
                penalty_action_rate,
                penalty_undersired_robot_arm_contacts,
                reward_distance_ee_to_obj,
                reward_obj_grasped,
                reward_obj_lifted,
                reward_align_peg_to_hole_primary,
                reward_align_peg_to_hole_secondary,
                reward_distance_peg_to_hole_entrance,
                reward_distance_peg_to_hole_bottom,
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
        hole_entrance_pos_wrt_peg_ends,
        hole_bottom_pos_wrt_peg_ends,
        hole_rotmat_wrt_peg,
        rewards,
        terminations,
        truncations,
    )

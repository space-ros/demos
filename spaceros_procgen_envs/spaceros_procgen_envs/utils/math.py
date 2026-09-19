import math
from typing import Iterable, Tuple

import torch
from omni.isaac.lab.utils.math import *  # noqa: F403
from omni.isaac.lab.utils.math import matrix_from_quat, quat_apply, quat_inv, quat_mul


def quat_from_rpy(
    *args: Iterable[float], deg: bool = True
) -> Tuple[float, float, float, float]:
    """
    Returns wxyz quaternion from roll-pitch-yaw angles.
    """

    rpy = args[0] if len(args) == 1 else args

    roll, pitch, yaw = (r * (math.pi / 180.0) for r in rpy) if deg else rpy
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)

    qw = cy * cr * cp + sy * sr * sp
    qx = cy * sr * cp - sy * cr * sp
    qy = cy * cr * sp + sy * sr * cp
    qz = sy * cr * cp - cy * sr * sp

    return (qw, qx, qy, qz)


@torch.jit.script
def quat_to_rot6d(quaternions: torch.Tensor) -> torch.Tensor:
    return matrix_from_quat(quaternions)[..., :2].reshape(-1, 6)


@torch.jit.script
def rotmat_to_rot6d(rotmat: torch.Tensor) -> torch.Tensor:
    return rotmat[..., :2].reshape(-1, 6)


@torch.jit.script
def combine_frame_transforms(
    t01: torch.Tensor,
    q01: torch.Tensor,
    t12: torch.Tensor | None = None,
    q12: torch.Tensor | None = None,
) -> Tuple[torch.Tensor, torch.Tensor]:
    r"""Combine transformations between two reference frames into a stationary frame.

    It performs the following transformation operation: :math:`T_{02} = T_{01} \times T_{12}`,
    where :math:`T_{AB}` is the homogeneous transformation matrix from frame A to B.

    Args:
        t01: Position of frame 1 w.r.t. frame 0. Shape is (N, 3).
        q01: Quaternion orientation of frame 1 w.r.t. frame 0 in (w, x, y, z). Shape is (N, 4).
        t12: Position of frame 2 w.r.t. frame 1. Shape is (N, 3).
            Defaults to None, in which case the position is assumed to be zero.
        q12: Quaternion orientation of frame 2 w.r.t. frame 1 in (w, x, y, z). Shape is (N, 4).
            Defaults to None, in which case the orientation is assumed to be identity.

    Returns:
        A tuple containing the position and orientation of frame 2 w.r.t. frame 0.
        Shape of the tensors are (N, 3) and (N, 4) respectively.
    """
    # compute orientation
    q02 = quat_mul(q01, q12) if q12 is not None else q01
    # compute translation
    t02 = t01 + quat_apply(q01, t12) if t12 is not None else t01
    return t02, q02


@torch.jit.script
def subtract_frame_transforms(
    t01: torch.Tensor,
    q01: torch.Tensor,
    t02: torch.Tensor | None = None,
    q02: torch.Tensor | None = None,
) -> Tuple[torch.Tensor, torch.Tensor]:
    r"""Subtract transformations between two reference frames into a stationary frame.

    It performs the following transformation operation: :math:`T_{12} = T_{01}^{-1} \times T_{02}`,
    where :math:`T_{AB}` is the homogeneous transformation matrix from frame A to B.

    Args:
        t01: Position of frame 1 w.r.t. frame 0. Shape is (N, 3).
        q01: Quaternion orientation of frame 1 w.r.t. frame 0 in (w, x, y, z). Shape is (N, 4).
        t02: Position of frame 2 w.r.t. frame 0. Shape is (N, 3).
            Defaults to None, in which case the position is assumed to be zero.
        q02: Quaternion orientation of frame 2 w.r.t. frame 0 in (w, x, y, z). Shape is (N, 4).
            Defaults to None, in which case the orientation is assumed to be identity.

    Returns:
        A tuple containing the position and orientation of frame 2 w.r.t. frame 1.
        Shape of the tensors are (N, 3) and (N, 4) respectively.
    """
    # compute orientation
    q10 = quat_inv(q01)
    q12 = quat_mul(q10, q02) if q02 is not None else q10
    # compute translation
    t12 = quat_apply(q10, t02 - t01) if t02 is not None else quat_apply(q10, -t01)
    return t12, q12


@torch.jit.script
def transform_points(
    points: torch.Tensor,
    pos: torch.Tensor | None = None,
    quat: torch.Tensor | None = None,
) -> torch.Tensor:
    r"""Transform input points in a given frame to a target frame.

    This function transform points from a source frame to a target frame. The transformation is defined by the
    position :math:`t` and orientation :math:`R` of the target frame in the source frame.

    .. math::
        p_{target} = R_{target} \times p_{source} + t_{target}

    If the input `points` is a batch of points, the inputs `pos` and `quat` must be either a batch of
    positions and quaternions or a single position and quaternion. If the inputs `pos` and `quat` are
    a single position and quaternion, the same transformation is applied to all points in the batch.

    If either the inputs :attr:`pos` and :attr:`quat` are None, the corresponding transformation is not applied.

    Args:
        points: Points to transform. Shape is (N, P, 3) or (P, 3).
        pos: Position of the target frame. Shape is (N, 3) or (3,).
            Defaults to None, in which case the position is assumed to be zero.
        quat: Quaternion orientation of the target frame in (w, x, y, z). Shape is (N, 4) or (4,).
            Defaults to None, in which case the orientation is assumed to be identity.

    Returns:
        Transformed points in the target frame. Shape is (N, P, 3) or (P, 3).

    Raises:
        ValueError: If the inputs `points` is not of shape (N, P, 3) or (P, 3).
        ValueError: If the inputs `pos` is not of shape (N, 3) or (3,).
        ValueError: If the inputs `quat` is not of shape (N, 4) or (4,).
    """
    points_batch = points.clone()
    # check if inputs are batched
    is_batched = points_batch.dim() == 3
    # -- check inputs
    if points_batch.dim() == 2:
        points_batch = points_batch[None]  # (P, 3) -> (1, P, 3)
    if points_batch.dim() != 3:
        raise ValueError(
            f"Expected points to have dim = 2 or dim = 3: got shape {points.shape}"
        )
    if pos is not None and pos.dim() != 1 and pos.dim() != 2:
        raise ValueError(
            f"Expected pos to have dim = 1 or dim = 2: got shape {pos.shape}"
        )
    if quat is not None and quat.dim() != 1 and quat.dim() != 2:
        raise ValueError(
            f"Expected quat to have dim = 1 or dim = 2: got shape {quat.shape}"
        )
    # -- rotation
    if quat is not None:
        # convert to batched rotation matrix
        rot_mat = matrix_from_quat(quat)
        if rot_mat.dim() == 2:
            rot_mat = rot_mat[None]  # (3, 3) -> (1, 3, 3)
        # convert points to matching batch size (N, P, 3) -> (N, 3, P)
        # and apply rotation
        points_batch = torch.matmul(rot_mat, points_batch.transpose_(1, 2))
        # (N, 3, P) -> (N, P, 3)
        points_batch = points_batch.transpose_(1, 2)
    # -- translation
    if pos is not None:
        # convert to batched translation vector
        pos = pos[None, None, :] if pos.dim() == 1 else pos[:, None, :]
        # apply translation
        points_batch += pos
    # -- return points in same shape as input
    if not is_batched:
        points_batch = points_batch.squeeze(0)  # (1, P, 3) -> (P, 3)

    return points_batch

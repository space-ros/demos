from typing import Dict, Tuple

import torch
from omni.isaac.lab.sensors import Camera


def extract_images(camera: Camera) -> Dict[str, torch.Tensor]:
    output = camera.data.output
    return {
        "rgb": output["rgb"],
        "depth": output["distance_to_camera"],
    }


@torch.jit.script
def process_rgb(
    rgb: torch.Tensor,
    dtype: torch.dtype,
) -> torch.Tensor:
    return rgb[..., :3].to(dtype) / 255.0


@torch.jit.script
def process_depth(
    depth: torch.Tensor,
    depth_range: Tuple[float, float],
) -> torch.Tensor:
    return (
        depth.nan_to_num(
            nan=depth_range[1], posinf=depth_range[1], neginf=depth_range[1]
        ).clamp(depth_range[0], depth_range[1])
        - depth_range[0]
    ) / (depth_range[1] - depth_range[0])


@torch.jit.script
def construct_observation(
    *,
    rgb: torch.Tensor,
    depth: torch.Tensor,
    depth_range: Tuple[float, float],
    image_name: str,
) -> Dict[str, torch.Tensor]:
    return {
        f"{image_name}_rgb": process_rgb(rgb, depth.dtype),
        f"{image_name}_depth": process_depth(depth, depth_range).unsqueeze(-1),
    }

import hashlib
import json
import math
import subprocess
import sys
from os import path
from typing import TYPE_CHECKING, Any, Dict, Iterable, Tuple

from pxr import Usd

import spaceros_procgen_envs.core.sim as sim_utils
from spaceros_procgen_envs.core.sim.spawners.multi import MultiAssetCfg

if TYPE_CHECKING:
    from . import cfg


def spawn_blender_procgen_assets(
    prim_path: str,
    cfg: "cfg.ProceduralAssetCfg",
    translation: Tuple[float, float, float] | None = None,
    orientation: Tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    # Scale the texture resolution by its factor
    texture_resolution = max(16, math.ceil(cfg.detail * cfg.texture_resolution))
    if texture_resolution % 2 != 0:
        texture_resolution += 1

    # Extract configuration into script kwargs
    script_kwargs = {
        "autorun_scripts": cfg.autorun_scripts,
        "name": cfg.name,
        "ext": cfg.ext,
        "overwrite_min_age": cfg.overwrite_min_age,
        "seed": cfg.seed,
        "num_assets": cfg.num_assets,
        "export_kwargs": cfg.export_kwargs,
        "geometry_nodes": cfg.geometry_nodes,
        "decimate_angle_limit": cfg.decimate_angle_limit,
        "decimate_face_count": cfg.decimate_face_count,
        "material": cfg.material,
        "texture_resolution": texture_resolution,
    }

    # Derive the output directory based on the configuration
    outdir = path.join(
        cfg.cache_dir,
        cfg.name,
        _hash_filtered_dict(
            script_kwargs,
            filter_keys=[
                "autorun_scripts",
                "name",
                "ext",
                "overwrite_min_age",
                "seed",
                "num_assets",
            ],
        ),
    )
    script_kwargs["outdir"] = outdir
    print(f"[TRACE] Caching procedural assets to '{outdir}'")

    # Convert kwargs to args
    script_args = []
    for key, value in script_kwargs.items():
        if value:
            if value is not False:
                script_args.append(f"--{key}")

            if isinstance(value, str):
                script_args.append(value)
            elif isinstance(value, Dict):
                script_args.append(json.dumps(value))
            elif isinstance(value, Iterable):
                script_args.extend(str(v) for v in value)
            elif value is None or value is True:
                continue
            else:
                script_args.append(str(value))

    # Run the Blender script
    print(
        f"[INFO] Generating {cfg.num_assets} procedural '{cfg.name}' asset(s) for '{prim_path}' (this might take a while)"
    )
    cmd = [
        cfg.blender_bin,
        *cfg.blender_args,
        "--python",
        cfg.bpy_script,
        "--",
        *script_args,
    ]
    print(f"[TRACE] Call: {' '.join(cmd)}")
    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(result.stdout, file=sys.stdout)
        print(result.stderr, file=sys.stderr)
        raise RuntimeError(
            f"Failed to generate procedural assets using Blender: {' '.join(result.args)}"
        )

    # Leverage MultiAssetCfg to spawn the generated assets
    multi_asset_cfg = MultiAssetCfg(
        assets_cfg=[
            sim_utils.UsdFileCfg(
                usd_path=path.join(
                    outdir,
                    asset_basename,
                ),
                **{
                    k: v
                    for k, v in cfg.usd_file_cfg.__dict__.items()
                    if not k.startswith("_") and k not in ["func", "usd_path"]
                },
            )
            for asset_basename in [
                f"{cfg.name}{current_seed}{cfg.ext}"
                for current_seed in range(cfg.seed, cfg.seed + cfg.num_assets)
            ]
        ]
    )
    return multi_asset_cfg.func(
        prim_path=prim_path,
        cfg=multi_asset_cfg,
        translation=translation,
        orientation=orientation,
    )


def _hash_filtered_dict(input: Dict[str, Any], filter_keys: Iterable[str] = []) -> str:
    dhash = hashlib.md5()
    filtered = {k: v for k, v in input.items() if k not in filter_keys}
    encoded = json.dumps(filtered, sort_keys=True).encode()
    dhash.update(encoded)
    return dhash.hexdigest()

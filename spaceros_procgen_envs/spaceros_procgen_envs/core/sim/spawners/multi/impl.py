import re
from typing import TYPE_CHECKING, Tuple

import omni.isaac.core.utils.prims as prim_utils
import omni.usd
from pxr import Gf, Sdf, Semantics, Usd, UsdGeom, Vt

import spaceros_procgen_envs.core.sim as sim_utils

if TYPE_CHECKING:
    from . import cfg


def spawn_multi_asset_sequential(
    prim_path: str,
    cfg: "cfg.MultiAssetCfg",
    translation: Tuple[float, float, float] | None = None,
    orientation: Tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    # Check if prim path contains a regex expression
    # Note: A valid prim path can only contain alphanumeric characters, underscores, and forward slashes
    root_path, asset_path = prim_path.rsplit("/", 1)
    is_regex_expression = re.match(r"^[a-zA-Z0-9/_]+$", root_path) is None

    # Resolve matching prims for source prim path expression
    if is_regex_expression and root_path != "":
        source_prim_paths = sim_utils.find_matching_prim_paths(root_path)
        if len(source_prim_paths) == 0:
            raise RuntimeError(
                f"Unable to find source prim path: '{root_path}'. Please create the prim before spawning."
            )
    else:
        source_prim_paths = [root_path]

    # Spawn everything first in a dataset prim
    proto_root_prim_path = f"/World/db_{asset_path}"
    prim_utils.create_prim(proto_root_prim_path, "Scope")

    proto_prim_paths = []
    n_assets = len(cfg.assets_cfg)
    zfill_len = len(str(n_assets))
    for index, asset_cfg in enumerate(cfg.assets_cfg):
        # Save the proto prim path
        proto_prim_path = (
            f"{proto_root_prim_path}/{asset_path}_{str(index).zfill(zfill_len)}"
        )
        proto_prim_paths.append(proto_prim_path)

        # Spawn single instance
        prim = asset_cfg.func(proto_prim_path, asset_cfg)

        # Set the prim visibility
        if hasattr(asset_cfg, "visible"):
            imageable = UsdGeom.Imageable(prim)
            if asset_cfg.visible:
                imageable.MakeVisible()
            else:
                imageable.MakeInvisible()

        # Set the semantic annotations
        if hasattr(asset_cfg, "semantic_tags") and asset_cfg.semantic_tags is not None:
            # Note: Taken from replicator scripts.utils.utils.py
            for semantic_type, semantic_value in asset_cfg.semantic_tags:
                # Sanitize by replacing spaces with underscores
                semantic_type_sanitized = semantic_type.replace(" ", "_")
                semantic_value_sanitized = semantic_value.replace(" ", "_")
                # Set the semantic API for the instance
                instance_name = f"{semantic_type_sanitized}_{semantic_value_sanitized}"
                sem = Semantics.SemanticsAPI.Apply(prim, instance_name)
                # Create semantic type and data attributes
                sem.CreateSemanticTypeAttr()
                sem.CreateSemanticDataAttr()
                sem.GetSemanticTypeAttr().Set(semantic_type)
                sem.GetSemanticDataAttr().Set(semantic_value)

        # Activate rigid body contact sensors
        if (
            hasattr(asset_cfg, "activate_contact_sensors")
            and asset_cfg.activate_contact_sensors
        ):
            sim_utils.activate_contact_sensors(
                proto_prim_path, asset_cfg.activate_contact_sensors
            )

    # Acquire stage
    stage = omni.usd.get_context().get_stage()

    # Resolve prim paths for spawning and cloning
    prim_paths = [
        f"{source_prim_path}/{asset_path}" for source_prim_path in source_prim_paths
    ]

    # Convert orientation ordering (wxyz to xyzw)
    orientation = (orientation[1], orientation[2], orientation[3], orientation[0])

    # manually clone prims if the source prim path is a regex expression
    with Sdf.ChangeBlock():
        for i, prim_path in enumerate(prim_paths):
            # Spawn single instance
            env_spec = Sdf.CreatePrimInLayer(stage.GetRootLayer(), prim_path)

            # Select assets in order for uniform distribution
            proto_path = proto_prim_paths[i % n_assets]

            # Copy spec from the proto prim
            Sdf.CopySpec(
                env_spec.layer,
                Sdf.Path(proto_path),
                env_spec.layer,
                Sdf.Path(prim_path),
            )

            ## Set the XformOp for the prim
            _ = UsdGeom.Xform(stage.GetPrimAtPath(proto_path)).GetPrim().GetPrimStack()
            # Set the order
            op_order_spec = env_spec.GetAttributeAtPath(f"{prim_path}.xformOpOrder")
            if op_order_spec is None:
                op_order_spec = Sdf.AttributeSpec(
                    env_spec, UsdGeom.Tokens.xformOpOrder, Sdf.ValueTypeNames.TokenArray
                )
            op_order_spec.default = Vt.TokenArray(
                ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
            )
            # Translation
            translate_spec = env_spec.GetAttributeAtPath(
                f"{prim_path}.xformOp:translate"
            )
            if translate_spec is None:
                translate_spec = Sdf.AttributeSpec(
                    env_spec, "xformOp:translate", Sdf.ValueTypeNames.Double3
                )
            translate_spec.default = Gf.Vec3d(*translation)
            # Orientation
            orient_spec = env_spec.GetAttributeAtPath(f"{prim_path}.xformOp:orient")
            if orient_spec is None:
                orient_spec = Sdf.AttributeSpec(
                    env_spec, "xformOp:orient", Sdf.ValueTypeNames.Quatd
                )
            orient_spec.default = Gf.Quatd(*orientation)
            # Scale
            scale_spec = env_spec.GetAttributeAtPath(f"{prim_path}.xformOp:scale")
            if scale_spec is None:
                scale_spec = Sdf.AttributeSpec(
                    env_spec, "xformOp:scale", Sdf.ValueTypeNames.Double3
                )
            scale_spec.default = Gf.Vec3d(1.0, 1.0, 1.0)

    # Delete the dataset prim after spawning
    prim_utils.delete_prim(proto_root_prim_path)

    # Return the prim
    return prim_utils.get_prim_at_path(prim_paths[0])

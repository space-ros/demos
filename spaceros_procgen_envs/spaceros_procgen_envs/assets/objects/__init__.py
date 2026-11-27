from typing import Any, Dict, Optional, Tuple

import spaceros_procgen_envs.core.envs as env_utils
import spaceros_procgen_envs.core.sim as sim_utils
from spaceros_procgen_envs.core.assets import AssetBaseCfg, RigidObjectCfg
from spaceros_procgen_envs.utils import color as color_utils

from .lunar_rock_procgen import LunarRockProcgenCfg
from .martian_rock_procgen import MartianRockProcgenCfg
from .peg_in_hole_profile import ProfileHoleCfg, ProfilePegCfg
from .sample_tube import SampleTubeCfg


@staticmethod
def object_of_interest_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    size: Tuple[float, float] = (0.075, 0.075, 0.06),
    num_assets: int = 1,
    prim_path: str = "{ENV_REGEX_NS}/object",
    spawn_kwargs: Dict[str, Any] = {},
    procgen_kwargs: Dict[str, Any] = {},
    **kwargs,
) -> Optional[RigidObjectCfg]:
    spawn: Optional[sim_utils.SpawnerCfg] = None

    if spawn_kwargs.get("collision_props") is None:
        spawn_kwargs["collision_props"] = sim_utils.CollisionPropertiesCfg()
    if spawn_kwargs.get("rigid_props") is None:
        spawn_kwargs["rigid_props"] = sim_utils.RigidBodyPropertiesCfg(
            max_depenetration_velocity=0.25,
        )
    if spawn_kwargs.get("mass_props") is None:
        spawn_kwargs["mass_props"] = sim_utils.MassPropertiesCfg(density=2500.0)
    if spawn_kwargs.get(
        "mesh_collision_props"
    ) is None and env_cfg.assets.object.variant in (
        env_utils.AssetVariant.DATASET,
        env_utils.AssetVariant.PROCEDURAL,
    ):
        spawn_kwargs["mesh_collision_props"] = sim_utils.MeshCollisionPropertiesCfg(
            mesh_approximation="sdf",
            sdf_resolution=360,
        )

    match env_cfg.assets.object.variant:
        case env_utils.AssetVariant.NONE:
            return None

        case env_utils.AssetVariant.PRIMITIVE:
            if (
                spawn_kwargs.get("visual_material") is None
                and env_utils.AssetVariant.PRIMITIVE == env_cfg.assets.object.variant
            ):
                spawn_kwargs["visual_material"] = sim_utils.PreviewSurfaceCfg(
                    diffuse_color=color_utils.contrastive_color_from_env_cfg(env_cfg),
                )
            spawn = sim_utils.MultiShapeCfg(
                size=size,
                shape_cfg=sim_utils.ShapeCfg(**spawn_kwargs),
            )

        case env_utils.AssetVariant.DATASET:
            match env_cfg.scenario:
                case env_utils.Scenario.MARS:
                    spawn = SampleTubeCfg(**spawn_kwargs)
                case _:
                    if (
                        spawn_kwargs.get("visual_material") is None
                        and env_utils.AssetVariant.PRIMITIVE
                        == env_cfg.assets.object.variant
                    ):
                        spawn_kwargs["visual_material"] = sim_utils.PreviewSurfaceCfg(
                            diffuse_color=color_utils.contrastive_color_from_env_cfg(
                                env_cfg
                            ),
                        )
                    spawn = ProfilePegCfg(**spawn_kwargs)

        case env_utils.AssetVariant.PROCEDURAL:
            usd_file_cfg = sim_utils.UsdFileCfg(
                usd_path="IGNORED",
                **spawn_kwargs,
            )

            match env_cfg.scenario:
                case env_utils.Scenario.MOON | env_utils.Scenario.ORBIT:
                    spawn = LunarRockProcgenCfg(
                        num_assets=num_assets,
                        usd_file_cfg=usd_file_cfg,
                        seed=env_cfg.seed,
                        detail=env_cfg.detail,
                    )

                case env_utils.Scenario.MARS:
                    spawn = MartianRockProcgenCfg(
                        num_assets=num_assets,
                        usd_file_cfg=usd_file_cfg,
                        seed=env_cfg.seed,
                        detail=env_cfg.detail,
                    )
                case _:
                    return None

            scale = size
            for node_cfg in spawn.geometry_nodes.values():
                if node_cfg.get("scale") is not None:
                    node_cfg["scale"] = scale

            for key, value in procgen_kwargs.items():
                for node_cfg in spawn.geometry_nodes.values():
                    if node_cfg.get(key) is not None:
                        node_cfg[key] = value
                    elif hasattr(spawn, key):
                        setattr(spawn, key, value)

    if spawn is None:
        raise NotImplementedError
    return RigidObjectCfg(
        prim_path=prim_path,
        spawn=spawn,
        **kwargs,
    )


@staticmethod
def peg_in_hole_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    size: Tuple[float, float] = (0.1, 0.1, 0.1),
    num_assets: int = 1,
    prim_path_peg: str = "{ENV_REGEX_NS}/peg",
    prim_path_hole: str = "{ENV_REGEX_NS}/hole",
    spawn_kwargs_peg: Dict[str, Any] = {},
    spawn_kwargs_hole: Dict[str, Any] = {},
    procgen_kwargs_peg: Dict[str, Any] = {},
    procgen_kwargs_hole: Dict[str, Any] = {},
    **kwargs,
) -> Optional[Tuple[RigidObjectCfg, AssetBaseCfg]]:
    spawn_peg: Optional[sim_utils.SpawnerCfg] = None
    spawn_hole: Optional[sim_utils.SpawnerCfg] = None

    if spawn_kwargs_peg.get("collision_props") is None:
        spawn_kwargs_peg["collision_props"] = sim_utils.CollisionPropertiesCfg()
    if spawn_kwargs_peg.get("rigid_props") is None:
        spawn_kwargs_peg["rigid_props"] = sim_utils.RigidBodyPropertiesCfg()
    if spawn_kwargs_peg.get("mass_props") is None:
        spawn_kwargs_peg["mass_props"] = sim_utils.MassPropertiesCfg(density=2500.0)
    if spawn_kwargs_peg.get("mesh_collision_props") is None:
        spawn_kwargs_peg["mesh_collision_props"] = sim_utils.MeshCollisionPropertiesCfg(
            mesh_approximation="boundingCube",
        )
    if spawn_kwargs_peg.get("visual_material") is None:
        spawn_kwargs_peg["visual_material"] = sim_utils.PreviewSurfaceCfg(
            diffuse_color=color_utils.contrastive_color_from_env_cfg(env_cfg),
        )

    if spawn_kwargs_hole.get("collision_props") is None:
        spawn_kwargs_hole["collision_props"] = sim_utils.CollisionPropertiesCfg()
    if spawn_kwargs_hole.get("visual_material") is None:
        spawn_kwargs_hole["visual_material"] = sim_utils.PreviewSurfaceCfg(
            diffuse_color=color_utils.contrastive_color_from_env_cfg(env_cfg),
        )

    spawn_peg = ProfilePegCfg(**spawn_kwargs_peg)
    spawn_hole = ProfileHoleCfg(**spawn_kwargs_hole)

    return RigidObjectCfg(
        prim_path=prim_path_peg,
        spawn=spawn_peg,
        **kwargs,
    ), AssetBaseCfg(
        prim_path=prim_path_hole,
        spawn=spawn_hole,
        **kwargs,
    )

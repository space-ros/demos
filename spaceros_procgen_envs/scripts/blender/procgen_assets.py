#!/usr/bin/env -S blender --factory-startup --background --offline-mode --enable-autoexec --python-exit-code 1 --python
"""
Script for automated procedural asset generation using Blender that revolves around its rich
node-based system for geometry (Geometry Nodes) and materials (Shader Nodes).

Overview:
    The requested node trees are constructed via scripts defined by `--autorun_scripts`. A sequence of
    Geometry Nodes moodifiers is then applied to a prototype object that is duplicated for each generated
    variant. Once the geometry is finalized, a procedural material is applied and baked into PBR textures
    before exporting the final model.

Example (manual invocation is not recommended):
    blender --python procgen_assets.py -- \
        --autorun_scripts path/to/nodes_0.py path/to/nodes_1.py ... \
        --geometry_nodes '{"NodeModifierName": {"input_name": input_value, ...}, ...}' \
        --material MaterialName \
        --texture_resolution 4096 \
        --num_assets 10 \
        --outdir path/to/output/directory
"""

from __future__ import annotations

import argparse
import contextlib
import enum
import io
import json
import os
import re
import sys
import time
from copy import deepcopy
from os import path
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, TextIO, Tuple, Union

import bpy


def main(**kwargs):
    print_bpy(f"[INFO]: Starting procedural asset generation with kwargs: {kwargs}")
    verify_requirements()
    ProceduralGenerator.generate(**kwargs)


class ProceduralGenerator:
    """
    Generator of procedural models using Blender.
    """

    @classmethod
    def generate(
        cls,
        *,
        ## Input
        autorun_scripts: List[str],
        ## Output
        outdir: str,
        name: str,
        ext: str,
        overwrite_min_age: int,
        ## Generator
        seed: int,
        num_assets: int,
        ## Export
        export_kwargs: Dict[str, Any],
        ## Geometry
        geometry_nodes: Dict[str, Dict[str, Any]],
        decimate_angle_limit: Optional[float],
        decimate_face_count: Optional[int],
        ## Material
        material: Optional[str],
        texture_resolution: int,
    ):
        """
        Entrypoint method for generating a set of procedural models.
        """

        # Preprocess the input arguments
        outdir = Path(outdir).absolute().as_posix()
        if not ext.startswith("."):
            ext = f".{ext}"

        # Create the output directory
        os.makedirs(name=outdir, exist_ok=True)

        # Reset to factory settings with an empty scene
        bpy.ops.wm.read_factory_settings(use_empty=True)

        # Autorun all input scripts
        for script in autorun_scripts:
            print_bpy(f"[INFO]: Running script: {script}")
            bpy.ops.script.python_file_run(filepath=script)

        # Extract a map of the input socket mapping for all loaded node groups
        node_group_input_socket_maps = cls.extract_aliased_input_socket_id_mappings()

        # Create an empty mesh object and treat it as a prototype that will be duplicated for each processed seed
        bpy.ops.object.add(type="MESH")
        proto_obj: bpy.types.Object = bpy.context.active_object
        proto_obj.name = name
        proto_obj.data.name = name

        # Apply the requested list of geometry nodes to the prototype object
        cls.apply_geometry_nodes_modifiers(
            obj=proto_obj,
            geometry_nodes=geometry_nodes,
            node_group_input_socket_maps=node_group_input_socket_maps,
        )

        # Preheat the oven for baking the material into PBR textures
        if material:
            ProceduralGenerator.Baker.preheat_oven()

        # Generate models over the specified range
        for current_seed in range(seed, seed + num_assets):
            # Form the output filepath
            filepath = path.join(outdir, f"{name}{current_seed}{ext}")

            # Skip generation if the file already exists and is too recent
            if path.exists(filepath) and (
                overwrite_min_age < 0
                or (overwrite_min_age > time.time() - path.getmtime(filepath))
            ):
                print_bpy(
                    f"[INFO]: Skipping generation of '{filepath}' because it was generated in the last {overwrite_min_age} seconds"
                )
                continue

            # Duplicate the prototype object, rename it, select it and mark it as the active object
            obj = cls.duplicate_object(obj=proto_obj, seed=current_seed)

            # Update the random seed of all geometry nodes modifiers that have a seed input socket
            geometry_nodes_modifiers = [
                modifier
                for modifier in obj.modifiers.values()
                if modifier.type == "NODES"
            ]
            for modifier in geometry_nodes_modifiers:
                if seed_id := node_group_input_socket_maps[
                    modifier.node_group.name
                ].get("seed"):
                    modifier[seed_id] = current_seed

            # Apply changes via mesh update
            obj.data.update()

            # Apply all modifiers
            for modifier in obj.modifiers:
                bpy.ops.object.modifier_apply(modifier=modifier.name)

            # If specified, bake the material into PBR textures
            if material:
                obj.data.materials.clear()
                obj.data.materials.append(bpy.data.materials.get(material))
                ProceduralGenerator.Baker.bake_into_pbr_material(
                    obj=obj, texture_resolution=texture_resolution
                )

            # Decimate the mesh if necessary
            if decimate_angle_limit:
                bpy.ops.object.modifier_add(type="DECIMATE")
                obj.modifiers["Decimate"].decimate_type = "DISSOLVE"
                obj.modifiers["Decimate"].angle_limit = decimate_angle_limit

            if decimate_face_count:
                # Decimate the mesh
                bpy.ops.object.modifier_add(type="DECIMATE")
                obj.modifiers["Decimate"].ratio = decimate_face_count / len(
                    obj.data.polygons
                )
                bpy.ops.object.modifier_apply(modifier="Decimate")

            # Export the model
            cls.Exporter.usd_export(
                filepath=filepath, ext=ext, makedirs=False, **export_kwargs
            )
            print_bpy(
                f"[LOG]: Generated asset #{current_seed} ({current_seed-seed+1}/{num_assets}): {filepath}"
            )

            # Update the viewport to keep track of progress
            if not bpy.app.background:
                bpy.ops.wm.redraw_timer(type="DRAW_WIN_SWAP", iterations=1)

            # Remove the generated object
            bpy.data.objects.remove(obj)

        print_bpy("[INFO]: Generation completed")

    @staticmethod
    def extract_aliased_input_socket_id_mappings() -> Dict[str, Dict[str, str]]:
        def _extract_input_socket_id_mapping(
            node_group: bpy.types.NodeTree,
        ) -> Dict[str, str]:
            return {
                canonicalize_str(item.name): item.identifier
                for item in node_group.interface.items_tree.values()
                if item.item_type == "SOCKET" and item.in_out == "INPUT"
            }

        # Extract a map of the input socket mapping for all node groups
        node_group_input_socket_maps = {
            node_group_name: _extract_input_socket_id_mapping(node_group)
            for node_group_name, node_group in bpy.data.node_groups.items()
        }

        # Rename common aliases for convenience
        COMMON_ALIASES: Dict[str, List[str]] = {
            "seed": [
                "pseodorandomseed",
                "randomseed",
                "rng",
            ],
            "detail": [
                "detaillevel",
                "detailobject",
                "levelofdetail",
                "subdivisionlevel",
                "subdivisions",
                "subdivlevel",
            ],
        }
        for node_group_input_socket_map in node_group_input_socket_maps.values():
            for target, possible_alias in COMMON_ALIASES.items():
                original_alias: Optional[str] = (
                    target if target in node_group_input_socket_map.keys() else None
                )
                for key in node_group_input_socket_map.keys():
                    if key in possible_alias:
                        if original_alias is not None:
                            raise ValueError(
                                "Ambiguous name of the input socket '{target}' (canonicalized): '{original_alias}', '{key}'"
                            )
                        original_alias = key
                if original_alias is not None and original_alias != target:
                    node_group_input_socket_map[target] = node_group_input_socket_map[
                        original_alias
                    ]

        return node_group_input_socket_maps

    @staticmethod
    def apply_geometry_nodes_modifiers(
        obj: bpy.types.Object,
        *,
        geometry_nodes: Dict[str, Dict[str, Any]],
        node_group_input_socket_maps: Dict[str, Dict[str, str]],
    ):
        for i, (node_group_name, node_group_inputs) in enumerate(
            geometry_nodes.items()
        ):
            # Create a new nodes modifier
            modifier: bpy.types.NodesModifier = obj.modifiers.new(
                name=f"node{i}", type="NODES"
            )

            # Assign the requested node group
            if node_group := bpy.data.node_groups.get(node_group_name):
                modifier.node_group = node_group
            else:
                raise ValueError(
                    f"Node group '{node_group_name}' not found in the list of available groups: {bpy.data.node_groups.keys()}"
                )

            # Set inputs accordingly
            for key, value in node_group_inputs.items():
                socket_id = node_group_input_socket_maps[node_group_name][
                    canonicalize_str(key)
                ]
                modifier[socket_id] = value

        # Apply changes via mesh update
        obj.data.update()

    @staticmethod
    def duplicate_object(obj: bpy.types.Object, seed: int) -> bpy.types.Object:
        # Select the object to duplicate
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj

        # Duplicate the object
        bpy.ops.object.duplicate()

        # Get the new duplicated object (it will be the active object)
        new_obj = bpy.context.active_object

        # Rename the new object based on the seed
        new_obj.name = f"{obj.name}_{seed}"
        new_obj.data.name = f"{obj.data.name}_{seed}"

        # Select the new object and make it the active object
        bpy.ops.object.select_all(action="DESELECT")
        new_obj.select_set(True)
        bpy.context.view_layer.objects.active = new_obj

        return new_obj

    class Recipe(enum.Enum):
        ALBEDO = enum.auto()
        METALLIC = enum.auto()
        SPECULAR = enum.auto()
        ROUGHNESS = enum.auto()
        NORMAL = enum.auto()

        @staticmethod
        def enabled_recipes():
            return (
                ProceduralGenerator.Recipe.ALBEDO,
                ProceduralGenerator.Recipe.METALLIC,
                # ProceduralGenerator.Recipe.SPECULAR,
                ProceduralGenerator.Recipe.ROUGHNESS,
                ProceduralGenerator.Recipe.NORMAL,
            )

        @property
        def bake_type(self):
            match self:
                case self.ALBEDO:
                    return "DIFFUSE"
                case self.METALLIC:
                    return "EMIT"
                case self.SPECULAR:
                    return "GLOSSY"
                case _:
                    return self.name

        @property
        def color_space(self):
            return "sRGB" if self.ALBEDO == self else "Non-Color"

        @property
        def shader_socket_name(self):
            match self:
                case self.ALBEDO:
                    return "Base Color"
                case self.METALLIC:
                    return "Metallic"
                case self.SPECULAR:
                    return "Specular Tint"
                case self.ROUGHNESS:
                    return "Roughness"
                case self.NORMAL:
                    return "Normal"

        def prep(self, material: bpy.types.Material) -> Dict[str, Any]:
            match self:
                # If the shader node has a "Metallic" input, it needs to be disabled for baking
                case self.ALBEDO:
                    # Get the output material node
                    output_material_node = [
                        node
                        for node in material.node_tree.nodes
                        if node.type == "OUTPUT_MATERIAL"
                    ][0]

                    # Get surface shader socket
                    shader_node_socket = output_material_node.inputs["Surface"]
                    if not shader_node_socket.is_linked:
                        return {}

                    # Get the shader node connected to the socket
                    shader_node = shader_node_socket.links[0].from_node

                    # No need to do anything if the shader node does not have a "Metallic" input
                    if "Metallic" not in shader_node.inputs:
                        return {}

                    metallic_socket = shader_node.inputs["Metallic"]
                    ingredients = {}

                    # If the metallic input is linked, store the original link source and temporarily disconnect it
                    if metallic_socket.is_linked:
                        metallic_socket_links = metallic_socket.links[0]
                        ingredients["orig_metallic_from_socket"] = (
                            metallic_socket_links.from_socket
                        )
                        material.node_tree.links.remove(metallic_socket_links)

                    # Always store the original default value and set it to 0.0
                    ingredients["orig_metallic_default_value"] = (
                        metallic_socket.default_value
                    )
                    metallic_socket.default_value = 0.0

                    return ingredients

                # Render the metallic input as emission originating from the surface shader's metallic input
                case self.METALLIC:
                    # Get the output material node
                    output_material_node = [
                        node
                        for node in material.node_tree.nodes
                        if node.type == "OUTPUT_MATERIAL"
                    ][0]

                    # Get surface shader socket
                    shader_node_socket = output_material_node.inputs["Surface"]
                    if not shader_node_socket.is_linked:
                        return {}

                    # Get the shader link
                    shader_link = shader_node_socket.links[0]

                    # Get the shader node connected to the socket
                    shader_node = shader_link.from_node

                    # Store the original link source and temporarily disconnect it
                    ingredients = {
                        "orig_surface_shader_source": shader_link.from_socket
                    }
                    material.node_tree.links.remove(shader_link)

                    # No need to do anything if the shader node does not have a "Metallic" input
                    orig_metallic_default_value = 0.0
                    with_emissive_rgb_node = False
                    if "Metallic" in shader_node.inputs:
                        metallic_socket = shader_node.inputs["Metallic"]

                        # If the metallic input is linked, store the original link source and temporarily disconnect it
                        if metallic_socket.is_linked:
                            metallic_socket_links = metallic_socket.links[0]
                            material.node_tree.links.new(
                                metallic_socket_links.from_socket,
                                shader_node_socket,
                            )
                        else:
                            with_emissive_rgb_node = True
                            orig_metallic_default_value = metallic_socket.default_value
                    else:
                        with_emissive_rgb_node = True

                    if with_emissive_rgb_node:
                        rgb_node = material.node_tree.nodes.new(type="ShaderNodeRGB")
                        rgb_node.outputs[0].default_value = (
                            *((orig_metallic_default_value,) * 3),
                            1.0,
                        )
                        material.node_tree.links.new(
                            rgb_node.outputs["Color"], shader_node_socket
                        )
                        ingredients["emissive_rgb_node"] = rgb_node

                    return ingredients

                case _:
                    return {}

        def cleanup(
            self,
            material: bpy.types.Material,
            *,
            orig_metallic_from_socket: Optional[bpy.types.NodeSocket] = None,
            orig_metallic_default_value: Optional[float] = None,
            orig_surface_shader_source: Optional[bpy.types.NodeSocket] = None,
            emissive_rgb_node: Optional[bpy.types.Node] = None,
        ):
            match self:
                case self.ALBEDO:
                    if orig_metallic_default_value or orig_metallic_from_socket:
                        metallic_socket = (
                            [
                                node
                                for node in material.node_tree.nodes
                                if node.type == "OUTPUT_MATERIAL"
                            ][0]
                            .inputs["Surface"]
                            .links[0]
                            .inputs["Metallic"]
                        )
                        metallic_socket.default_value = orig_metallic_default_value
                    if orig_metallic_from_socket:
                        material.node_tree.links.new(
                            orig_metallic_from_socket,
                            metallic_socket,
                        )

                case self.METALLIC:
                    if orig_surface_shader_source:
                        shader_node_socket = [
                            node
                            for node in material.node_tree.nodes
                            if node.type == "OUTPUT_MATERIAL"
                        ][0].inputs["Surface"]
                        shader_link = shader_node_socket.links[0]
                        material.node_tree.links.remove(shader_link)
                        material.node_tree.links.new(
                            orig_surface_shader_source,
                            shader_node_socket,
                        )
                    if emissive_rgb_node:
                        material.node_tree.nodes.remove(emissive_rgb_node)

    class Baker:
        """
        Simple wrapper around Blender baking capabilities.
        """

        @staticmethod
        def preheat_oven():
            # Only Cycles supports texture baking
            bpy.context.scene.render.engine = "CYCLES"
            bpy.data.scenes[0].render.engine = "CYCLES"

            # Bake using GPU
            bpy.context.preferences.addons["cycles"].preferences.get_devices()
            for device in bpy.context.preferences.addons["cycles"].preferences.devices:
                device.use = device.type == "CUDA"
            bpy.context.preferences.addons["cycles"].preferences.compute_device_type = (
                "CUDA"
            )
            bpy.context.scene.cycles.device = "GPU"

            # Improve performance
            bpy.context.scene.cycles.samples = 1
            bpy.context.scene.cycles.use_auto_tile = False

            # Consider only the color pass (no environment lighting)
            bpy.context.scene.render.bake.use_pass_direct = False
            bpy.context.scene.render.bake.use_pass_indirect = False
            bpy.context.scene.render.bake.use_pass_color = True

        @classmethod
        def bake_into_pbr_material(
            cls,
            obj: bpy.types.Object,
            *,
            texture_resolution: int,
        ):
            # Adjust the recipe according to the object
            bpy.context.scene.render.bake.margin = texture_resolution // 64

            # Unwrap the object if necessary
            if not obj.data.uv_layers:
                cls._unwrap_uv_on_active_obj(obj, texture_resolution)

            # Get the material
            material = obj.data.materials[0]

            # Bake all textures from the recipe
            baked_textures = {}
            for recipe in ProceduralGenerator.Recipe.enabled_recipes():
                ingredients = recipe.prep(material=material)

                # Create the image node into which the texture will be baked
                image_node = cls._create_image_node(
                    material=material,
                    recipe=recipe,
                    texture_resolution=texture_resolution,
                )

                # Bake the texture
                bpy.ops.object.bake(type=recipe.bake_type)
                baked_textures[recipe] = image_node.image

                # Remove the image node and cleanup the material
                material.node_tree.nodes.remove(image_node)
                recipe.cleanup(material=material, **ingredients)

            # Create a new PBR material with the baked textures
            pbr_material = cls._bake_into_pbr_material(
                name=f"PBR_{obj.name}", baked_textures=baked_textures
            )

            # Replace the original material with the new PBR material
            obj.data.materials.clear()
            obj.data.materials.append(pbr_material)

        @classmethod
        def _bake_into_pbr_material(
            cls,
            name: str,
            baked_textures: Dict[ProceduralGenerator.Recipe, bpy.types.Image],
        ) -> bpy.types.Material:
            # Create a new material
            pbr_material = bpy.data.materials.new(name=name)
            pbr_material.use_nodes = True

            # Get handles to the nodes and links
            nodes = pbr_material.node_tree.nodes
            links = pbr_material.node_tree.links

            # Clear the existing nodes
            nodes.clear()

            # Create Material Output and Principled BSDF nodes
            shader_node = nodes.new(type="ShaderNodeBsdfPrincipled")
            shader_node.location = (-300, 0)
            output_node = nodes.new(type="ShaderNodeOutputMaterial")
            output_node.location = (0, 0)
            links.new(shader_node.outputs["BSDF"], output_node.inputs["Surface"])

            # Create Texture Coordinate and Mapping nodes
            texcoord_node = nodes.new(type="ShaderNodeTexCoord")
            texcoord_node.location = (-1200, 0)
            mapping_node = nodes.new(type="ShaderNodeMapping")
            mapping_node.location = (-1000, 0)
            links.new(mapping_node.inputs["Vector"], texcoord_node.outputs["UV"])

            # Create baked textures
            for i, (recipe, texture) in enumerate(baked_textures.items()):
                # Create Image Texture node
                img_texture = nodes.new(type="ShaderNodeTexImage")
                img_texture.image = texture
                img_texture.location = (
                    -800,
                    375 * (0.5 * len(baked_textures) + 0.5 - i),
                )
                links.new(img_texture.inputs["Vector"], mapping_node.outputs["Vector"])

                match recipe:
                    # Normal map requires the Normal Map node
                    case ProceduralGenerator.Recipe.NORMAL:
                        normal_map_node = nodes.new(type="ShaderNodeNormalMap")
                        normal_map_node.location = (-500, -127)
                        links.new(
                            img_texture.outputs["Color"],
                            normal_map_node.inputs["Color"],
                        )
                        links.new(
                            normal_map_node.outputs["Normal"],
                            shader_node.inputs[recipe.shader_socket_name],
                        )

                    # Other textures are directly linked to the shader node
                    case _:
                        links.new(
                            img_texture.outputs["Color"],
                            shader_node.inputs[recipe.shader_socket_name],
                        )

            return pbr_material

        @staticmethod
        def _unwrap_uv_on_active_obj(obj: bpy.types.Object, texture_resolution: int):
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.select_all(action="SELECT")

            # Try with the default unwrap first, but capture the output in case it fails (printed as warning to stdout)
            stdout_str = io.StringIO()
            with contextlib.redirect_stdout(stdout_str):
                bpy.ops.uv.unwrap()

            # If the default unwrap failed, try with the Smart UV Project
            if "Unwrap failed" in stdout_str.getvalue():
                bpy.ops.uv.smart_project(
                    rotate_method="AXIS_ALIGNED", island_margin=0.5 / texture_resolution
                )

            bpy.ops.object.mode_set(mode="OBJECT")

        @classmethod
        def _create_image_node(
            cls,
            material: bpy.types.Material,
            recipe: ProceduralGenerator.Recipe,
            texture_resolution: int,
        ):
            node = material.node_tree.nodes.new("ShaderNodeTexImage")
            node.image = cls._create_image_texture(
                name=recipe.name.lower(),
                texture_resolution=texture_resolution,
                color_space=recipe.color_space,
            )

            material.node_tree.nodes.active = node

            return node

        @staticmethod
        def _create_image_texture(
            name: str,
            *,
            texture_resolution: int,
            color_space: Literal["Non-Color", "sRGB"],
        ):
            image = bpy.data.images.new(
                name,
                width=texture_resolution,
                height=texture_resolution,
                alpha=False,
                float_buffer=False,
                tiled=False,
            )
            image.alpha_mode = "NONE"
            image.colorspace_settings.name = color_space

            return image

    class Exporter:
        """
        Simple wrapper around Blender export capabilities.
        """

        DEFAULT_USD_EXPORT_OVERRIDES = {
            "check_existing": False,
            "selected_objects_only": True,
            "author_blender_name": False,
            "use_instancing": True,
        }

        @classmethod
        def usd_export(
            cls,
            filepath: Union[str, Path],
            *,
            ext: str = ".usdz",
            makedirs: bool = True,
            **kwargs,
        ):
            """
            Export via `bpy.ops.wm.usd_export()`
            """

            # Prepare the output path
            if not isinstance(filepath, Path):
                filepath = Path(filepath)
            filepath = filepath.with_suffix(ext).absolute()

            # Create parent directories if necessary
            if makedirs:
                os.makedirs(name=filepath.parent, exist_ok=True)

            # Forward export kwargs
            export_kwargs = deepcopy(cls.DEFAULT_USD_EXPORT_OVERRIDES)
            export_kwargs.update(kwargs)

            # Export the USD file
            bpy.ops.wm.usd_export(
                filepath=filepath.as_posix(),
                **export_kwargs,
            )


### String utils ###
REGEX_CANONICALIZE_STR_PATTERN: re.Pattern = re.compile("[\W_]+")


def canonicalize_str(input: str) -> str:
    """
    Canonicalizes a string by converting it to lowercase and removing unwanted characters.

    This function processes the input string to ensure it is in a standardized format, making it suitable for consistent usage in applications. It utilizes a predefined regular expression pattern to eliminate any characters that do not meet the specified criteria.

    Args:
        input (str): The string to be canonicalized.

    Returns:
        str: The canonicalized version of the input string.
    """
    return REGEX_CANONICALIZE_STR_PATTERN.sub("", input.lower())


### Misc utils ###
def print_bpy(msg: Any, file: Optional[TextIO] = sys.stdout, *args, **kwargs):
    """
    Helper print function that also provides output inside the Blender console in addition to the system console.
    """

    print(msg, file=file, *args, **kwargs)
    for window in bpy.context.window_manager.windows:
        for area in window.screen.areas:
            if area.type == "CONSOLE":
                with bpy.context.temp_override(
                    window=window, screen=window.screen, area=area
                ):
                    bpy.ops.console.scrollback_append(
                        text=str(msg),
                        type="ERROR" if file == sys.stderr else "OUTPUT",
                    )


def verify_requirements():
    VERSION_BPY_MIN: Tuple[int, int] = (4, 2)
    if (
        bpy.app.version[0] != VERSION_BPY_MIN[0]
        or bpy.app.version[1] < VERSION_BPY_MIN[1]
    ):
        print_bpy(
            f"[WARNING]: Blender {bpy.app.version_string} is likely incompatible with this script (written for Blender {VERSION_BPY_MIN[0]}.{VERSION_BPY_MIN[1]})",
            file=sys.stderr,
        )


### CLI ###
def parse_cli_args() -> argparse.Namespace:
    """
    Parse command-line arguments for this script.
    """

    parser = argparse.ArgumentParser(
        description="Generate procedural dataset of USD assets using Blender",
        usage=f"{sys.argv[0] if sys.argv[0].endswith('blender') else 'blender'} --python {path.realpath(__file__)} -- [options]",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        argument_default=argparse.SUPPRESS,
    )

    group = parser.add_argument_group("Input")
    group.add_argument(
        "-i",
        "--autorun_scripts",
        type=str,
        nargs="*",
        help="List of Blender scripts to execute",
        required=True,
    )

    group = parser.add_argument_group("Output")
    group.add_argument(
        "-o",
        "--outdir",
        type=str,
        help="The output directory",
        required=True,
    )
    group.add_argument(
        "--name",
        type=str,
        help="The base name of the exported models",
        default="model",
    )
    group.add_argument(
        "--ext",
        type=str,
        help="The file extension of the exported models",
        default=".usdz",
    )
    group.add_argument(
        "--overwrite_min_age",
        type=int,
        help="Number of seconds after which to overwrite the generated assets if they already exist (disabled if negative)",
        default=0,
    )

    group = parser.add_argument_group("Generator")
    group.add_argument(
        "-s",
        "--seed",
        type=int,
        help="The initial seed of the random number generator",
        default=0,
    )
    group.add_argument(
        "-n",
        "--num_assets",
        type=int,
        help="Number of assets to generate",
        default=1,
    )

    group = parser.add_argument_group("Export")
    group.add_argument(
        "--export_kwargs",
        type=json.loads,
        help="Keyword arguments for the USD export",
        default={},
    )

    group = parser.add_argument_group("Geometry")
    group.add_argument(
        "--geometry_nodes",
        type=json.loads,
        help="List of Geometry Nodes modifiers from `--autorun_scripts` for generating the geometry, with an optional dictionary for configuring their inputs",
        required=True,
    )
    group.add_argument(
        "--decimate_angle_limit",
        type=float,
        help="If specified, decimate the generated geometry to the specified target angle limit",
        default=None,
    )
    group.add_argument(
        "--decimate_face_count",
        type=int,
        help="If specified, decimate the generated geometry to the specified target face count",
        default=None,
    )

    group = parser.add_argument_group("Material")
    group.add_argument(
        "--material",
        type=str,
        help="Material of the generated models from `--autorun_scripts`, which will be baked as PBR textures into the USD file",
        default=None,
    )
    group.add_argument(
        "--texture_resolution",
        type=int,
        help="Resolution of the baked PBR textures",
        default=1024,
    )

    if "--" in sys.argv:
        args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :])
    else:
        args, unknown_args = parser.parse_known_args()
        if unknown_args:
            print_bpy(
                f"[WARNING]: Unknown args: {unknown_args}",
                file=sys.stderr,
            )
            print_bpy(
                '[HINT]: Consider delimiting your args for Python script with "--"'
            )

    return args


if __name__ == "__main__":
    main(**vars(parse_cli_args()))

import bpy

mat = bpy.data.materials.new(name="LunarRock")
mat.use_nodes = True


def random_x4___mat_node_group():
    random_x4___mat = bpy.data.node_groups.new(
        type="ShaderNodeTree", name="Random x4 | Mat"
    )
    random_x4___mat.color_tag = "NONE"
    _0_socket = random_x4___mat.interface.new_socket(
        name="0", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    _0_socket.default_value = 0.0
    _0_socket.subtype = "NONE"
    _0_socket.attribute_domain = "POINT"
    _1_socket = random_x4___mat.interface.new_socket(
        name="1", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    _1_socket.default_value = 0.0
    _1_socket.subtype = "NONE"
    _1_socket.attribute_domain = "POINT"
    _2_socket = random_x4___mat.interface.new_socket(
        name="2", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    _2_socket.default_value = 0.0
    _2_socket.subtype = "NONE"
    _2_socket.attribute_domain = "POINT"
    _3_socket = random_x4___mat.interface.new_socket(
        name="3", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    _3_socket.default_value = 0.0
    _3_socket.subtype = "NONE"
    _3_socket.attribute_domain = "POINT"
    _4_socket = random_x4___mat.interface.new_socket(
        name="4", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    _4_socket.default_value = 0.0
    _4_socket.subtype = "NONE"
    _4_socket.attribute_domain = "POINT"
    seed_socket = random_x4___mat.interface.new_socket(
        name="Seed", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    seed_socket.default_value = 0.0
    seed_socket.subtype = "NONE"
    seed_socket.attribute_domain = "POINT"
    group_output = random_x4___mat.nodes.new("NodeGroupOutput")
    group_output.is_active_output = True
    group_input = random_x4___mat.nodes.new("NodeGroupInput")
    object_info = random_x4___mat.nodes.new("ShaderNodeObjectInfo")
    math = random_x4___mat.nodes.new("ShaderNodeMath")
    math.operation = "ADD"
    math.use_clamp = False
    white_noise_texture = random_x4___mat.nodes.new("ShaderNodeTexWhiteNoise")
    white_noise_texture.noise_dimensions = "4D"
    separate_color = random_x4___mat.nodes.new("ShaderNodeSeparateColor")
    separate_color.mode = "RGB"
    math_001 = random_x4___mat.nodes.new("ShaderNodeMath")
    math_001.operation = "ADD"
    math_001.use_clamp = False
    white_noise_texture_001 = random_x4___mat.nodes.new("ShaderNodeTexWhiteNoise")
    white_noise_texture_001.noise_dimensions = "4D"
    separate_color_001 = random_x4___mat.nodes.new("ShaderNodeSeparateColor")
    separate_color_001.mode = "RGB"
    group_output.location = (689.6586303710938, -17.691898345947266)
    group_input.location = (-490.65618896484375, 343.00933837890625)
    object_info.location = (-490.65618896484375, 63.65891647338867)
    math.location = (-280.6562194824219, 343.00933837890625)
    white_noise_texture.location = (-70.65621948242188, 343.00933837890625)
    separate_color.location = (139.34378051757812, 343.00933837890625)
    math_001.location = (-280.6562194824219, 63.65891647338867)
    white_noise_texture_001.location = (-70.65621948242188, 63.65891647338867)
    separate_color_001.location = (139.34378051757812, 63.65891647338867)
    random_x4___mat.links.new(object_info.outputs[5], white_noise_texture.inputs[1])
    random_x4___mat.links.new(math.outputs[0], white_noise_texture.inputs[0])
    random_x4___mat.links.new(white_noise_texture.outputs[1], separate_color.inputs[0])
    random_x4___mat.links.new(object_info.outputs[3], math.inputs[1])
    random_x4___mat.links.new(group_input.outputs[0], math.inputs[0])
    random_x4___mat.links.new(separate_color.outputs[0], group_output.inputs[0])
    random_x4___mat.links.new(separate_color.outputs[1], group_output.inputs[1])
    random_x4___mat.links.new(math_001.outputs[0], white_noise_texture_001.inputs[0])
    random_x4___mat.links.new(
        white_noise_texture_001.outputs[1], separate_color_001.inputs[0]
    )
    random_x4___mat.links.new(separate_color.outputs[2], math_001.inputs[1])
    random_x4___mat.links.new(math.outputs[0], math_001.inputs[0])
    random_x4___mat.links.new(separate_color_001.outputs[0], group_output.inputs[2])
    random_x4___mat.links.new(separate_color_001.outputs[1], group_output.inputs[3])
    random_x4___mat.links.new(object_info.outputs[5], white_noise_texture_001.inputs[1])
    random_x4___mat.links.new(separate_color_001.outputs[2], group_output.inputs[4])
    return random_x4___mat


random_x4___mat = random_x4___mat_node_group()


def rockshader_node_group():
    rockshader = bpy.data.node_groups.new(type="ShaderNodeTree", name="RockShader")
    rockshader.color_tag = "NONE"
    bsdf_socket = rockshader.interface.new_socket(
        name="BSDF", in_out="OUTPUT", socket_type="NodeSocketShader"
    )
    bsdf_socket.attribute_domain = "POINT"
    scale_socket = rockshader.interface.new_socket(
        name="Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    scale_socket.default_value = 1.0
    scale_socket.subtype = "NONE"
    scale_socket.attribute_domain = "POINT"
    rock_color_1_socket = rockshader.interface.new_socket(
        name="Rock Color 1", in_out="INPUT", socket_type="NodeSocketColor"
    )
    rock_color_1_socket.default_value = (
        0.015996191650629044,
        0.015996308997273445,
        0.015996301546692848,
        1.0,
    )
    rock_color_1_socket.attribute_domain = "POINT"
    rock_color_2_socket = rockshader.interface.new_socket(
        name="Rock Color 2", in_out="INPUT", socket_type="NodeSocketColor"
    )
    rock_color_2_socket.default_value = (0.0, 0.0, 0.0, 1.0)
    rock_color_2_socket.attribute_domain = "POINT"
    edge_lightness_socket = rockshader.interface.new_socket(
        name="Edge Lightness", in_out="INPUT", socket_type="NodeSocketColor"
    )
    edge_lightness_socket.default_value = (
        0.0998980849981308,
        0.0998988226056099,
        0.09989877790212631,
        1.0,
    )
    edge_lightness_socket.attribute_domain = "POINT"
    noise_scale_socket = rockshader.interface.new_socket(
        name="Noise Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_scale_socket.default_value = 12.799999237060547
    noise_scale_socket.subtype = "NONE"
    noise_scale_socket.attribute_domain = "POINT"
    noise_detail_socket = rockshader.interface.new_socket(
        name="Noise Detail", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_detail_socket.default_value = 15.0
    noise_detail_socket.subtype = "NONE"
    noise_detail_socket.attribute_domain = "POINT"
    noise_roughness_socket = rockshader.interface.new_socket(
        name="Noise Roughness", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_roughness_socket.default_value = 0.800000011920929
    noise_roughness_socket.subtype = "FACTOR"
    noise_roughness_socket.attribute_domain = "POINT"
    light_noise_scale_socket = rockshader.interface.new_socket(
        name="Light Noise Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    light_noise_scale_socket.default_value = 15.0
    light_noise_scale_socket.subtype = "NONE"
    light_noise_scale_socket.attribute_domain = "POINT"
    light_roughness_socket = rockshader.interface.new_socket(
        name="Light Roughness", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    light_roughness_socket.default_value = 0.5716666579246521
    light_roughness_socket.subtype = "FACTOR"
    light_roughness_socket.attribute_domain = "POINT"
    rughness_socket = rockshader.interface.new_socket(
        name="Rughness", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    rughness_socket.default_value = 1.0
    rughness_socket.subtype = "NONE"
    rughness_socket.attribute_domain = "POINT"
    noise_bump_scale_socket = rockshader.interface.new_socket(
        name="Noise Bump Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_bump_scale_socket.default_value = 10.0
    noise_bump_scale_socket.subtype = "NONE"
    noise_bump_scale_socket.attribute_domain = "POINT"
    noise_bump__strength_socket = rockshader.interface.new_socket(
        name="Noise Bump  Strength", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_bump__strength_socket.default_value = 0.20000000298023224
    noise_bump__strength_socket.subtype = "FACTOR"
    noise_bump__strength_socket.attribute_domain = "POINT"
    detailed_noise_bump_strength_socket = rockshader.interface.new_socket(
        name="Detailed Noise Bump Strength",
        in_out="INPUT",
        socket_type="NodeSocketFloat",
    )
    detailed_noise_bump_strength_socket.default_value = 0.30000001192092896
    detailed_noise_bump_strength_socket.subtype = "FACTOR"
    detailed_noise_bump_strength_socket.attribute_domain = "POINT"
    edge_lightness_strength_socket = rockshader.interface.new_socket(
        name="Edge Lightness strength", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    edge_lightness_strength_socket.default_value = 0.4000000059604645
    edge_lightness_strength_socket.subtype = "FACTOR"
    edge_lightness_strength_socket.attribute_domain = "POINT"
    noise_scale_mixer_socket = rockshader.interface.new_socket(
        name="Noise scale mixer", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_scale_mixer_socket.default_value = 0.675000011920929
    noise_scale_mixer_socket.subtype = "FACTOR"
    noise_scale_mixer_socket.attribute_domain = "POINT"
    noise_bump_roughness_socket = rockshader.interface.new_socket(
        name="Noise Bump Roughness", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_bump_roughness_socket.default_value = 0.5
    noise_bump_roughness_socket.subtype = "FACTOR"
    noise_bump_roughness_socket.attribute_domain = "POINT"
    voronoi_bump_scale_socket = rockshader.interface.new_socket(
        name="Voronoi Bump Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    voronoi_bump_scale_socket.default_value = 5.0
    voronoi_bump_scale_socket.subtype = "NONE"
    voronoi_bump_scale_socket.attribute_domain = "POINT"
    voronoi_bump_strength_socket = rockshader.interface.new_socket(
        name="Voronoi Bump Strength", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    voronoi_bump_strength_socket.default_value = 1.0
    voronoi_bump_strength_socket.subtype = "FACTOR"
    voronoi_bump_strength_socket.attribute_domain = "POINT"
    group_output_1 = rockshader.nodes.new("NodeGroupOutput")
    group_output_1.is_active_output = True
    group_input_1 = rockshader.nodes.new("NodeGroupInput")
    noise_texture = rockshader.nodes.new("ShaderNodeTexNoise")
    noise_texture.noise_dimensions = "4D"
    noise_texture.noise_type = "FBM"
    noise_texture.normalize = True
    noise_texture.inputs[5].default_value = 20.0
    noise_texture.inputs[8].default_value = 0.0
    mapping_001 = rockshader.nodes.new("ShaderNodeMapping")
    mapping_001.vector_type = "POINT"
    mapping_001.inputs[2].default_value = (0.0, 0.0, 0.0)
    texture_coordinate_001 = rockshader.nodes.new("ShaderNodeTexCoord")
    texture_coordinate_001.from_instancer = False
    texture_coordinate_001.outputs[0].hide = True
    texture_coordinate_001.outputs[1].hide = True
    texture_coordinate_001.outputs[2].hide = True
    texture_coordinate_001.outputs[4].hide = True
    texture_coordinate_001.outputs[5].hide = True
    texture_coordinate_001.outputs[6].hide = True
    bump = rockshader.nodes.new("ShaderNodeBump")
    bump.invert = False
    bump.inputs[1].default_value = 1.0
    color_ramp = rockshader.nodes.new("ShaderNodeValToRGB")
    color_ramp.color_ramp.color_mode = "RGB"
    color_ramp.color_ramp.hue_interpolation = "NEAR"
    color_ramp.color_ramp.interpolation = "LINEAR"
    color_ramp.color_ramp.elements.remove(color_ramp.color_ramp.elements[0])
    color_ramp_cre_0 = color_ramp.color_ramp.elements[0]
    color_ramp_cre_0.position = 0.30181822180747986
    color_ramp_cre_0.alpha = 1.0
    color_ramp_cre_0.color = (0.0, 0.0, 0.0, 1.0)
    color_ramp_cre_1 = color_ramp.color_ramp.elements.new(0.3945455849170685)
    color_ramp_cre_1.alpha = 1.0
    color_ramp_cre_1.color = (1.0, 1.0, 1.0, 1.0)
    noise_texture_001 = rockshader.nodes.new("ShaderNodeTexNoise")
    noise_texture_001.noise_dimensions = "4D"
    noise_texture_001.noise_type = "FBM"
    noise_texture_001.normalize = True
    noise_texture_001.inputs[5].default_value = 2.0
    noise_texture_001.inputs[8].default_value = 0.0
    color_ramp_001 = rockshader.nodes.new("ShaderNodeValToRGB")
    color_ramp_001.color_ramp.color_mode = "RGB"
    color_ramp_001.color_ramp.hue_interpolation = "NEAR"
    color_ramp_001.color_ramp.interpolation = "LINEAR"
    color_ramp_001.color_ramp.elements.remove(color_ramp_001.color_ramp.elements[0])
    color_ramp_001_cre_0 = color_ramp_001.color_ramp.elements[0]
    color_ramp_001_cre_0.position = 0.4054546356201172
    color_ramp_001_cre_0.alpha = 1.0
    color_ramp_001_cre_0.color = (0.0, 0.0, 0.0, 1.0)
    color_ramp_001_cre_1 = color_ramp_001.color_ramp.elements.new(0.64090895652771)
    color_ramp_001_cre_1.alpha = 1.0
    color_ramp_001_cre_1.color = (1.0, 1.0, 1.0, 1.0)
    mix = rockshader.nodes.new("ShaderNodeMix")
    mix.blend_type = "MIX"
    mix.clamp_factor = True
    mix.clamp_result = False
    mix.data_type = "RGBA"
    mix.factor_mode = "UNIFORM"
    mix_001 = rockshader.nodes.new("ShaderNodeMix")
    mix_001.blend_type = "MIX"
    mix_001.clamp_factor = True
    mix_001.clamp_result = False
    mix_001.data_type = "RGBA"
    mix_001.factor_mode = "UNIFORM"
    geometry = rockshader.nodes.new("ShaderNodeNewGeometry")
    geometry.outputs[0].hide = True
    geometry.outputs[1].hide = True
    geometry.outputs[2].hide = True
    geometry.outputs[3].hide = True
    geometry.outputs[4].hide = True
    geometry.outputs[5].hide = True
    geometry.outputs[6].hide = True
    geometry.outputs[8].hide = True
    color_ramp_002 = rockshader.nodes.new("ShaderNodeValToRGB")
    color_ramp_002.color_ramp.color_mode = "RGB"
    color_ramp_002.color_ramp.hue_interpolation = "NEAR"
    color_ramp_002.color_ramp.interpolation = "EASE"
    color_ramp_002.color_ramp.elements.remove(color_ramp_002.color_ramp.elements[0])
    color_ramp_002_cre_0 = color_ramp_002.color_ramp.elements[0]
    color_ramp_002_cre_0.position = 0.5186362266540527
    color_ramp_002_cre_0.alpha = 1.0
    color_ramp_002_cre_0.color = (0.0, 0.0, 0.0, 1.0)
    color_ramp_002_cre_1 = color_ramp_002.color_ramp.elements.new(0.6045457124710083)
    color_ramp_002_cre_1.alpha = 1.0
    color_ramp_002_cre_1.color = (1.0, 1.0, 1.0, 1.0)
    mix_003 = rockshader.nodes.new("ShaderNodeMix")
    mix_003.blend_type = "MIX"
    mix_003.clamp_factor = True
    mix_003.clamp_result = False
    mix_003.data_type = "RGBA"
    mix_003.factor_mode = "UNIFORM"
    color_ramp_004 = rockshader.nodes.new("ShaderNodeValToRGB")
    color_ramp_004.color_ramp.color_mode = "RGB"
    color_ramp_004.color_ramp.hue_interpolation = "NEAR"
    color_ramp_004.color_ramp.interpolation = "LINEAR"
    color_ramp_004.color_ramp.elements.remove(color_ramp_004.color_ramp.elements[0])
    color_ramp_004_cre_0 = color_ramp_004.color_ramp.elements[0]
    color_ramp_004_cre_0.position = 0.0
    color_ramp_004_cre_0.alpha = 1.0
    color_ramp_004_cre_0.color = (
        0.6514015197753906,
        0.6514063477516174,
        0.6514060497283936,
        1.0,
    )
    color_ramp_004_cre_1 = color_ramp_004.color_ramp.elements.new(1.0)
    color_ramp_004_cre_1.alpha = 1.0
    color_ramp_004_cre_1.color = (1.0, 1.0, 1.0, 1.0)
    noise_texture_003 = rockshader.nodes.new("ShaderNodeTexNoise")
    noise_texture_003.noise_dimensions = "4D"
    noise_texture_003.noise_type = "FBM"
    noise_texture_003.normalize = True
    noise_texture_003.inputs[3].default_value = 15.0
    noise_texture_003.inputs[5].default_value = 0.0
    noise_texture_003.inputs[8].default_value = 0.0
    bump_001 = rockshader.nodes.new("ShaderNodeBump")
    bump_001.invert = False
    bump_001.inputs[1].default_value = 1.0
    frame_001 = rockshader.nodes.new("NodeFrame")
    frame_001.shrink = True
    frame_002 = rockshader.nodes.new("NodeFrame")
    frame_002.shrink = True
    frame = rockshader.nodes.new("NodeFrame")
    frame.shrink = True
    hue_saturation_value = rockshader.nodes.new("ShaderNodeHueSaturation")
    hue_saturation_value.inputs[0].default_value = 0.5
    hue_saturation_value.inputs[1].default_value = 1.0
    hue_saturation_value.inputs[3].default_value = 1.0
    frame_003 = rockshader.nodes.new("NodeFrame")
    frame_003.shrink = True
    principled_bsdf = rockshader.nodes.new("ShaderNodeBsdfPrincipled")
    principled_bsdf.distribution = "MULTI_GGX"
    principled_bsdf.subsurface_method = "RANDOM_WALK"
    principled_bsdf.inputs[1].default_value = 0.0
    principled_bsdf.inputs[3].default_value = 1.5
    principled_bsdf.inputs[4].default_value = 1.0
    principled_bsdf.inputs[7].default_value = 0.0
    principled_bsdf.inputs[8].default_value = (
        1.0,
        0.20000000298023224,
        0.10000000149011612,
    )
    principled_bsdf.inputs[9].default_value = 0.05000000074505806
    principled_bsdf.inputs[11].default_value = 0.0
    principled_bsdf.inputs[12].default_value = 0.5
    principled_bsdf.inputs[13].default_value = (1.0, 1.0, 1.0, 1.0)
    principled_bsdf.inputs[14].default_value = 0.0
    principled_bsdf.inputs[15].default_value = 0.0
    principled_bsdf.inputs[16].default_value = (0.0, 0.0, 0.0)
    principled_bsdf.inputs[17].default_value = 0.0
    principled_bsdf.inputs[18].default_value = 0.0
    principled_bsdf.inputs[19].default_value = 0.029999999329447746
    principled_bsdf.inputs[20].default_value = 1.5
    principled_bsdf.inputs[21].default_value = (1.0, 1.0, 1.0, 1.0)
    principled_bsdf.inputs[22].default_value = (0.0, 0.0, 0.0)
    principled_bsdf.inputs[23].default_value = 0.0
    principled_bsdf.inputs[24].default_value = 0.5
    principled_bsdf.inputs[25].default_value = (1.0, 1.0, 1.0, 1.0)
    principled_bsdf.inputs[26].default_value = (1.0, 1.0, 1.0, 1.0)
    principled_bsdf.inputs[27].default_value = 0.0
    principled_bsdf.inputs[28].default_value = 0.0
    principled_bsdf.inputs[29].default_value = 1.3300000429153442
    math_1 = rockshader.nodes.new("ShaderNodeMath")
    math_1.operation = "MULTIPLY"
    math_1.use_clamp = False
    math_1.inputs[1].default_value = 10.0
    group_001 = rockshader.nodes.new("ShaderNodeGroup")
    group_001.node_tree = random_x4___mat
    group_001.inputs[0].default_value = 0.5213124752044678
    voronoi_texture = rockshader.nodes.new("ShaderNodeTexVoronoi")
    voronoi_texture.distance = "EUCLIDEAN"
    voronoi_texture.feature = "F1"
    voronoi_texture.normalize = True
    voronoi_texture.voronoi_dimensions = "4D"
    voronoi_texture.inputs[3].default_value = 0.0
    voronoi_texture.inputs[4].default_value = 1.0
    voronoi_texture.inputs[5].default_value = 2.0
    voronoi_texture.inputs[8].default_value = 1.0
    bump_002 = rockshader.nodes.new("ShaderNodeBump")
    bump_002.invert = False
    bump_002.inputs[1].default_value = 1.0
    color_ramp_005 = rockshader.nodes.new("ShaderNodeValToRGB")
    color_ramp_005.color_ramp.color_mode = "RGB"
    color_ramp_005.color_ramp.hue_interpolation = "NEAR"
    color_ramp_005.color_ramp.interpolation = "EASE"
    color_ramp_005.color_ramp.elements.remove(color_ramp_005.color_ramp.elements[0])
    color_ramp_005_cre_0 = color_ramp_005.color_ramp.elements[0]
    color_ramp_005_cre_0.position = 0.0
    color_ramp_005_cre_0.alpha = 1.0
    color_ramp_005_cre_0.color = (0.0, 0.0, 0.0, 1.0)
    color_ramp_005_cre_1 = color_ramp_005.color_ramp.elements.new(0.15909108519554138)
    color_ramp_005_cre_1.alpha = 1.0
    color_ramp_005_cre_1.color = (1.0, 1.0, 1.0, 1.0)
    voronoi_texture_001 = rockshader.nodes.new("ShaderNodeTexVoronoi")
    voronoi_texture_001.distance = "EUCLIDEAN"
    voronoi_texture_001.feature = "SMOOTH_F1"
    voronoi_texture_001.normalize = True
    voronoi_texture_001.voronoi_dimensions = "4D"
    voronoi_texture_001.inputs[3].default_value = 0.0
    voronoi_texture_001.inputs[4].default_value = 1.0
    voronoi_texture_001.inputs[5].default_value = 2.0
    voronoi_texture_001.inputs[6].default_value = 1.0
    voronoi_texture_001.inputs[8].default_value = 1.0
    color_ramp_006 = rockshader.nodes.new("ShaderNodeValToRGB")
    color_ramp_006.color_ramp.color_mode = "RGB"
    color_ramp_006.color_ramp.hue_interpolation = "NEAR"
    color_ramp_006.color_ramp.interpolation = "CARDINAL"
    color_ramp_006.color_ramp.elements.remove(color_ramp_006.color_ramp.elements[0])
    color_ramp_006_cre_0 = color_ramp_006.color_ramp.elements[0]
    color_ramp_006_cre_0.position = 0.0
    color_ramp_006_cre_0.alpha = 1.0
    color_ramp_006_cre_0.color = (0.0, 0.0, 0.0, 1.0)
    color_ramp_006_cre_1 = color_ramp_006.color_ramp.elements.new(0.13181859254837036)
    color_ramp_006_cre_1.alpha = 1.0
    color_ramp_006_cre_1.color = (1.0, 1.0, 1.0, 1.0)
    math_001_1 = rockshader.nodes.new("ShaderNodeMath")
    math_001_1.operation = "DIVIDE"
    math_001_1.use_clamp = False
    bump_003 = rockshader.nodes.new("ShaderNodeBump")
    bump_003.invert = False
    bump_003.inputs[1].default_value = 1.0
    bump_003.inputs[3].default_value = (0.0, 0.0, 0.0)
    map_range_004 = rockshader.nodes.new("ShaderNodeMapRange")
    map_range_004.clamp = True
    map_range_004.data_type = "FLOAT"
    map_range_004.interpolation_type = "LINEAR"
    map_range_004.inputs[1].default_value = 0.0
    map_range_004.inputs[2].default_value = 1.0
    map_range_004.inputs[3].default_value = -1000.0
    map_range_004.inputs[4].default_value = 1000.0
    group_002 = rockshader.nodes.new("ShaderNodeGroup")
    group_002.node_tree = random_x4___mat
    math_002 = rockshader.nodes.new("ShaderNodeMath")
    math_002.operation = "MULTIPLY"
    math_002.use_clamp = False
    math_003 = rockshader.nodes.new("ShaderNodeMath")
    math_003.operation = "MULTIPLY"
    math_003.use_clamp = False
    math_003.inputs[1].default_value = 5.0
    math_004 = rockshader.nodes.new("ShaderNodeMath")
    math_004.operation = "MULTIPLY"
    math_004.use_clamp = False
    noise_texture.parent = frame
    color_ramp.parent = frame
    noise_texture_001.parent = frame
    color_ramp_001.parent = frame
    mix.parent = frame
    mix_001.parent = frame_002
    geometry.parent = frame_001
    color_ramp_002.parent = frame_001
    mix_003.parent = frame_002
    color_ramp_004.parent = frame_003
    hue_saturation_value.parent = frame_003
    group_output_1.location = (2044.083740234375, -366.00262451171875)
    group_input_1.location = (-1756.011962890625, -822.6982421875)
    noise_texture.location = (-3084.742431640625, 781.9205322265625)
    mapping_001.location = (-1281.65478515625, -227.8770751953125)
    texture_coordinate_001.location = (-1471.65478515625, -236.3770751953125)
    bump.location = (1154.56298828125, -790.7999267578125)
    color_ramp.location = (-2845.918701171875, 769.3270263671875)
    noise_texture_001.location = (-3091.82958984375, 348.28857421875)
    color_ramp_001.location = (-2840.2607421875, 369.6982421875)
    mix.location = (-2463.6015625, 642.8758544921875)
    mix_001.location = (-1338.03955078125, 856.2105102539062)
    geometry.location = (-1595.263427734375, 1377.4110107421875)
    color_ramp_002.location = (-1332.5478515625, 1497.3221435546875)
    mix_003.location = (-1139.2666015625, 857.9177856445312)
    color_ramp_004.location = (-1898.9849853515625, 572.5324096679688)
    noise_texture_003.location = (233.37887573242188, -895.6905517578125)
    bump_001.location = (1390.9708251953125, -663.4024658203125)
    frame_001.location = (1076.4444580078125, -1275.853271484375)
    frame_002.location = (1587.0386962890625, -923.2500610351562)
    frame.location = (2204.56005859375, -1019.8477783203125)
    hue_saturation_value.location = (-1571.6060791015625, 569.7412719726562)
    frame_003.location = (2145.8759765625, -1014.9539794921875)
    principled_bsdf.location = (1568.39306640625, -416.8108215332031)
    math_1.location = (-1059.811279296875, -390.11346435546875)
    group_001.location = (-2127.677001953125, -45.7719612121582)
    voronoi_texture.location = (201.54551696777344, -1322.15673828125)
    bump_002.location = (925.5811157226562, -915.0869750976562)
    color_ramp_005.location = (387.2950439453125, -1225.90478515625)
    voronoi_texture_001.location = (209.61325073242188, -1741.732666015625)
    color_ramp_006.location = (464.92108154296875, -1571.82275390625)
    math_001_1.location = (-162.15603637695312, -1974.9114990234375)
    bump_003.location = (761.9248046875, -1172.5350341796875)
    map_range_004.location = (-1697.904541015625, -193.53184509277344)
    group_002.location = (-1084.7215576171875, -1829.677734375)
    math_002.location = (-578.4093627929688, -1308.6357421875)
    math_003.location = (-452.7193603515625, -1984.625732421875)
    math_004.location = (-351.4325866699219, -1473.386962890625)
    rockshader.links.new(mapping_001.outputs[0], noise_texture_001.inputs[0])
    rockshader.links.new(noise_texture_001.outputs[0], color_ramp_001.inputs[0])
    rockshader.links.new(color_ramp_001.outputs[0], mix.inputs[7])
    rockshader.links.new(color_ramp_004.outputs[0], hue_saturation_value.inputs[4])
    rockshader.links.new(mix_001.outputs[2], mix_003.inputs[6])
    rockshader.links.new(mix_003.outputs[2], principled_bsdf.inputs[0])
    rockshader.links.new(color_ramp_002.outputs[0], mix_003.inputs[0])
    rockshader.links.new(hue_saturation_value.outputs[0], principled_bsdf.inputs[2])
    rockshader.links.new(color_ramp.outputs[0], mix.inputs[6])
    rockshader.links.new(mix.outputs[2], color_ramp_004.inputs[0])
    rockshader.links.new(mapping_001.outputs[0], noise_texture_003.inputs[0])
    rockshader.links.new(bump.outputs[0], bump_001.inputs[3])
    rockshader.links.new(mix.outputs[2], mix_001.inputs[0])
    rockshader.links.new(mapping_001.outputs[0], noise_texture.inputs[0])
    rockshader.links.new(geometry.outputs[7], color_ramp_002.inputs[0])
    rockshader.links.new(mix.outputs[2], bump_001.inputs[2])
    rockshader.links.new(noise_texture.outputs[0], color_ramp.inputs[0])
    rockshader.links.new(texture_coordinate_001.outputs[3], mapping_001.inputs[0])
    rockshader.links.new(principled_bsdf.outputs[0], group_output_1.inputs[0])
    rockshader.links.new(group_input_1.outputs[0], mapping_001.inputs[3])
    rockshader.links.new(group_input_1.outputs[1], mix_001.inputs[6])
    rockshader.links.new(group_input_1.outputs[2], mix_001.inputs[7])
    rockshader.links.new(group_input_1.outputs[3], mix_003.inputs[7])
    rockshader.links.new(group_input_1.outputs[5], noise_texture.inputs[3])
    rockshader.links.new(group_input_1.outputs[6], noise_texture.inputs[4])
    rockshader.links.new(group_input_1.outputs[5], noise_texture_001.inputs[3])
    rockshader.links.new(group_input_1.outputs[6], noise_texture_001.inputs[4])
    rockshader.links.new(group_input_1.outputs[9], hue_saturation_value.inputs[2])
    rockshader.links.new(group_input_1.outputs[11], bump.inputs[0])
    rockshader.links.new(group_input_1.outputs[10], noise_texture_003.inputs[2])
    rockshader.links.new(group_input_1.outputs[12], bump_001.inputs[0])
    rockshader.links.new(group_input_1.outputs[4], noise_texture_001.inputs[2])
    rockshader.links.new(group_input_1.outputs[14], mix.inputs[0])
    rockshader.links.new(group_input_1.outputs[4], math_1.inputs[0])
    rockshader.links.new(math_1.outputs[0], noise_texture.inputs[2])
    rockshader.links.new(group_input_1.outputs[15], noise_texture_003.inputs[4])
    rockshader.links.new(group_001.outputs[4], noise_texture_001.inputs[1])
    rockshader.links.new(group_001.outputs[3], noise_texture.inputs[1])
    rockshader.links.new(group_001.outputs[1], noise_texture_003.inputs[1])
    rockshader.links.new(bump_001.outputs[0], principled_bsdf.inputs[5])
    rockshader.links.new(noise_texture_003.outputs[0], bump.inputs[2])
    rockshader.links.new(mapping_001.outputs[0], voronoi_texture.inputs[0])
    rockshader.links.new(group_001.outputs[1], voronoi_texture.inputs[1])
    rockshader.links.new(color_ramp_005.outputs[0], bump_002.inputs[2])
    rockshader.links.new(bump_002.outputs[0], bump.inputs[3])
    rockshader.links.new(voronoi_texture.outputs[0], color_ramp_005.inputs[0])
    rockshader.links.new(group_input_1.outputs[16], voronoi_texture.inputs[2])
    rockshader.links.new(mapping_001.outputs[0], voronoi_texture_001.inputs[0])
    rockshader.links.new(group_001.outputs[1], voronoi_texture_001.inputs[1])
    rockshader.links.new(math_001_1.outputs[0], voronoi_texture_001.inputs[2])
    rockshader.links.new(voronoi_texture_001.outputs[0], color_ramp_006.inputs[0])
    rockshader.links.new(group_input_1.outputs[16], math_001_1.inputs[0])
    rockshader.links.new(color_ramp_006.outputs[0], bump_003.inputs[2])
    rockshader.links.new(bump_003.outputs[0], bump_002.inputs[3])
    rockshader.links.new(map_range_004.outputs[0], mapping_001.inputs[1])
    rockshader.links.new(group_001.outputs[0], map_range_004.inputs[0])
    rockshader.links.new(group_002.outputs[0], math_002.inputs[1])
    rockshader.links.new(group_input_1.outputs[17], math_002.inputs[0])
    rockshader.links.new(math_002.outputs[0], bump_003.inputs[0])
    rockshader.links.new(group_001.outputs[2], group_002.inputs[0])
    rockshader.links.new(math_003.outputs[0], math_001_1.inputs[1])
    rockshader.links.new(group_002.outputs[1], math_003.inputs[0])
    rockshader.links.new(group_input_1.outputs[17], math_004.inputs[0])
    rockshader.links.new(group_002.outputs[2], math_004.inputs[1])
    rockshader.links.new(math_004.outputs[0], bump_002.inputs[0])
    return rockshader


rockshader = rockshader_node_group()


def lunarrock_node_group():
    lunarrock = mat.node_tree
    for node in lunarrock.nodes:
        lunarrock.nodes.remove(node)
    lunarrock.color_tag = "NONE"
    material_output = lunarrock.nodes.new("ShaderNodeOutputMaterial")
    material_output.is_active_output = True
    material_output.target = "ALL"
    material_output.inputs[2].default_value = (0.0, 0.0, 0.0)
    material_output.inputs[3].default_value = 0.0
    group_006 = lunarrock.nodes.new("ShaderNodeGroup")
    group_006.node_tree = rockshader
    group_006.inputs[0].default_value = 4.0
    group_006.inputs[4].default_value = 7.0
    group_006.inputs[5].default_value = 15.0
    group_006.inputs[6].default_value = 0.25
    group_006.inputs[7].default_value = 5.0
    group_006.inputs[8].default_value = 0.800000011920929
    group_006.inputs[9].default_value = 1.0
    group_006.inputs[10].default_value = 15.0
    group_006.inputs[11].default_value = 0.05000000074505806
    group_006.inputs[12].default_value = 0.25
    group_006.inputs[13].default_value = 0.75
    group_006.inputs[14].default_value = 0.009999999776482582
    group_006.inputs[15].default_value = 1.0
    group_006.inputs[16].default_value = 20.0
    group_006.inputs[17].default_value = 0.75
    combine_color_004 = lunarrock.nodes.new("ShaderNodeCombineColor")
    combine_color_004.mode = "HSV"
    combine_color_004.inputs[0].default_value = 0.0
    combine_color_004.inputs[1].default_value = 0.0
    map_range_009 = lunarrock.nodes.new("ShaderNodeMapRange")
    map_range_009.clamp = True
    map_range_009.data_type = "FLOAT"
    map_range_009.interpolation_type = "LINEAR"
    map_range_009.inputs[1].default_value = 0.0
    map_range_009.inputs[2].default_value = 1.0
    map_range_009.inputs[3].default_value = 0.02500000037252903
    map_range_009.inputs[4].default_value = 0.10000000149011612
    combine_color_005 = lunarrock.nodes.new("ShaderNodeCombineColor")
    combine_color_005.mode = "HSV"
    combine_color_005.inputs[0].default_value = 0.0
    combine_color_005.inputs[1].default_value = 0.0
    map_range_010 = lunarrock.nodes.new("ShaderNodeMapRange")
    map_range_010.clamp = True
    map_range_010.data_type = "FLOAT"
    map_range_010.interpolation_type = "LINEAR"
    map_range_010.inputs[1].default_value = 0.0
    map_range_010.inputs[2].default_value = 1.0
    map_range_010.inputs[3].default_value = 0.0
    map_range_010.inputs[4].default_value = 0.02500000037252903
    group = lunarrock.nodes.new("ShaderNodeGroup")
    group.node_tree = random_x4___mat
    group.inputs[0].default_value = 0.5123251080513
    combine_color_001 = lunarrock.nodes.new("ShaderNodeCombineColor")
    combine_color_001.mode = "HSV"
    combine_color_001.inputs[0].default_value = 0.0
    combine_color_001.inputs[1].default_value = 0.0
    map_range_005 = lunarrock.nodes.new("ShaderNodeMapRange")
    map_range_005.clamp = True
    map_range_005.data_type = "FLOAT"
    map_range_005.interpolation_type = "LINEAR"
    map_range_005.inputs[1].default_value = 0.0
    map_range_005.inputs[2].default_value = 1.0
    map_range_005.inputs[3].default_value = 0.20000000298023224
    map_range_005.inputs[4].default_value = 0.3499999940395355
    material_output.location = (-63.728790283203125, -75.2525634765625)
    group_006.location = (-555.7568969726562, 120.46104431152344)
    combine_color_004.location = (-975.1624145507812, 48.7264404296875)
    map_range_009.location = (-1165.162353515625, 97.7264404296875)
    combine_color_005.location = (-971.2743530273438, -249.97727966308594)
    map_range_010.location = (-1165.162353515625, -265.1905517578125)
    group.location = (-1451.4873046875, -12.02740478515625)
    combine_color_001.location = (-972.9628295898438, 405.77777099609375)
    map_range_005.location = (-1162.962890625, 454.77777099609375)
    lunarrock.links.new(map_range_009.outputs[0], combine_color_004.inputs[2])
    lunarrock.links.new(map_range_010.outputs[0], combine_color_005.inputs[2])
    lunarrock.links.new(group.outputs[1], map_range_009.inputs[0])
    lunarrock.links.new(group.outputs[2], map_range_010.inputs[0])
    lunarrock.links.new(combine_color_005.outputs[0], group_006.inputs[2])
    lunarrock.links.new(combine_color_004.outputs[0], group_006.inputs[1])
    lunarrock.links.new(map_range_005.outputs[0], combine_color_001.inputs[2])
    lunarrock.links.new(combine_color_001.outputs[0], group_006.inputs[3])
    lunarrock.links.new(group.outputs[0], map_range_005.inputs[0])
    lunarrock.links.new(group_006.outputs[0], material_output.inputs[0])
    return lunarrock


lunarrock = lunarrock_node_group()

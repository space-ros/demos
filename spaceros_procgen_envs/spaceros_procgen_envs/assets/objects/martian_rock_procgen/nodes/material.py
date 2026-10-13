import bpy

mat = bpy.data.materials.new(name="MartianRock")
mat.use_nodes = True


def random_x2___mat_node_group():
    random_x2___mat = bpy.data.node_groups.new(
        type="ShaderNodeTree", name="Random x2 | Mat"
    )
    random_x2___mat.color_tag = "NONE"
    _0_socket = random_x2___mat.interface.new_socket(
        name="0", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    _0_socket.default_value = 0.0
    _0_socket.subtype = "NONE"
    _0_socket.attribute_domain = "POINT"
    _1_socket = random_x2___mat.interface.new_socket(
        name="1", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    _1_socket.default_value = 0.0
    _1_socket.subtype = "NONE"
    _1_socket.attribute_domain = "POINT"
    _2_socket = random_x2___mat.interface.new_socket(
        name="2", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    _2_socket.default_value = 0.0
    _2_socket.subtype = "NONE"
    _2_socket.attribute_domain = "POINT"
    seed_socket = random_x2___mat.interface.new_socket(
        name="Seed", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    seed_socket.default_value = 0.0
    seed_socket.subtype = "NONE"
    seed_socket.attribute_domain = "POINT"
    group_output = random_x2___mat.nodes.new("NodeGroupOutput")
    group_output.is_active_output = True
    group_input = random_x2___mat.nodes.new("NodeGroupInput")
    object_info = random_x2___mat.nodes.new("ShaderNodeObjectInfo")
    math = random_x2___mat.nodes.new("ShaderNodeMath")
    math.operation = "ADD"
    math.use_clamp = False
    white_noise_texture = random_x2___mat.nodes.new("ShaderNodeTexWhiteNoise")
    white_noise_texture.noise_dimensions = "4D"
    separate_color = random_x2___mat.nodes.new("ShaderNodeSeparateColor")
    separate_color.mode = "RGB"
    group_output.location = (689.6586303710938, -17.691898345947266)
    group_input.location = (-490.65618896484375, 343.00933837890625)
    object_info.location = (-490.65618896484375, 63.65891647338867)
    math.location = (-280.6562194824219, 343.00933837890625)
    white_noise_texture.location = (-70.65621948242188, 343.00933837890625)
    separate_color.location = (139.34378051757812, 343.00933837890625)
    random_x2___mat.links.new(object_info.outputs[5], white_noise_texture.inputs[1])
    random_x2___mat.links.new(math.outputs[0], white_noise_texture.inputs[0])
    random_x2___mat.links.new(white_noise_texture.outputs[1], separate_color.inputs[0])
    random_x2___mat.links.new(object_info.outputs[3], math.inputs[1])
    random_x2___mat.links.new(group_input.outputs[0], math.inputs[0])
    random_x2___mat.links.new(separate_color.outputs[0], group_output.inputs[0])
    random_x2___mat.links.new(separate_color.outputs[1], group_output.inputs[1])
    random_x2___mat.links.new(separate_color.outputs[2], group_output.inputs[2])
    return random_x2___mat


random_x2___mat = random_x2___mat_node_group()


def rockshader___3_node_group():
    rockshader___3 = bpy.data.node_groups.new(
        type="ShaderNodeTree", name="RockShader | 3"
    )
    rockshader___3.color_tag = "NONE"
    shader_socket = rockshader___3.interface.new_socket(
        name="Shader", in_out="OUTPUT", socket_type="NodeSocketShader"
    )
    shader_socket.attribute_domain = "POINT"
    scale_socket = rockshader___3.interface.new_socket(
        name="Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    scale_socket.default_value = 1.0
    scale_socket.subtype = "NONE"
    scale_socket.attribute_domain = "POINT"
    color_1_socket = rockshader___3.interface.new_socket(
        name="Color 1", in_out="INPUT", socket_type="NodeSocketColor"
    )
    color_1_socket.default_value = (1.0, 0.33455199003219604, 0.12201099842786789, 1.0)
    color_1_socket.attribute_domain = "POINT"
    color_2_socket = rockshader___3.interface.new_socket(
        name="Color 2", in_out="INPUT", socket_type="NodeSocketColor"
    )
    color_2_socket.default_value = (
        0.10239599645137787,
        0.009690999984741211,
        0.0059830001555383205,
        1.0,
    )
    color_2_socket.attribute_domain = "POINT"
    color_3_socket = rockshader___3.interface.new_socket(
        name="Color 3", in_out="INPUT", socket_type="NodeSocketColor"
    )
    color_3_socket.default_value = (
        0.13511300086975098,
        0.041269998997449875,
        0.015100999735295773,
        1.0,
    )
    color_3_socket.attribute_domain = "POINT"
    color_4_socket = rockshader___3.interface.new_socket(
        name="Color 4", in_out="INPUT", socket_type="NodeSocketColor"
    )
    color_4_socket.default_value = (1.0, 0.27467700839042664, 0.0886560007929802, 1.0)
    color_4_socket.attribute_domain = "POINT"
    noise_scale_socket = rockshader___3.interface.new_socket(
        name="Noise Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_scale_socket.default_value = 16.0
    noise_scale_socket.subtype = "NONE"
    noise_scale_socket.attribute_domain = "POINT"
    chunks_scale_socket = rockshader___3.interface.new_socket(
        name="Chunks Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    chunks_scale_socket.default_value = 4.0
    chunks_scale_socket.subtype = "NONE"
    chunks_scale_socket.attribute_domain = "POINT"
    noise_detail_1_socket = rockshader___3.interface.new_socket(
        name="Noise Detail 1", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_detail_1_socket.default_value = 15.0
    noise_detail_1_socket.subtype = "NONE"
    noise_detail_1_socket.attribute_domain = "POINT"
    noise_detail_2_socket = rockshader___3.interface.new_socket(
        name="Noise Detail 2", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_detail_2_socket.default_value = 0.44999998807907104
    noise_detail_2_socket.subtype = "FACTOR"
    noise_detail_2_socket.attribute_domain = "POINT"
    distortion_socket = rockshader___3.interface.new_socket(
        name="Distortion", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    distortion_socket.default_value = 0.10000000149011612
    distortion_socket.subtype = "FACTOR"
    distortion_socket.attribute_domain = "POINT"
    roughness_socket = rockshader___3.interface.new_socket(
        name="Roughness", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    roughness_socket.default_value = 1.0
    roughness_socket.subtype = "NONE"
    roughness_socket.attribute_domain = "POINT"
    noise_bump_strength_socket = rockshader___3.interface.new_socket(
        name="Noise Bump Strength", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_bump_strength_socket.default_value = 0.20000000298023224
    noise_bump_strength_socket.subtype = "FACTOR"
    noise_bump_strength_socket.attribute_domain = "POINT"
    detail_bump_strength_socket = rockshader___3.interface.new_socket(
        name="Detail Bump Strength", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    detail_bump_strength_socket.default_value = 0.10000000149011612
    detail_bump_strength_socket.subtype = "FACTOR"
    detail_bump_strength_socket.attribute_domain = "POINT"
    group_output_1 = rockshader___3.nodes.new("NodeGroupOutput")
    group_output_1.is_active_output = True
    texture_coordinate = rockshader___3.nodes.new("ShaderNodeTexCoord")
    texture_coordinate.from_instancer = False
    mapping_001 = rockshader___3.nodes.new("ShaderNodeMapping")
    mapping_001.vector_type = "POINT"
    mapping_001.inputs[1].default_value = (0.0, 0.0, 0.0)
    mapping_001.inputs[2].default_value = (0.0, 0.0, 0.0)
    mapping_001.inputs[3].default_value = (1.0, 1.0, 1.5)
    noise_texture_001 = rockshader___3.nodes.new("ShaderNodeTexNoise")
    noise_texture_001.noise_dimensions = "3D"
    noise_texture_001.noise_type = "FBM"
    noise_texture_001.normalize = True
    noise_texture_001.inputs[2].default_value = 19.0
    noise_texture_001.inputs[3].default_value = 15.0
    noise_texture_001.inputs[4].default_value = 0.699999988079071
    noise_texture_001.inputs[5].default_value = 2.0
    noise_texture_001.inputs[8].default_value = 0.0
    colorramp_001 = rockshader___3.nodes.new("ShaderNodeValToRGB")
    colorramp_001.color_ramp.color_mode = "RGB"
    colorramp_001.color_ramp.hue_interpolation = "NEAR"
    colorramp_001.color_ramp.interpolation = "LINEAR"
    colorramp_001.color_ramp.elements.remove(colorramp_001.color_ramp.elements[0])
    colorramp_001_cre_0 = colorramp_001.color_ramp.elements[0]
    colorramp_001_cre_0.position = 0.0
    colorramp_001_cre_0.alpha = 1.0
    colorramp_001_cre_0.color = (0.0, 0.0, 0.0, 1.0)
    colorramp_001_cre_1 = colorramp_001.color_ramp.elements.new(0.604113757610321)
    colorramp_001_cre_1.alpha = 1.0
    colorramp_001_cre_1.color = (1.0, 1.0, 1.0, 1.0)
    mix_001 = rockshader___3.nodes.new("ShaderNodeMix")
    mix_001.blend_type = "MIX"
    mix_001.clamp_factor = True
    mix_001.clamp_result = False
    mix_001.data_type = "RGBA"
    mix_001.factor_mode = "UNIFORM"
    mix_001.inputs[6].default_value = (
        0.14487500488758087,
        0.14487500488758087,
        0.14487500488758087,
        1.0,
    )
    colorramp_003 = rockshader___3.nodes.new("ShaderNodeValToRGB")
    colorramp_003.color_ramp.color_mode = "RGB"
    colorramp_003.color_ramp.hue_interpolation = "NEAR"
    colorramp_003.color_ramp.interpolation = "LINEAR"
    colorramp_003.color_ramp.elements.remove(colorramp_003.color_ramp.elements[0])
    colorramp_003_cre_0 = colorramp_003.color_ramp.elements[0]
    colorramp_003_cre_0.position = 0.0
    colorramp_003_cre_0.alpha = 1.0
    colorramp_003_cre_0.color = (
        0.5663849711418152,
        0.5663849711418152,
        0.5663849711418152,
        1.0,
    )
    colorramp_003_cre_1 = colorramp_003.color_ramp.elements.new(1.0)
    colorramp_003_cre_1.alpha = 1.0
    colorramp_003_cre_1.color = (1.0, 1.0, 1.0, 1.0)
    voronoi_texture = rockshader___3.nodes.new("ShaderNodeTexVoronoi")
    voronoi_texture.distance = "EUCLIDEAN"
    voronoi_texture.feature = "F1"
    voronoi_texture.normalize = False
    voronoi_texture.voronoi_dimensions = "3D"
    voronoi_texture.inputs[2].default_value = 350.0
    voronoi_texture.inputs[3].default_value = 0.0
    voronoi_texture.inputs[4].default_value = 0.5
    voronoi_texture.inputs[5].default_value = 2.0
    voronoi_texture.inputs[8].default_value = 1.0
    bump_001 = rockshader___3.nodes.new("ShaderNodeBump")
    bump_001.invert = False
    bump_001.inputs[1].default_value = 1.0
    principled_bsdf = rockshader___3.nodes.new("ShaderNodeBsdfPrincipled")
    principled_bsdf.distribution = "GGX"
    principled_bsdf.subsurface_method = "RANDOM_WALK_SKIN"
    principled_bsdf.inputs[1].default_value = 0.0
    principled_bsdf.inputs[3].default_value = 1.4500000476837158
    principled_bsdf.inputs[4].default_value = 1.0
    principled_bsdf.inputs[7].default_value = 0.0
    principled_bsdf.inputs[8].default_value = (
        1.0,
        0.20000000298023224,
        0.10000000149011612,
    )
    principled_bsdf.inputs[9].default_value = 0.05000000074505806
    principled_bsdf.inputs[10].default_value = 1.399999976158142
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
    principled_bsdf.inputs[26].default_value = (0.0, 0.0, 0.0, 1.0)
    principled_bsdf.inputs[27].default_value = 1.0
    principled_bsdf.inputs[28].default_value = 0.0
    principled_bsdf.inputs[29].default_value = 1.3300000429153442
    mapping = rockshader___3.nodes.new("ShaderNodeMapping")
    mapping.vector_type = "POINT"
    mapping.inputs[2].default_value = (0.0, 0.0, 0.0)
    colorramp_002 = rockshader___3.nodes.new("ShaderNodeValToRGB")
    colorramp_002.color_ramp.color_mode = "RGB"
    colorramp_002.color_ramp.hue_interpolation = "NEAR"
    colorramp_002.color_ramp.interpolation = "LINEAR"
    colorramp_002.color_ramp.elements.remove(colorramp_002.color_ramp.elements[0])
    colorramp_002_cre_0 = colorramp_002.color_ramp.elements[0]
    colorramp_002_cre_0.position = 0.08226212114095688
    colorramp_002_cre_0.alpha = 1.0
    colorramp_002_cre_0.color = (0.0, 0.0, 0.0, 1.0)
    colorramp_002_cre_1 = colorramp_002.color_ramp.elements.new(0.15424175560474396)
    colorramp_002_cre_1.alpha = 1.0
    colorramp_002_cre_1.color = (
        0.3593989908695221,
        0.3593989908695221,
        0.3593989908695221,
        1.0,
    )
    colorramp_002_cre_2 = colorramp_002.color_ramp.elements.new(0.2776348292827606)
    colorramp_002_cre_2.alpha = 1.0
    colorramp_002_cre_2.color = (0.0, 0.0, 0.0, 1.0)
    colorramp = rockshader___3.nodes.new("ShaderNodeValToRGB")
    colorramp.color_ramp.color_mode = "RGB"
    colorramp.color_ramp.hue_interpolation = "NEAR"
    colorramp.color_ramp.interpolation = "LINEAR"
    colorramp.color_ramp.elements.remove(colorramp.color_ramp.elements[0])
    colorramp_cre_0 = colorramp.color_ramp.elements[0]
    colorramp_cre_0.position = 0.010282879695296288
    colorramp_cre_0.alpha = 1.0
    colorramp_cre_0.color = (1.0, 1.0, 1.0, 1.0)
    colorramp_cre_1 = colorramp.color_ramp.elements.new(0.15167097747325897)
    colorramp_cre_1.alpha = 1.0
    colorramp_cre_1.color = (0.0, 0.0, 0.0, 1.0)
    mix_002 = rockshader___3.nodes.new("ShaderNodeMix")
    mix_002.blend_type = "MIX"
    mix_002.clamp_factor = True
    mix_002.clamp_result = False
    mix_002.data_type = "RGBA"
    mix_002.factor_mode = "UNIFORM"
    mix_003 = rockshader___3.nodes.new("ShaderNodeMix")
    mix_003.blend_type = "MIX"
    mix_003.clamp_factor = True
    mix_003.clamp_result = False
    mix_003.data_type = "RGBA"
    mix_003.factor_mode = "UNIFORM"
    mix_004 = rockshader___3.nodes.new("ShaderNodeMix")
    mix_004.blend_type = "LIGHTEN"
    mix_004.clamp_factor = True
    mix_004.clamp_result = False
    mix_004.data_type = "RGBA"
    mix_004.factor_mode = "UNIFORM"
    voronoi_texture_001 = rockshader___3.nodes.new("ShaderNodeTexVoronoi")
    voronoi_texture_001.distance = "EUCLIDEAN"
    voronoi_texture_001.feature = "DISTANCE_TO_EDGE"
    voronoi_texture_001.normalize = False
    voronoi_texture_001.voronoi_dimensions = "3D"
    voronoi_texture_001.inputs[3].default_value = 0.0
    voronoi_texture_001.inputs[4].default_value = 0.5
    voronoi_texture_001.inputs[5].default_value = 2.0
    voronoi_texture_001.inputs[8].default_value = 1.0
    noise_texture = rockshader___3.nodes.new("ShaderNodeTexNoise")
    noise_texture.noise_dimensions = "3D"
    noise_texture.noise_type = "FBM"
    noise_texture.normalize = True
    noise_texture.inputs[5].default_value = 2.0
    noise_texture.inputs[8].default_value = 0.0
    mix = rockshader___3.nodes.new("ShaderNodeMix")
    mix.blend_type = "LINEAR_LIGHT"
    mix.clamp_factor = True
    mix.clamp_result = False
    mix.data_type = "RGBA"
    mix.factor_mode = "UNIFORM"
    hue_saturation_value = rockshader___3.nodes.new("ShaderNodeHueSaturation")
    hue_saturation_value.inputs[0].default_value = 0.5
    hue_saturation_value.inputs[1].default_value = 1.0
    hue_saturation_value.inputs[3].default_value = 1.0
    bump = rockshader___3.nodes.new("ShaderNodeBump")
    bump.invert = False
    bump.inputs[1].default_value = 1.0
    bump.inputs[3].default_value = (0.0, 0.0, 0.0)
    group_input_1 = rockshader___3.nodes.new("NodeGroupInput")
    clamp = rockshader___3.nodes.new("ShaderNodeClamp")
    clamp.hide = True
    clamp.clamp_type = "MINMAX"
    clamp.inputs[1].default_value = 0.0
    clamp.inputs[2].default_value = 1.0
    map_range_011 = rockshader___3.nodes.new("ShaderNodeMapRange")
    map_range_011.clamp = True
    map_range_011.data_type = "FLOAT"
    map_range_011.interpolation_type = "LINEAR"
    map_range_011.inputs[1].default_value = 0.0
    map_range_011.inputs[2].default_value = 1.0
    map_range_011.inputs[3].default_value = -1000.0
    map_range_011.inputs[4].default_value = 1000.0
    group_001 = rockshader___3.nodes.new("ShaderNodeGroup")
    group_001.node_tree = random_x2___mat
    group_001.inputs[0].default_value = 0.5241000056266785
    group_output_1.location = (1321.0780029296875, 0.0)
    texture_coordinate.location = (-1131.0780029296875, -12.876541137695312)
    mapping_001.location = (-770.3403930664062, -10.9368896484375)
    noise_texture_001.location = (-503.7137451171875, 421.400390625)
    colorramp_001.location = (-283.625, 445.20013427734375)
    mix_001.location = (-19.82373046875, 127.36956787109375)
    colorramp_003.location = (471.98138427734375, -5.3743896484375)
    voronoi_texture.location = (-214.30291748046875, 173.81625366210938)
    bump_001.location = (813.8541259765625, -215.06895446777344)
    principled_bsdf.location = (1028.9171142578125, 249.29254150390625)
    mapping.location = (-939.576171875, -12.199813842773438)
    colorramp_002.location = (353.46002197265625, 471.2227783203125)
    colorramp.location = (172.41802978515625, 188.79144287109375)
    mix_002.location = (436.03082275390625, 237.861083984375)
    mix_003.location = (599.032958984375, 246.4215087890625)
    mix_004.location = (766.0985107421875, 244.40390014648438)
    voronoi_texture_001.location = (-207.63560485839844, -74.17858123779297)
    noise_texture.location = (-564.4707641601562, 2.5886731147766113)
    mix.location = (-408.1333923339844, 3.6961328983306885)
    hue_saturation_value.location = (787.67919921875, -13.640274047851562)
    bump.location = (588.8912353515625, -232.5960235595703)
    group_input_1.location = (-1126.564453125, -255.41787719726562)
    clamp.location = (-564.4707641601562, -297.41131591796875)
    map_range_011.location = (-1311.203125, -52.487457275390625)
    group_001.location = (-1493.3907470703125, -203.5081329345703)
    rockshader___3.links.new(colorramp_001.outputs[0], mix_003.inputs[0])
    rockshader___3.links.new(mix_003.outputs[2], mix_004.inputs[6])
    rockshader___3.links.new(voronoi_texture.outputs[0], bump_001.inputs[2])
    rockshader___3.links.new(mix.outputs[2], voronoi_texture.inputs[0])
    rockshader___3.links.new(bump_001.outputs[0], principled_bsdf.inputs[5])
    rockshader___3.links.new(texture_coordinate.outputs[3], mapping.inputs[0])
    rockshader___3.links.new(hue_saturation_value.outputs[0], principled_bsdf.inputs[2])
    rockshader___3.links.new(mix_004.outputs[2], principled_bsdf.inputs[0])
    rockshader___3.links.new(voronoi_texture_001.outputs[0], colorramp_002.inputs[0])
    rockshader___3.links.new(mapping_001.outputs[0], noise_texture.inputs[0])
    rockshader___3.links.new(noise_texture.outputs[0], mix.inputs[7])
    rockshader___3.links.new(mapping_001.outputs[0], mix.inputs[6])
    rockshader___3.links.new(voronoi_texture_001.outputs[0], bump.inputs[2])
    rockshader___3.links.new(mix.outputs[2], voronoi_texture_001.inputs[0])
    rockshader___3.links.new(colorramp_003.outputs[0], hue_saturation_value.inputs[4])
    rockshader___3.links.new(mapping.outputs[0], mapping_001.inputs[0])
    rockshader___3.links.new(voronoi_texture.outputs[0], mix_001.inputs[0])
    rockshader___3.links.new(voronoi_texture_001.outputs[0], mix_001.inputs[7])
    rockshader___3.links.new(colorramp.outputs[0], mix_002.inputs[0])
    rockshader___3.links.new(mix_001.outputs[2], colorramp.inputs[0])
    rockshader___3.links.new(bump.outputs[0], bump_001.inputs[3])
    rockshader___3.links.new(mapping_001.outputs[0], noise_texture_001.inputs[0])
    rockshader___3.links.new(noise_texture_001.outputs[0], colorramp_001.inputs[0])
    rockshader___3.links.new(mix_001.outputs[2], colorramp_003.inputs[0])
    rockshader___3.links.new(mix_002.outputs[2], mix_003.inputs[6])
    rockshader___3.links.new(colorramp_002.outputs[0], mix_004.inputs[0])
    rockshader___3.links.new(principled_bsdf.outputs[0], group_output_1.inputs[0])
    rockshader___3.links.new(group_input_1.outputs[0], mapping.inputs[3])
    rockshader___3.links.new(group_input_1.outputs[1], mix_002.inputs[6])
    rockshader___3.links.new(group_input_1.outputs[2], mix_002.inputs[7])
    rockshader___3.links.new(group_input_1.outputs[3], mix_003.inputs[7])
    rockshader___3.links.new(group_input_1.outputs[4], mix_004.inputs[7])
    rockshader___3.links.new(group_input_1.outputs[5], noise_texture.inputs[2])
    rockshader___3.links.new(group_input_1.outputs[6], voronoi_texture_001.inputs[2])
    rockshader___3.links.new(group_input_1.outputs[7], noise_texture.inputs[3])
    rockshader___3.links.new(group_input_1.outputs[9], mix.inputs[0])
    rockshader___3.links.new(group_input_1.outputs[10], hue_saturation_value.inputs[2])
    rockshader___3.links.new(group_input_1.outputs[11], bump.inputs[0])
    rockshader___3.links.new(group_input_1.outputs[12], bump_001.inputs[0])
    rockshader___3.links.new(group_input_1.outputs[8], clamp.inputs[0])
    rockshader___3.links.new(clamp.outputs[0], noise_texture.inputs[4])
    rockshader___3.links.new(group_001.outputs[0], map_range_011.inputs[0])
    rockshader___3.links.new(map_range_011.outputs[0], mapping.inputs[1])
    return rockshader___3


rockshader___3 = rockshader___3_node_group()


def rockshader___4_node_group():
    rockshader___4 = bpy.data.node_groups.new(
        type="ShaderNodeTree", name="RockShader | 4"
    )
    rockshader___4.color_tag = "NONE"
    shader_socket_1 = rockshader___4.interface.new_socket(
        name="Shader", in_out="OUTPUT", socket_type="NodeSocketShader"
    )
    shader_socket_1.attribute_domain = "POINT"
    scale_socket_1 = rockshader___4.interface.new_socket(
        name="Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    scale_socket_1.default_value = 1.0
    scale_socket_1.subtype = "NONE"
    scale_socket_1.attribute_domain = "POINT"
    color_1_socket_1 = rockshader___4.interface.new_socket(
        name="Color 1", in_out="INPUT", socket_type="NodeSocketColor"
    )
    color_1_socket_1.default_value = (
        0.6590179800987244,
        0.24836499989032745,
        0.0748089998960495,
        1.0,
    )
    color_1_socket_1.attribute_domain = "POINT"
    color_2_socket_1 = rockshader___4.interface.new_socket(
        name="Color 2", in_out="INPUT", socket_type="NodeSocketColor"
    )
    color_2_socket_1.default_value = (
        0.07483299821615219,
        0.01208100002259016,
        0.006566000171005726,
        1.0,
    )
    color_2_socket_1.attribute_domain = "POINT"
    color_3_socket_1 = rockshader___4.interface.new_socket(
        name="Color 3", in_out="INPUT", socket_type="NodeSocketColor"
    )
    color_3_socket_1.default_value = (
        0.046142999082803726,
        0.007871000096201897,
        0.004050000105053186,
        1.0,
    )
    color_3_socket_1.attribute_domain = "POINT"
    noise_scale_socket_1 = rockshader___4.interface.new_socket(
        name="Noise Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_scale_socket_1.default_value = 18.0
    noise_scale_socket_1.subtype = "NONE"
    noise_scale_socket_1.attribute_domain = "POINT"
    voronoi_scale_socket = rockshader___4.interface.new_socket(
        name="Voronoi Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    voronoi_scale_socket.default_value = 16.0
    voronoi_scale_socket.subtype = "NONE"
    voronoi_scale_socket.attribute_domain = "POINT"
    wave_scale_socket = rockshader___4.interface.new_socket(
        name="Wave Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    wave_scale_socket.default_value = 6.0
    wave_scale_socket.subtype = "NONE"
    wave_scale_socket.attribute_domain = "POINT"
    cracks_scale_socket = rockshader___4.interface.new_socket(
        name="Cracks Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    cracks_scale_socket.default_value = 4.0
    cracks_scale_socket.subtype = "NONE"
    cracks_scale_socket.attribute_domain = "POINT"
    texture_detail_socket = rockshader___4.interface.new_socket(
        name="Texture Detail", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    texture_detail_socket.default_value = 15.0
    texture_detail_socket.subtype = "NONE"
    texture_detail_socket.attribute_domain = "POINT"
    roughness_socket_1 = rockshader___4.interface.new_socket(
        name="Roughness", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    roughness_socket_1.default_value = 1.0
    roughness_socket_1.subtype = "NONE"
    roughness_socket_1.attribute_domain = "POINT"
    noise_bump_strength_socket_1 = rockshader___4.interface.new_socket(
        name="Noise Bump Strength", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_bump_strength_socket_1.default_value = 0.10000000149011612
    noise_bump_strength_socket_1.subtype = "FACTOR"
    noise_bump_strength_socket_1.attribute_domain = "POINT"
    cracks_bump_strength_socket = rockshader___4.interface.new_socket(
        name="Cracks Bump Strength", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    cracks_bump_strength_socket.default_value = 0.6000000238418579
    cracks_bump_strength_socket.subtype = "FACTOR"
    cracks_bump_strength_socket.attribute_domain = "POINT"
    strength_socket = rockshader___4.interface.new_socket(
        name="Strength", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    strength_socket.default_value = 0.09132219105958939
    strength_socket.subtype = "FACTOR"
    strength_socket.attribute_domain = "POINT"
    group_output_2 = rockshader___4.nodes.new("NodeGroupOutput")
    group_output_2.is_active_output = True
    texture_coordinate_1 = rockshader___4.nodes.new("ShaderNodeTexCoord")
    texture_coordinate_1.from_instancer = False
    mapping_1 = rockshader___4.nodes.new("ShaderNodeMapping")
    mapping_1.vector_type = "POINT"
    mapping_1.inputs[2].default_value = (0.0, 0.0, 0.0)
    mapping_001_1 = rockshader___4.nodes.new("ShaderNodeMapping")
    mapping_001_1.vector_type = "POINT"
    mapping_001_1.inputs[1].default_value = (0.0, 0.0, 0.0)
    mapping_001_1.inputs[2].default_value = (0.0, 0.0, 0.0)
    mapping_001_1.inputs[3].default_value = (1.0, 1.0, 1.399999976158142)
    mix_1 = rockshader___4.nodes.new("ShaderNodeMix")
    mix_1.blend_type = "LINEAR_LIGHT"
    mix_1.clamp_factor = True
    mix_1.clamp_result = False
    mix_1.data_type = "RGBA"
    mix_1.factor_mode = "UNIFORM"
    mix_1.inputs[0].default_value = 0.10000000149011612
    colorramp_1 = rockshader___4.nodes.new("ShaderNodeValToRGB")
    colorramp_1.color_ramp.color_mode = "RGB"
    colorramp_1.color_ramp.hue_interpolation = "NEAR"
    colorramp_1.color_ramp.interpolation = "LINEAR"
    colorramp_1.color_ramp.elements.remove(colorramp_1.color_ramp.elements[0])
    colorramp_1_cre_0 = colorramp_1.color_ramp.elements[0]
    colorramp_1_cre_0.position = 0.0
    colorramp_1_cre_0.alpha = 1.0
    colorramp_1_cre_0.color = (1.0, 1.0, 1.0, 1.0)
    colorramp_1_cre_1 = colorramp_1.color_ramp.elements.new(0.20822615921497345)
    colorramp_1_cre_1.alpha = 1.0
    colorramp_1_cre_1.color = (
        0.5014079809188843,
        0.5014079809188843,
        0.5014079809188843,
        1.0,
    )
    principled_bsdf_1 = rockshader___4.nodes.new("ShaderNodeBsdfPrincipled")
    principled_bsdf_1.distribution = "GGX"
    principled_bsdf_1.subsurface_method = "RANDOM_WALK_SKIN"
    principled_bsdf_1.inputs[1].default_value = 0.0
    principled_bsdf_1.inputs[3].default_value = 1.4500000476837158
    principled_bsdf_1.inputs[4].default_value = 1.0
    principled_bsdf_1.inputs[7].default_value = 0.0
    principled_bsdf_1.inputs[8].default_value = (
        1.0,
        0.20000000298023224,
        0.10000000149011612,
    )
    principled_bsdf_1.inputs[9].default_value = 0.05000000074505806
    principled_bsdf_1.inputs[10].default_value = 1.399999976158142
    principled_bsdf_1.inputs[11].default_value = 0.0
    principled_bsdf_1.inputs[12].default_value = 0.5
    principled_bsdf_1.inputs[13].default_value = (1.0, 1.0, 1.0, 1.0)
    principled_bsdf_1.inputs[14].default_value = 0.0
    principled_bsdf_1.inputs[15].default_value = 0.0
    principled_bsdf_1.inputs[16].default_value = (0.0, 0.0, 0.0)
    principled_bsdf_1.inputs[17].default_value = 0.0
    principled_bsdf_1.inputs[18].default_value = 0.0
    principled_bsdf_1.inputs[19].default_value = 0.029999999329447746
    principled_bsdf_1.inputs[20].default_value = 1.5
    principled_bsdf_1.inputs[21].default_value = (1.0, 1.0, 1.0, 1.0)
    principled_bsdf_1.inputs[22].default_value = (0.0, 0.0, 0.0)
    principled_bsdf_1.inputs[23].default_value = 0.0
    principled_bsdf_1.inputs[24].default_value = 0.5
    principled_bsdf_1.inputs[25].default_value = (1.0, 1.0, 1.0, 1.0)
    principled_bsdf_1.inputs[26].default_value = (0.0, 0.0, 0.0, 1.0)
    principled_bsdf_1.inputs[27].default_value = 1.0
    principled_bsdf_1.inputs[28].default_value = 0.0
    principled_bsdf_1.inputs[29].default_value = 1.3300000429153442
    rgb_curves = rockshader___4.nodes.new("ShaderNodeRGBCurve")
    rgb_curves.mapping.extend = "EXTRAPOLATED"
    rgb_curves.mapping.tone = "STANDARD"
    rgb_curves.mapping.black_level = (0.0, 0.0, 0.0)
    rgb_curves.mapping.white_level = (1.0, 1.0, 1.0)
    rgb_curves.mapping.clip_min_x = 0.0
    rgb_curves.mapping.clip_min_y = 0.0
    rgb_curves.mapping.clip_max_x = 1.0
    rgb_curves.mapping.clip_max_y = 1.0
    rgb_curves.mapping.use_clip = True
    rgb_curves_curve_0 = rgb_curves.mapping.curves[0]
    rgb_curves_curve_0_point_0 = rgb_curves_curve_0.points[0]
    rgb_curves_curve_0_point_0.location = (0.0, 0.0)
    rgb_curves_curve_0_point_0.handle_type = "AUTO"
    rgb_curves_curve_0_point_1 = rgb_curves_curve_0.points[1]
    rgb_curves_curve_0_point_1.location = (1.0, 1.0)
    rgb_curves_curve_0_point_1.handle_type = "AUTO"
    rgb_curves_curve_1 = rgb_curves.mapping.curves[1]
    rgb_curves_curve_1_point_0 = rgb_curves_curve_1.points[0]
    rgb_curves_curve_1_point_0.location = (0.0, 0.0)
    rgb_curves_curve_1_point_0.handle_type = "AUTO"
    rgb_curves_curve_1_point_1 = rgb_curves_curve_1.points[1]
    rgb_curves_curve_1_point_1.location = (1.0, 1.0)
    rgb_curves_curve_1_point_1.handle_type = "AUTO"
    rgb_curves_curve_2 = rgb_curves.mapping.curves[2]
    rgb_curves_curve_2_point_0 = rgb_curves_curve_2.points[0]
    rgb_curves_curve_2_point_0.location = (0.0, 0.0)
    rgb_curves_curve_2_point_0.handle_type = "AUTO"
    rgb_curves_curve_2_point_1 = rgb_curves_curve_2.points[1]
    rgb_curves_curve_2_point_1.location = (1.0, 1.0)
    rgb_curves_curve_2_point_1.handle_type = "AUTO"
    rgb_curves_curve_3 = rgb_curves.mapping.curves[3]
    rgb_curves_curve_3_point_0 = rgb_curves_curve_3.points[0]
    rgb_curves_curve_3_point_0.location = (0.0, 0.0)
    rgb_curves_curve_3_point_0.handle_type = "AUTO"
    rgb_curves_curve_3_point_1 = rgb_curves_curve_3.points[1]
    rgb_curves_curve_3_point_1.location = (0.2107970118522644, 0.2904413044452667)
    rgb_curves_curve_3_point_1.handle_type = "AUTO"
    rgb_curves_curve_3_point_2 = rgb_curves_curve_3.points.new(
        0.4678661823272705, 0.4007352888584137
    )
    rgb_curves_curve_3_point_2.handle_type = "AUTO"
    rgb_curves_curve_3_point_3 = rgb_curves_curve_3.points.new(
        0.5861183404922485, 0.7536765933036804
    )
    rgb_curves_curve_3_point_3.handle_type = "AUTO"
    rgb_curves_curve_3_point_4 = rgb_curves_curve_3.points.new(1.0, 1.0)
    rgb_curves_curve_3_point_4.handle_type = "AUTO"
    rgb_curves.mapping.update()
    rgb_curves.inputs[0].default_value = 1.0
    mix_001_1 = rockshader___4.nodes.new("ShaderNodeMix")
    mix_001_1.blend_type = "DARKEN"
    mix_001_1.clamp_factor = True
    mix_001_1.clamp_result = False
    mix_001_1.data_type = "RGBA"
    mix_001_1.factor_mode = "UNIFORM"
    mix_001_1.inputs[0].default_value = 1.0
    colorramp_001_1 = rockshader___4.nodes.new("ShaderNodeValToRGB")
    colorramp_001_1.color_ramp.color_mode = "RGB"
    colorramp_001_1.color_ramp.hue_interpolation = "NEAR"
    colorramp_001_1.color_ramp.interpolation = "LINEAR"
    colorramp_001_1.color_ramp.elements.remove(colorramp_001_1.color_ramp.elements[0])
    colorramp_001_1_cre_0 = colorramp_001_1.color_ramp.elements[0]
    colorramp_001_1_cre_0.position = 0.0
    colorramp_001_1_cre_0.alpha = 1.0
    colorramp_001_1_cre_0.color = (0.0, 0.0, 0.0, 1.0)
    colorramp_001_1_cre_1 = colorramp_001_1.color_ramp.elements.new(0.05912623554468155)
    colorramp_001_1_cre_1.alpha = 1.0
    colorramp_001_1_cre_1.color = (1.0, 1.0, 1.0, 1.0)
    rgb_curves_001 = rockshader___4.nodes.new("ShaderNodeRGBCurve")
    rgb_curves_001.mapping.extend = "EXTRAPOLATED"
    rgb_curves_001.mapping.tone = "STANDARD"
    rgb_curves_001.mapping.black_level = (0.0, 0.0, 0.0)
    rgb_curves_001.mapping.white_level = (1.0, 1.0, 1.0)
    rgb_curves_001.mapping.clip_min_x = 0.0
    rgb_curves_001.mapping.clip_min_y = 0.0
    rgb_curves_001.mapping.clip_max_x = 1.0
    rgb_curves_001.mapping.clip_max_y = 1.0
    rgb_curves_001.mapping.use_clip = True
    rgb_curves_001_curve_0 = rgb_curves_001.mapping.curves[0]
    rgb_curves_001_curve_0_point_0 = rgb_curves_001_curve_0.points[0]
    rgb_curves_001_curve_0_point_0.location = (0.0, 0.0)
    rgb_curves_001_curve_0_point_0.handle_type = "AUTO"
    rgb_curves_001_curve_0_point_1 = rgb_curves_001_curve_0.points[1]
    rgb_curves_001_curve_0_point_1.location = (1.0, 1.0)
    rgb_curves_001_curve_0_point_1.handle_type = "AUTO"
    rgb_curves_001_curve_1 = rgb_curves_001.mapping.curves[1]
    rgb_curves_001_curve_1_point_0 = rgb_curves_001_curve_1.points[0]
    rgb_curves_001_curve_1_point_0.location = (0.0, 0.0)
    rgb_curves_001_curve_1_point_0.handle_type = "AUTO"
    rgb_curves_001_curve_1_point_1 = rgb_curves_001_curve_1.points[1]
    rgb_curves_001_curve_1_point_1.location = (1.0, 1.0)
    rgb_curves_001_curve_1_point_1.handle_type = "AUTO"
    rgb_curves_001_curve_2 = rgb_curves_001.mapping.curves[2]
    rgb_curves_001_curve_2_point_0 = rgb_curves_001_curve_2.points[0]
    rgb_curves_001_curve_2_point_0.location = (0.0, 0.0)
    rgb_curves_001_curve_2_point_0.handle_type = "AUTO"
    rgb_curves_001_curve_2_point_1 = rgb_curves_001_curve_2.points[1]
    rgb_curves_001_curve_2_point_1.location = (1.0, 1.0)
    rgb_curves_001_curve_2_point_1.handle_type = "AUTO"
    rgb_curves_001_curve_3 = rgb_curves_001.mapping.curves[3]
    rgb_curves_001_curve_3_point_0 = rgb_curves_001_curve_3.points[0]
    rgb_curves_001_curve_3_point_0.location = (0.0, 0.0)
    rgb_curves_001_curve_3_point_0.handle_type = "AUTO"
    rgb_curves_001_curve_3_point_1 = rgb_curves_001_curve_3.points[1]
    rgb_curves_001_curve_3_point_1.location = (0.34961429238319397, 0.15073515474796295)
    rgb_curves_001_curve_3_point_1.handle_type = "AUTO"
    rgb_curves_001_curve_3_point_2 = rgb_curves_001_curve_3.points.new(
        0.6143959164619446, 0.764706015586853
    )
    rgb_curves_001_curve_3_point_2.handle_type = "AUTO"
    rgb_curves_001_curve_3_point_3 = rgb_curves_001_curve_3.points.new(1.0, 1.0)
    rgb_curves_001_curve_3_point_3.handle_type = "AUTO"
    rgb_curves_001.mapping.update()
    rgb_curves_001.inputs[0].default_value = 1.0
    mix_002_1 = rockshader___4.nodes.new("ShaderNodeMix")
    mix_002_1.blend_type = "MIX"
    mix_002_1.clamp_factor = True
    mix_002_1.clamp_result = False
    mix_002_1.data_type = "RGBA"
    mix_002_1.factor_mode = "UNIFORM"
    mix_003_1 = rockshader___4.nodes.new("ShaderNodeMix")
    mix_003_1.blend_type = "MIX"
    mix_003_1.clamp_factor = True
    mix_003_1.clamp_result = False
    mix_003_1.data_type = "RGBA"
    mix_003_1.factor_mode = "UNIFORM"
    voronoi_texture_1 = rockshader___4.nodes.new("ShaderNodeTexVoronoi")
    voronoi_texture_1.distance = "CHEBYCHEV"
    voronoi_texture_1.feature = "F1"
    voronoi_texture_1.normalize = False
    voronoi_texture_1.voronoi_dimensions = "3D"
    voronoi_texture_1.inputs[3].default_value = 0.0
    voronoi_texture_1.inputs[4].default_value = 0.5
    voronoi_texture_1.inputs[5].default_value = 2.0
    voronoi_texture_1.inputs[8].default_value = 1.0
    wave_texture = rockshader___4.nodes.new("ShaderNodeTexWave")
    wave_texture.bands_direction = "Z"
    wave_texture.rings_direction = "X"
    wave_texture.wave_profile = "SIN"
    wave_texture.wave_type = "BANDS"
    wave_texture.inputs[2].default_value = 8.0
    wave_texture.inputs[4].default_value = 2.0
    wave_texture.inputs[5].default_value = 0.6200000047683716
    wave_texture.inputs[6].default_value = 0.0
    noise_texture_1 = rockshader___4.nodes.new("ShaderNodeTexNoise")
    noise_texture_1.noise_dimensions = "4D"
    noise_texture_1.noise_type = "FBM"
    noise_texture_1.normalize = True
    noise_texture_1.inputs[1].default_value = 0.0
    noise_texture_1.inputs[4].default_value = 0.550000011920929
    noise_texture_1.inputs[5].default_value = 2.0
    noise_texture_1.inputs[8].default_value = 0.0
    wave_texture_001 = rockshader___4.nodes.new("ShaderNodeTexWave")
    wave_texture_001.bands_direction = "Z"
    wave_texture_001.rings_direction = "X"
    wave_texture_001.wave_profile = "SIN"
    wave_texture_001.wave_type = "BANDS"
    wave_texture_001.inputs[2].default_value = 20.0
    wave_texture_001.inputs[4].default_value = 2.0
    wave_texture_001.inputs[5].default_value = 0.6200000047683716
    wave_texture_001.inputs[6].default_value = 0.0
    hue_saturation_value_1 = rockshader___4.nodes.new("ShaderNodeHueSaturation")
    hue_saturation_value_1.inputs[0].default_value = 0.5
    hue_saturation_value_1.inputs[1].default_value = 1.0
    hue_saturation_value_1.inputs[3].default_value = 1.0
    bump_1 = rockshader___4.nodes.new("ShaderNodeBump")
    bump_1.invert = False
    bump_1.inputs[1].default_value = 1.0
    bump_1.inputs[3].default_value = (0.0, 0.0, 0.0)
    bump_001_1 = rockshader___4.nodes.new("ShaderNodeBump")
    bump_001_1.invert = False
    bump_001_1.inputs[1].default_value = 1.0
    group_input_2 = rockshader___4.nodes.new("NodeGroupInput")
    map_range_011_1 = rockshader___4.nodes.new("ShaderNodeMapRange")
    map_range_011_1.clamp = True
    map_range_011_1.data_type = "FLOAT"
    map_range_011_1.interpolation_type = "LINEAR"
    map_range_011_1.inputs[1].default_value = 0.0
    map_range_011_1.inputs[2].default_value = 1.0
    map_range_011_1.inputs[3].default_value = -1000.0
    map_range_011_1.inputs[4].default_value = 1000.0
    group_001_1 = rockshader___4.nodes.new("ShaderNodeGroup")
    group_001_1.node_tree = random_x2___mat
    group_001_1.inputs[0].default_value = 0.12449999898672104
    bump_002 = rockshader___4.nodes.new("ShaderNodeBump")
    bump_002.invert = False
    bump_002.inputs[1].default_value = 1.0
    bump_002.inputs[2].default_value = 1.0
    group_output_2.location = (1435.3160400390625, 0.0)
    texture_coordinate_1.location = (-1245.31591796875, 66.28211975097656)
    mapping_1.location = (-1077.75537109375, 82.47952270507812)
    mapping_001_1.location = (-915.2177734375, 85.06170654296875)
    mix_1.location = (-578.0686645507812, 109.40525817871094)
    colorramp_1.location = (676.802978515625, 130.7415771484375)
    principled_bsdf_1.location = (1138.4149169921875, 346.54254150390625)
    rgb_curves.location = (-253.71597290039062, 427.0670166015625)
    mix_001_1.location = (78.52862548828125, 274.6394348144531)
    colorramp_001_1.location = (-553.4833984375, -227.36936950683594)
    rgb_curves_001.location = (-232.36538696289062, 97.50946044921875)
    mix_002_1.location = (303.08209228515625, 305.1787414550781)
    mix_003_1.location = (488.4228515625, 295.63385009765625)
    voronoi_texture_1.location = (-421.1315612792969, 115.88117980957031)
    wave_texture.location = (-747.1722412109375, 424.21881103515625)
    noise_texture_1.location = (-746.4063720703125, 85.30537414550781)
    wave_texture_001.location = (-748.0964965820312, -159.0817413330078)
    hue_saturation_value_1.location = (939.0760498046875, 123.91618347167969)
    bump_1.location = (292.3201904296875, -128.81381225585938)
    bump_001_1.location = (487.4517822265625, -126.61894226074219)
    group_input_2.location = (-1247.6279296875, -180.5272216796875)
    map_range_011_1.location = (-1488.3250732421875, -47.044227600097656)
    group_001_1.location = (-1670.5126953125, -198.06488037109375)
    bump_002.location = (694.7019653320312, -130.1945343017578)
    rockshader___4.links.new(mix_001_1.outputs[2], mix_002_1.inputs[0])
    rockshader___4.links.new(colorramp_001_1.outputs[0], bump_001_1.inputs[2])
    rockshader___4.links.new(
        hue_saturation_value_1.outputs[0], principled_bsdf_1.inputs[2]
    )
    rockshader___4.links.new(mix_1.outputs[2], voronoi_texture_1.inputs[0])
    rockshader___4.links.new(mix_003_1.outputs[2], principled_bsdf_1.inputs[0])
    rockshader___4.links.new(noise_texture_1.outputs[1], mix_1.inputs[7])
    rockshader___4.links.new(bump_1.outputs[0], bump_001_1.inputs[3])
    rockshader___4.links.new(colorramp_1.outputs[0], hue_saturation_value_1.inputs[4])
    rockshader___4.links.new(mapping_001_1.outputs[0], mix_1.inputs[6])
    rockshader___4.links.new(noise_texture_1.outputs[0], bump_1.inputs[2])
    rockshader___4.links.new(rgb_curves_001.outputs[0], mix_001_1.inputs[6])
    rockshader___4.links.new(voronoi_texture_1.outputs[0], mix_003_1.inputs[0])
    rockshader___4.links.new(mapping_001_1.outputs[0], noise_texture_1.inputs[0])
    rockshader___4.links.new(mix_002_1.outputs[2], mix_003_1.inputs[6])
    rockshader___4.links.new(mapping_001_1.outputs[0], wave_texture_001.inputs[0])
    rockshader___4.links.new(texture_coordinate_1.outputs[3], mapping_1.inputs[0])
    rockshader___4.links.new(wave_texture.outputs[0], rgb_curves.inputs[1])
    rockshader___4.links.new(voronoi_texture_1.outputs[0], rgb_curves_001.inputs[1])
    rockshader___4.links.new(mix_003_1.outputs[2], colorramp_1.inputs[0])
    rockshader___4.links.new(mapping_001_1.outputs[0], wave_texture.inputs[0])
    rockshader___4.links.new(mapping_1.outputs[0], mapping_001_1.inputs[0])
    rockshader___4.links.new(rgb_curves.outputs[0], mix_001_1.inputs[7])
    rockshader___4.links.new(wave_texture_001.outputs[0], colorramp_001_1.inputs[0])
    rockshader___4.links.new(principled_bsdf_1.outputs[0], group_output_2.inputs[0])
    rockshader___4.links.new(group_input_2.outputs[0], mapping_1.inputs[3])
    rockshader___4.links.new(group_input_2.outputs[1], mix_002_1.inputs[6])
    rockshader___4.links.new(group_input_2.outputs[2], mix_002_1.inputs[7])
    rockshader___4.links.new(group_input_2.outputs[3], mix_003_1.inputs[7])
    rockshader___4.links.new(group_input_2.outputs[4], noise_texture_1.inputs[2])
    rockshader___4.links.new(group_input_2.outputs[5], voronoi_texture_1.inputs[2])
    rockshader___4.links.new(group_input_2.outputs[6], wave_texture.inputs[1])
    rockshader___4.links.new(group_input_2.outputs[7], wave_texture_001.inputs[1])
    rockshader___4.links.new(group_input_2.outputs[8], wave_texture.inputs[3])
    rockshader___4.links.new(group_input_2.outputs[8], noise_texture_1.inputs[3])
    rockshader___4.links.new(group_input_2.outputs[8], wave_texture_001.inputs[3])
    rockshader___4.links.new(group_input_2.outputs[9], hue_saturation_value_1.inputs[2])
    rockshader___4.links.new(group_input_2.outputs[10], bump_1.inputs[0])
    rockshader___4.links.new(group_input_2.outputs[11], bump_001_1.inputs[0])
    rockshader___4.links.new(group_001_1.outputs[0], map_range_011_1.inputs[0])
    rockshader___4.links.new(map_range_011_1.outputs[0], mapping_1.inputs[1])
    rockshader___4.links.new(bump_001_1.outputs[0], bump_002.inputs[3])
    rockshader___4.links.new(bump_002.outputs[0], principled_bsdf_1.inputs[5])
    rockshader___4.links.new(group_input_2.outputs[12], bump_002.inputs[0])
    return rockshader___4


rockshader___4 = rockshader___4_node_group()


def random_x4___mat_node_group():
    random_x4___mat = bpy.data.node_groups.new(
        type="ShaderNodeTree", name="Random x4 | Mat"
    )
    random_x4___mat.color_tag = "NONE"
    _0_socket_1 = random_x4___mat.interface.new_socket(
        name="0", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    _0_socket_1.default_value = 0.0
    _0_socket_1.subtype = "NONE"
    _0_socket_1.attribute_domain = "POINT"
    _1_socket_1 = random_x4___mat.interface.new_socket(
        name="1", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    _1_socket_1.default_value = 0.0
    _1_socket_1.subtype = "NONE"
    _1_socket_1.attribute_domain = "POINT"
    _2_socket_1 = random_x4___mat.interface.new_socket(
        name="2", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    _2_socket_1.default_value = 0.0
    _2_socket_1.subtype = "NONE"
    _2_socket_1.attribute_domain = "POINT"
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
    seed_socket_1 = random_x4___mat.interface.new_socket(
        name="Seed", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    seed_socket_1.default_value = 0.0
    seed_socket_1.subtype = "NONE"
    seed_socket_1.attribute_domain = "POINT"
    group_output_3 = random_x4___mat.nodes.new("NodeGroupOutput")
    group_output_3.is_active_output = True
    group_input_3 = random_x4___mat.nodes.new("NodeGroupInput")
    object_info_1 = random_x4___mat.nodes.new("ShaderNodeObjectInfo")
    math_1 = random_x4___mat.nodes.new("ShaderNodeMath")
    math_1.operation = "ADD"
    math_1.use_clamp = False
    white_noise_texture_1 = random_x4___mat.nodes.new("ShaderNodeTexWhiteNoise")
    white_noise_texture_1.noise_dimensions = "4D"
    separate_color_1 = random_x4___mat.nodes.new("ShaderNodeSeparateColor")
    separate_color_1.mode = "RGB"
    math_001 = random_x4___mat.nodes.new("ShaderNodeMath")
    math_001.operation = "ADD"
    math_001.use_clamp = False
    white_noise_texture_001 = random_x4___mat.nodes.new("ShaderNodeTexWhiteNoise")
    white_noise_texture_001.noise_dimensions = "4D"
    separate_color_001 = random_x4___mat.nodes.new("ShaderNodeSeparateColor")
    separate_color_001.mode = "RGB"
    group_output_3.location = (689.6586303710938, -17.691898345947266)
    group_input_3.location = (-490.65618896484375, 343.00933837890625)
    object_info_1.location = (-490.65618896484375, 63.65891647338867)
    math_1.location = (-280.6562194824219, 343.00933837890625)
    white_noise_texture_1.location = (-70.65621948242188, 343.00933837890625)
    separate_color_1.location = (139.34378051757812, 343.00933837890625)
    math_001.location = (-280.6562194824219, 63.65891647338867)
    white_noise_texture_001.location = (-70.65621948242188, 63.65891647338867)
    separate_color_001.location = (139.34378051757812, 63.65891647338867)
    random_x4___mat.links.new(object_info_1.outputs[5], white_noise_texture_1.inputs[1])
    random_x4___mat.links.new(math_1.outputs[0], white_noise_texture_1.inputs[0])
    random_x4___mat.links.new(
        white_noise_texture_1.outputs[1], separate_color_1.inputs[0]
    )
    random_x4___mat.links.new(object_info_1.outputs[3], math_1.inputs[1])
    random_x4___mat.links.new(group_input_3.outputs[0], math_1.inputs[0])
    random_x4___mat.links.new(separate_color_1.outputs[0], group_output_3.inputs[0])
    random_x4___mat.links.new(separate_color_1.outputs[1], group_output_3.inputs[1])
    random_x4___mat.links.new(math_001.outputs[0], white_noise_texture_001.inputs[0])
    random_x4___mat.links.new(
        white_noise_texture_001.outputs[1], separate_color_001.inputs[0]
    )
    random_x4___mat.links.new(separate_color_1.outputs[2], math_001.inputs[1])
    random_x4___mat.links.new(math_1.outputs[0], math_001.inputs[0])
    random_x4___mat.links.new(separate_color_001.outputs[0], group_output_3.inputs[2])
    random_x4___mat.links.new(separate_color_001.outputs[1], group_output_3.inputs[3])
    random_x4___mat.links.new(
        object_info_1.outputs[5], white_noise_texture_001.inputs[1]
    )
    random_x4___mat.links.new(separate_color_001.outputs[2], group_output_3.inputs[4])
    return random_x4___mat


random_x4___mat = random_x4___mat_node_group()


def rockshader_node_group():
    rockshader = bpy.data.node_groups.new(type="ShaderNodeTree", name="RockShader")
    rockshader.color_tag = "NONE"
    bsdf_socket = rockshader.interface.new_socket(
        name="BSDF", in_out="OUTPUT", socket_type="NodeSocketShader"
    )
    bsdf_socket.attribute_domain = "POINT"
    scale_socket_2 = rockshader.interface.new_socket(
        name="Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    scale_socket_2.default_value = 1.0
    scale_socket_2.subtype = "NONE"
    scale_socket_2.attribute_domain = "POINT"
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
    noise_scale_socket_2 = rockshader.interface.new_socket(
        name="Noise Scale", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    noise_scale_socket_2.default_value = 12.799999237060547
    noise_scale_socket_2.subtype = "NONE"
    noise_scale_socket_2.attribute_domain = "POINT"
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
    group_output_4 = rockshader.nodes.new("NodeGroupOutput")
    group_output_4.is_active_output = True
    group_input_4 = rockshader.nodes.new("NodeGroupInput")
    noise_texture_2 = rockshader.nodes.new("ShaderNodeTexNoise")
    noise_texture_2.noise_dimensions = "4D"
    noise_texture_2.noise_type = "FBM"
    noise_texture_2.normalize = True
    noise_texture_2.inputs[5].default_value = 20.0
    noise_texture_2.inputs[8].default_value = 0.0
    mapping_001_2 = rockshader.nodes.new("ShaderNodeMapping")
    mapping_001_2.vector_type = "POINT"
    mapping_001_2.inputs[2].default_value = (0.0, 0.0, 0.0)
    texture_coordinate_001 = rockshader.nodes.new("ShaderNodeTexCoord")
    texture_coordinate_001.from_instancer = False
    texture_coordinate_001.outputs[0].hide = True
    texture_coordinate_001.outputs[1].hide = True
    texture_coordinate_001.outputs[2].hide = True
    texture_coordinate_001.outputs[4].hide = True
    texture_coordinate_001.outputs[5].hide = True
    texture_coordinate_001.outputs[6].hide = True
    bump_2 = rockshader.nodes.new("ShaderNodeBump")
    bump_2.invert = False
    bump_2.inputs[1].default_value = 1.0
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
    noise_texture_001_1 = rockshader.nodes.new("ShaderNodeTexNoise")
    noise_texture_001_1.noise_dimensions = "4D"
    noise_texture_001_1.noise_type = "FBM"
    noise_texture_001_1.normalize = True
    noise_texture_001_1.inputs[5].default_value = 2.0
    noise_texture_001_1.inputs[8].default_value = 0.0
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
    mix_2 = rockshader.nodes.new("ShaderNodeMix")
    mix_2.blend_type = "MIX"
    mix_2.clamp_factor = True
    mix_2.clamp_result = False
    mix_2.data_type = "RGBA"
    mix_2.factor_mode = "UNIFORM"
    mix_001_2 = rockshader.nodes.new("ShaderNodeMix")
    mix_001_2.blend_type = "MIX"
    mix_001_2.clamp_factor = True
    mix_001_2.clamp_result = False
    mix_001_2.data_type = "RGBA"
    mix_001_2.factor_mode = "UNIFORM"
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
    mix_003_2 = rockshader.nodes.new("ShaderNodeMix")
    mix_003_2.blend_type = "MIX"
    mix_003_2.clamp_factor = True
    mix_003_2.clamp_result = False
    mix_003_2.data_type = "RGBA"
    mix_003_2.factor_mode = "UNIFORM"
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
    bump_001_2 = rockshader.nodes.new("ShaderNodeBump")
    bump_001_2.invert = False
    bump_001_2.inputs[1].default_value = 1.0
    frame_001 = rockshader.nodes.new("NodeFrame")
    frame_001.shrink = True
    frame_002 = rockshader.nodes.new("NodeFrame")
    frame_002.shrink = True
    frame = rockshader.nodes.new("NodeFrame")
    frame.shrink = True
    hue_saturation_value_2 = rockshader.nodes.new("ShaderNodeHueSaturation")
    hue_saturation_value_2.inputs[0].default_value = 0.5
    hue_saturation_value_2.inputs[1].default_value = 1.0
    hue_saturation_value_2.inputs[3].default_value = 1.0
    frame_003 = rockshader.nodes.new("NodeFrame")
    frame_003.shrink = True
    principled_bsdf_2 = rockshader.nodes.new("ShaderNodeBsdfPrincipled")
    principled_bsdf_2.distribution = "MULTI_GGX"
    principled_bsdf_2.subsurface_method = "RANDOM_WALK"
    principled_bsdf_2.inputs[1].default_value = 0.0
    principled_bsdf_2.inputs[3].default_value = 1.5
    principled_bsdf_2.inputs[4].default_value = 1.0
    principled_bsdf_2.inputs[7].default_value = 0.0
    principled_bsdf_2.inputs[8].default_value = (
        1.0,
        0.20000000298023224,
        0.10000000149011612,
    )
    principled_bsdf_2.inputs[9].default_value = 0.05000000074505806
    principled_bsdf_2.inputs[11].default_value = 0.0
    principled_bsdf_2.inputs[12].default_value = 0.5
    principled_bsdf_2.inputs[13].default_value = (1.0, 1.0, 1.0, 1.0)
    principled_bsdf_2.inputs[14].default_value = 0.0
    principled_bsdf_2.inputs[15].default_value = 0.0
    principled_bsdf_2.inputs[16].default_value = (0.0, 0.0, 0.0)
    principled_bsdf_2.inputs[17].default_value = 0.0
    principled_bsdf_2.inputs[18].default_value = 0.0
    principled_bsdf_2.inputs[19].default_value = 0.029999999329447746
    principled_bsdf_2.inputs[20].default_value = 1.5
    principled_bsdf_2.inputs[21].default_value = (1.0, 1.0, 1.0, 1.0)
    principled_bsdf_2.inputs[22].default_value = (0.0, 0.0, 0.0)
    principled_bsdf_2.inputs[23].default_value = 0.0
    principled_bsdf_2.inputs[24].default_value = 0.5
    principled_bsdf_2.inputs[25].default_value = (1.0, 1.0, 1.0, 1.0)
    principled_bsdf_2.inputs[26].default_value = (1.0, 1.0, 1.0, 1.0)
    principled_bsdf_2.inputs[27].default_value = 0.0
    principled_bsdf_2.inputs[28].default_value = 0.0
    principled_bsdf_2.inputs[29].default_value = 1.3300000429153442
    math_2 = rockshader.nodes.new("ShaderNodeMath")
    math_2.operation = "MULTIPLY"
    math_2.use_clamp = False
    math_2.inputs[1].default_value = 10.0
    group_001_2 = rockshader.nodes.new("ShaderNodeGroup")
    group_001_2.node_tree = random_x4___mat
    group_001_2.inputs[0].default_value = 0.5213124752044678
    voronoi_texture_2 = rockshader.nodes.new("ShaderNodeTexVoronoi")
    voronoi_texture_2.distance = "EUCLIDEAN"
    voronoi_texture_2.feature = "F1"
    voronoi_texture_2.normalize = True
    voronoi_texture_2.voronoi_dimensions = "4D"
    voronoi_texture_2.inputs[3].default_value = 0.0
    voronoi_texture_2.inputs[4].default_value = 1.0
    voronoi_texture_2.inputs[5].default_value = 2.0
    voronoi_texture_2.inputs[8].default_value = 1.0
    bump_002_1 = rockshader.nodes.new("ShaderNodeBump")
    bump_002_1.invert = False
    bump_002_1.inputs[1].default_value = 1.0
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
    voronoi_texture_001_1 = rockshader.nodes.new("ShaderNodeTexVoronoi")
    voronoi_texture_001_1.distance = "EUCLIDEAN"
    voronoi_texture_001_1.feature = "SMOOTH_F1"
    voronoi_texture_001_1.normalize = True
    voronoi_texture_001_1.voronoi_dimensions = "4D"
    voronoi_texture_001_1.inputs[3].default_value = 0.0
    voronoi_texture_001_1.inputs[4].default_value = 1.0
    voronoi_texture_001_1.inputs[5].default_value = 2.0
    voronoi_texture_001_1.inputs[6].default_value = 1.0
    voronoi_texture_001_1.inputs[8].default_value = 1.0
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
    noise_texture_2.parent = frame
    color_ramp.parent = frame
    noise_texture_001_1.parent = frame
    color_ramp_001.parent = frame
    mix_2.parent = frame
    mix_001_2.parent = frame_002
    geometry.parent = frame_001
    color_ramp_002.parent = frame_001
    mix_003_2.parent = frame_002
    color_ramp_004.parent = frame_003
    hue_saturation_value_2.parent = frame_003
    group_output_4.location = (2044.083740234375, -366.00262451171875)
    group_input_4.location = (-1756.011962890625, -822.6982421875)
    noise_texture_2.location = (-3084.742431640625, 781.9205322265625)
    mapping_001_2.location = (-1281.65478515625, -227.8770751953125)
    texture_coordinate_001.location = (-1471.65478515625, -236.3770751953125)
    bump_2.location = (1154.56298828125, -790.7999267578125)
    color_ramp.location = (-2845.918701171875, 769.3270263671875)
    noise_texture_001_1.location = (-3091.82958984375, 348.28857421875)
    color_ramp_001.location = (-2840.2607421875, 369.6982421875)
    mix_2.location = (-2463.6015625, 642.8758544921875)
    mix_001_2.location = (-1338.03955078125, 856.2105102539062)
    geometry.location = (-1595.263427734375, 1377.4110107421875)
    color_ramp_002.location = (-1332.5478515625, 1497.3221435546875)
    mix_003_2.location = (-1139.2666015625, 857.9177856445312)
    color_ramp_004.location = (-1898.9849853515625, 572.5324096679688)
    noise_texture_003.location = (233.37887573242188, -895.6905517578125)
    bump_001_2.location = (1390.9708251953125, -663.4024658203125)
    frame_001.location = (1076.4444580078125, -1275.853271484375)
    frame_002.location = (1587.0386962890625, -923.2500610351562)
    frame.location = (2204.56005859375, -1019.8477783203125)
    hue_saturation_value_2.location = (-1571.6060791015625, 569.7412719726562)
    frame_003.location = (2145.8759765625, -1014.9539794921875)
    principled_bsdf_2.location = (1568.39306640625, -416.8108215332031)
    math_2.location = (-1059.811279296875, -390.11346435546875)
    group_001_2.location = (-2127.677001953125, -45.7719612121582)
    voronoi_texture_2.location = (201.54551696777344, -1322.15673828125)
    bump_002_1.location = (925.5811157226562, -915.0869750976562)
    color_ramp_005.location = (387.2950439453125, -1225.90478515625)
    voronoi_texture_001_1.location = (209.61325073242188, -1741.732666015625)
    color_ramp_006.location = (464.92108154296875, -1571.82275390625)
    math_001_1.location = (-162.15603637695312, -1974.9114990234375)
    bump_003.location = (761.9248046875, -1172.5350341796875)
    map_range_004.location = (-1697.904541015625, -193.53184509277344)
    group_002.location = (-1084.7215576171875, -1829.677734375)
    math_002.location = (-578.4093627929688, -1308.6357421875)
    math_003.location = (-452.7193603515625, -1984.625732421875)
    math_004.location = (-351.4325866699219, -1473.386962890625)
    rockshader.links.new(mapping_001_2.outputs[0], noise_texture_001_1.inputs[0])
    rockshader.links.new(noise_texture_001_1.outputs[0], color_ramp_001.inputs[0])
    rockshader.links.new(color_ramp_001.outputs[0], mix_2.inputs[7])
    rockshader.links.new(color_ramp_004.outputs[0], hue_saturation_value_2.inputs[4])
    rockshader.links.new(mix_001_2.outputs[2], mix_003_2.inputs[6])
    rockshader.links.new(mix_003_2.outputs[2], principled_bsdf_2.inputs[0])
    rockshader.links.new(color_ramp_002.outputs[0], mix_003_2.inputs[0])
    rockshader.links.new(hue_saturation_value_2.outputs[0], principled_bsdf_2.inputs[2])
    rockshader.links.new(color_ramp.outputs[0], mix_2.inputs[6])
    rockshader.links.new(mix_2.outputs[2], color_ramp_004.inputs[0])
    rockshader.links.new(mapping_001_2.outputs[0], noise_texture_003.inputs[0])
    rockshader.links.new(bump_2.outputs[0], bump_001_2.inputs[3])
    rockshader.links.new(mix_2.outputs[2], mix_001_2.inputs[0])
    rockshader.links.new(mapping_001_2.outputs[0], noise_texture_2.inputs[0])
    rockshader.links.new(geometry.outputs[7], color_ramp_002.inputs[0])
    rockshader.links.new(mix_2.outputs[2], bump_001_2.inputs[2])
    rockshader.links.new(noise_texture_2.outputs[0], color_ramp.inputs[0])
    rockshader.links.new(texture_coordinate_001.outputs[3], mapping_001_2.inputs[0])
    rockshader.links.new(principled_bsdf_2.outputs[0], group_output_4.inputs[0])
    rockshader.links.new(group_input_4.outputs[0], mapping_001_2.inputs[3])
    rockshader.links.new(group_input_4.outputs[1], mix_001_2.inputs[6])
    rockshader.links.new(group_input_4.outputs[2], mix_001_2.inputs[7])
    rockshader.links.new(group_input_4.outputs[3], mix_003_2.inputs[7])
    rockshader.links.new(group_input_4.outputs[5], noise_texture_2.inputs[3])
    rockshader.links.new(group_input_4.outputs[6], noise_texture_2.inputs[4])
    rockshader.links.new(group_input_4.outputs[5], noise_texture_001_1.inputs[3])
    rockshader.links.new(group_input_4.outputs[6], noise_texture_001_1.inputs[4])
    rockshader.links.new(group_input_4.outputs[9], hue_saturation_value_2.inputs[2])
    rockshader.links.new(group_input_4.outputs[11], bump_2.inputs[0])
    rockshader.links.new(group_input_4.outputs[10], noise_texture_003.inputs[2])
    rockshader.links.new(group_input_4.outputs[12], bump_001_2.inputs[0])
    rockshader.links.new(group_input_4.outputs[4], noise_texture_001_1.inputs[2])
    rockshader.links.new(group_input_4.outputs[14], mix_2.inputs[0])
    rockshader.links.new(group_input_4.outputs[4], math_2.inputs[0])
    rockshader.links.new(math_2.outputs[0], noise_texture_2.inputs[2])
    rockshader.links.new(group_input_4.outputs[15], noise_texture_003.inputs[4])
    rockshader.links.new(group_001_2.outputs[4], noise_texture_001_1.inputs[1])
    rockshader.links.new(group_001_2.outputs[3], noise_texture_2.inputs[1])
    rockshader.links.new(group_001_2.outputs[1], noise_texture_003.inputs[1])
    rockshader.links.new(bump_001_2.outputs[0], principled_bsdf_2.inputs[5])
    rockshader.links.new(noise_texture_003.outputs[0], bump_2.inputs[2])
    rockshader.links.new(mapping_001_2.outputs[0], voronoi_texture_2.inputs[0])
    rockshader.links.new(group_001_2.outputs[1], voronoi_texture_2.inputs[1])
    rockshader.links.new(color_ramp_005.outputs[0], bump_002_1.inputs[2])
    rockshader.links.new(bump_002_1.outputs[0], bump_2.inputs[3])
    rockshader.links.new(voronoi_texture_2.outputs[0], color_ramp_005.inputs[0])
    rockshader.links.new(group_input_4.outputs[16], voronoi_texture_2.inputs[2])
    rockshader.links.new(mapping_001_2.outputs[0], voronoi_texture_001_1.inputs[0])
    rockshader.links.new(group_001_2.outputs[1], voronoi_texture_001_1.inputs[1])
    rockshader.links.new(math_001_1.outputs[0], voronoi_texture_001_1.inputs[2])
    rockshader.links.new(voronoi_texture_001_1.outputs[0], color_ramp_006.inputs[0])
    rockshader.links.new(group_input_4.outputs[16], math_001_1.inputs[0])
    rockshader.links.new(color_ramp_006.outputs[0], bump_003.inputs[2])
    rockshader.links.new(bump_003.outputs[0], bump_002_1.inputs[3])
    rockshader.links.new(map_range_004.outputs[0], mapping_001_2.inputs[1])
    rockshader.links.new(group_001_2.outputs[0], map_range_004.inputs[0])
    rockshader.links.new(group_002.outputs[0], math_002.inputs[1])
    rockshader.links.new(group_input_4.outputs[17], math_002.inputs[0])
    rockshader.links.new(math_002.outputs[0], bump_003.inputs[0])
    rockshader.links.new(group_001_2.outputs[2], group_002.inputs[0])
    rockshader.links.new(math_003.outputs[0], math_001_1.inputs[1])
    rockshader.links.new(group_002.outputs[1], math_003.inputs[0])
    rockshader.links.new(group_input_4.outputs[17], math_004.inputs[0])
    rockshader.links.new(group_002.outputs[2], math_004.inputs[1])
    rockshader.links.new(math_004.outputs[0], bump_002_1.inputs[0])
    return rockshader


rockshader = rockshader_node_group()


def martianrockshader_node_group():
    martianrockshader = bpy.data.node_groups.new(
        type="ShaderNodeTree", name="MartianRockShader"
    )
    martianrockshader.color_tag = "NONE"
    shader_socket_2 = martianrockshader.interface.new_socket(
        name="Shader", in_out="OUTPUT", socket_type="NodeSocketShader"
    )
    shader_socket_2.attribute_domain = "POINT"
    group_output_5 = martianrockshader.nodes.new("NodeGroupOutput")
    group_output_5.is_active_output = True
    group_005 = martianrockshader.nodes.new("ShaderNodeGroup")
    group_005.node_tree = rockshader___3
    group_005.inputs[0].default_value = 1.0
    group_005.inputs[5].default_value = 17.600000381469727
    group_005.inputs[6].default_value = 2.7400002479553223
    group_005.inputs[7].default_value = 15.0
    group_005.inputs[8].default_value = 0.6979339122772217
    group_005.inputs[9].default_value = 0.3623966872692108
    group_005.inputs[10].default_value = 2.0
    group_005.inputs[11].default_value = 0.43471091985702515
    group_005.inputs[12].default_value = 0.2264467179775238
    group_006 = martianrockshader.nodes.new("ShaderNodeGroup")
    group_006.node_tree = rockshader___4
    group_006.inputs[0].default_value = 1.0
    group_006.inputs[4].default_value = 18.68000030517578
    group_006.inputs[5].default_value = 3.840001106262207
    group_006.inputs[6].default_value = 4.180000305175781
    group_006.inputs[7].default_value = 0.010000094771385193
    group_006.inputs[8].default_value = 15.0
    group_006.inputs[9].default_value = 2.0
    group_006.inputs[10].default_value = 0.3370434641838074
    group_006.inputs[11].default_value = 0.931240439414978
    group_006.inputs[12].default_value = 0.19363099336624146
    combine_color_001 = martianrockshader.nodes.new("ShaderNodeCombineColor")
    combine_color_001.mode = "HSV"
    combine_color_001.inputs[0].default_value = 0.029999999329447746
    combine_color_001.inputs[1].default_value = 0.8999999761581421
    map_range_005 = martianrockshader.nodes.new("ShaderNodeMapRange")
    map_range_005.clamp = True
    map_range_005.data_type = "FLOAT"
    map_range_005.interpolation_type = "LINEAR"
    map_range_005.inputs[1].default_value = 0.0
    map_range_005.inputs[2].default_value = 1.0
    map_range_005.inputs[3].default_value = 0.05000000074505806
    map_range_005.inputs[4].default_value = 0.30000001192092896
    combine_color_002 = martianrockshader.nodes.new("ShaderNodeCombineColor")
    combine_color_002.mode = "HSV"
    combine_color_002.inputs[0].default_value = 0.021490026265382767
    combine_color_002.inputs[1].default_value = 0.800000011920929
    map_range_006 = martianrockshader.nodes.new("ShaderNodeMapRange")
    map_range_006.clamp = True
    map_range_006.data_type = "FLOAT"
    map_range_006.interpolation_type = "LINEAR"
    map_range_006.inputs[1].default_value = 0.0
    map_range_006.inputs[2].default_value = 1.0
    map_range_006.inputs[3].default_value = 0.009999999776482582
    map_range_006.inputs[4].default_value = 0.019999999552965164
    group_008 = martianrockshader.nodes.new("ShaderNodeGroup")
    group_008.node_tree = rockshader
    group_008.inputs[0].default_value = 1.5
    group_008.inputs[4].default_value = 11.029998779296875
    group_008.inputs[5].default_value = 15.0
    group_008.inputs[6].default_value = 0.6499999761581421
    group_008.inputs[7].default_value = 8.320000648498535
    group_008.inputs[8].default_value = 0.7736417055130005
    group_008.inputs[9].default_value = 2.0
    group_008.inputs[10].default_value = 17.369998931884766
    group_008.inputs[11].default_value = 0.16358695924282074
    group_008.inputs[12].default_value = 0.3608698546886444
    group_008.inputs[13].default_value = 0.5271739959716797
    group_008.inputs[15].default_value = 0.2190222144126892
    group_008.inputs[16].default_value = 15.0
    group_008.inputs[17].default_value = 0.2663043737411499
    mix_shader_001 = martianrockshader.nodes.new("ShaderNodeMixShader")
    mix_shader_002 = martianrockshader.nodes.new("ShaderNodeMixShader")
    mapping_001_3 = martianrockshader.nodes.new("ShaderNodeMapping")
    mapping_001_3.vector_type = "POINT"
    mapping_001_3.inputs[2].default_value = (0.0, 0.0, 0.0)
    mapping_001_3.inputs[3].default_value = (1.0, 1.0, 1.0)
    texture_coordinate_001_1 = martianrockshader.nodes.new("ShaderNodeTexCoord")
    texture_coordinate_001_1.from_instancer = False
    texture_coordinate_001_1.outputs[0].hide = True
    texture_coordinate_001_1.outputs[1].hide = True
    texture_coordinate_001_1.outputs[2].hide = True
    texture_coordinate_001_1.outputs[4].hide = True
    texture_coordinate_001_1.outputs[5].hide = True
    texture_coordinate_001_1.outputs[6].hide = True
    noise_texture_003_1 = martianrockshader.nodes.new("ShaderNodeTexNoise")
    noise_texture_003_1.noise_dimensions = "3D"
    noise_texture_003_1.noise_type = "HETERO_TERRAIN"
    noise_texture_003_1.normalize = True
    noise_texture_003_1.inputs[3].default_value = 15.0
    noise_texture_003_1.inputs[4].default_value = 0.5166667103767395
    noise_texture_003_1.inputs[5].default_value = 15.179998397827148
    noise_texture_003_1.inputs[6].default_value = 0.14000000059604645
    noise_texture_003_1.inputs[8].default_value = 0.0
    color_ramp_004_1 = martianrockshader.nodes.new("ShaderNodeValToRGB")
    color_ramp_004_1.color_ramp.color_mode = "RGB"
    color_ramp_004_1.color_ramp.hue_interpolation = "NEAR"
    color_ramp_004_1.color_ramp.interpolation = "EASE"
    color_ramp_004_1.color_ramp.elements.remove(color_ramp_004_1.color_ramp.elements[0])
    color_ramp_004_1_cre_0 = color_ramp_004_1.color_ramp.elements[0]
    color_ramp_004_1_cre_0.position = 0.18636341392993927
    color_ramp_004_1_cre_0.alpha = 1.0
    color_ramp_004_1_cre_0.color = (0.0, 0.0, 0.0, 1.0)
    color_ramp_004_1_cre_1 = color_ramp_004_1.color_ramp.elements.new(
        0.9186362028121948
    )
    color_ramp_004_1_cre_1.alpha = 1.0
    color_ramp_004_1_cre_1.color = (1.0, 1.0, 1.0, 1.0)
    map_range_002 = martianrockshader.nodes.new("ShaderNodeMapRange")
    map_range_002.clamp = True
    map_range_002.data_type = "FLOAT"
    map_range_002.interpolation_type = "LINEAR"
    map_range_002.inputs[1].default_value = 0.0
    map_range_002.inputs[2].default_value = 1.0
    map_range_002.inputs[3].default_value = 0.020000003278255463
    map_range_002.inputs[4].default_value = 0.08000001311302185
    noise_texture_005 = martianrockshader.nodes.new("ShaderNodeTexNoise")
    noise_texture_005.noise_dimensions = "3D"
    noise_texture_005.noise_type = "FBM"
    noise_texture_005.normalize = True
    noise_texture_005.inputs[3].default_value = 5.0
    noise_texture_005.inputs[4].default_value = 0.6670835614204407
    noise_texture_005.inputs[5].default_value = 5.0
    noise_texture_005.inputs[8].default_value = 0.0
    color_ramp_006_1 = martianrockshader.nodes.new("ShaderNodeValToRGB")
    color_ramp_006_1.color_ramp.color_mode = "RGB"
    color_ramp_006_1.color_ramp.hue_interpolation = "NEAR"
    color_ramp_006_1.color_ramp.interpolation = "EASE"
    color_ramp_006_1.color_ramp.elements.remove(color_ramp_006_1.color_ramp.elements[0])
    color_ramp_006_1_cre_0 = color_ramp_006_1.color_ramp.elements[0]
    color_ramp_006_1_cre_0.position = 0.5681818127632141
    color_ramp_006_1_cre_0.alpha = 1.0
    color_ramp_006_1_cre_0.color = (0.0, 0.0, 0.0, 1.0)
    color_ramp_006_1_cre_1 = color_ramp_006_1.color_ramp.elements.new(
        0.7000001072883606
    )
    color_ramp_006_1_cre_1.alpha = 1.0
    color_ramp_006_1_cre_1.color = (1.0, 1.0, 1.0, 1.0)
    map_range_007 = martianrockshader.nodes.new("ShaderNodeMapRange")
    map_range_007.clamp = True
    map_range_007.data_type = "FLOAT"
    map_range_007.interpolation_type = "LINEAR"
    map_range_007.inputs[1].default_value = 0.0
    map_range_007.inputs[2].default_value = 1.0
    map_range_007.inputs[3].default_value = 0.02500000037252903
    map_range_007.inputs[4].default_value = 0.07500000298023224
    map_range_015 = martianrockshader.nodes.new("ShaderNodeMapRange")
    map_range_015.clamp = True
    map_range_015.data_type = "FLOAT"
    map_range_015.interpolation_type = "LINEAR"
    map_range_015.inputs[1].default_value = 0.0
    map_range_015.inputs[2].default_value = 1.0
    map_range_015.inputs[3].default_value = -1000.0
    map_range_015.inputs[4].default_value = 1000.0
    group = martianrockshader.nodes.new("ShaderNodeGroup")
    group.node_tree = random_x4___mat
    group.inputs[0].default_value = 0.521340012550354
    combine_color_003 = martianrockshader.nodes.new("ShaderNodeCombineColor")
    combine_color_003.mode = "HSV"
    combine_color_003.inputs[0].default_value = 0.033332519233226776
    combine_color_003.inputs[1].default_value = 0.8999999761581421
    map_range_008 = martianrockshader.nodes.new("ShaderNodeMapRange")
    map_range_008.clamp = True
    map_range_008.data_type = "FLOAT"
    map_range_008.interpolation_type = "LINEAR"
    map_range_008.inputs[0].default_value = 1.0
    map_range_008.inputs[1].default_value = 0.0
    map_range_008.inputs[2].default_value = 1.0
    map_range_008.inputs[3].default_value = 0.2900000214576721
    map_range_008.inputs[4].default_value = 0.7899999618530273
    combine_color_006 = martianrockshader.nodes.new("ShaderNodeCombineColor")
    combine_color_006.mode = "HSV"
    combine_color_006.inputs[0].default_value = 0.03500000014901161
    combine_color_006.inputs[1].default_value = 0.08500000089406967
    map_range_011_2 = martianrockshader.nodes.new("ShaderNodeMapRange")
    map_range_011_2.clamp = True
    map_range_011_2.data_type = "FLOAT"
    map_range_011_2.interpolation_type = "LINEAR"
    map_range_011_2.inputs[1].default_value = 0.0
    map_range_011_2.inputs[2].default_value = 1.0
    map_range_011_2.inputs[3].default_value = 0.0
    map_range_011_2.inputs[4].default_value = 0.029999999329447746
    group_output_5.location = (1658.8565673828125, -863.8759765625)
    group_005.location = (763.2269287109375, -820.1624755859375)
    group_006.location = (734.3885498046875, -1368.90478515625)
    combine_color_001.location = (163.7159423828125, -380.75921630859375)
    map_range_005.location = (-26.2840576171875, -331.75921630859375)
    combine_color_002.location = (163.7159423828125, -743.6762084960938)
    map_range_006.location = (-26.2840576171875, -694.6762084960938)
    group_008.location = (694.5396118164062, -193.486328125)
    mix_shader_001.location = (1074.927001953125, -875.521240234375)
    mix_shader_002.location = (1273.0546875, -662.526611328125)
    mapping_001_3.location = (-378.397705078125, 1248.543212890625)
    texture_coordinate_001_1.location = (-633.934814453125, 1283.7763671875)
    noise_texture_003_1.location = (69.0587158203125, 770.0805053710938)
    color_ramp_004_1.location = (272.8131103515625, 795.0889282226562)
    map_range_002.location = (-183.6375732421875, 819.4551391601562)
    noise_texture_005.location = (69.0587158203125, 259.23516845703125)
    color_ramp_006_1.location = (278.5709228515625, 285.52447509765625)
    map_range_007.location = (-183.6375732421875, 308.60980224609375)
    map_range_015.location = (-601.584228515625, 1054.19091796875)
    group.location = (-1002.6082763671875, -191.56527709960938)
    combine_color_003.location = (-471.6387939453125, -544.2174682617188)
    map_range_008.location = (-686.9805908203125, -567.8173828125)
    combine_color_006.location = (185.722900390625, -1101.7760009765625)
    map_range_011_2.location = (-31.125974655151367, -1067.997802734375)
    martianrockshader.links.new(combine_color_002.outputs[0], group_006.inputs[2])
    martianrockshader.links.new(group.outputs[0], map_range_015.inputs[0])
    martianrockshader.links.new(group.outputs[2], map_range_002.inputs[0])
    martianrockshader.links.new(combine_color_001.outputs[0], group_005.inputs[1])
    martianrockshader.links.new(group_006.outputs[0], mix_shader_001.inputs[2])
    martianrockshader.links.new(group.outputs[2], map_range_007.inputs[0])
    martianrockshader.links.new(group_005.outputs[0], mix_shader_001.inputs[1])
    martianrockshader.links.new(color_ramp_004_1.outputs[0], mix_shader_001.inputs[0])
    martianrockshader.links.new(mix_shader_001.outputs[0], mix_shader_002.inputs[2])
    martianrockshader.links.new(color_ramp_006_1.outputs[0], mix_shader_002.inputs[0])
    martianrockshader.links.new(
        texture_coordinate_001_1.outputs[3], mapping_001_3.inputs[0]
    )
    martianrockshader.links.new(map_range_005.outputs[0], combine_color_001.inputs[2])
    martianrockshader.links.new(combine_color_001.outputs[0], group_008.inputs[3])
    martianrockshader.links.new(map_range_006.outputs[0], combine_color_002.inputs[2])
    martianrockshader.links.new(combine_color_002.outputs[0], group_008.inputs[1])
    martianrockshader.links.new(
        noise_texture_003_1.outputs[0], color_ramp_004_1.inputs[0]
    )
    martianrockshader.links.new(mapping_001_3.outputs[0], noise_texture_003_1.inputs[0])
    martianrockshader.links.new(map_range_002.outputs[0], noise_texture_003_1.inputs[2])
    martianrockshader.links.new(group.outputs[1], map_range_005.inputs[0])
    martianrockshader.links.new(map_range_015.outputs[0], mapping_001_3.inputs[1])
    martianrockshader.links.new(
        noise_texture_005.outputs[0], color_ramp_006_1.inputs[0]
    )
    martianrockshader.links.new(combine_color_002.outputs[0], group_005.inputs[4])
    martianrockshader.links.new(mapping_001_3.outputs[0], noise_texture_005.inputs[0])
    martianrockshader.links.new(map_range_007.outputs[0], noise_texture_005.inputs[2])
    martianrockshader.links.new(combine_color_001.outputs[0], group_006.inputs[3])
    martianrockshader.links.new(mix_shader_002.outputs[0], group_output_5.inputs[0])
    martianrockshader.links.new(group_008.outputs[0], mix_shader_002.inputs[1])
    martianrockshader.links.new(group.outputs[0], map_range_006.inputs[0])
    martianrockshader.links.new(group.outputs[3], group_008.inputs[14])
    martianrockshader.links.new(combine_color_006.outputs[0], group_008.inputs[2])
    martianrockshader.links.new(combine_color_006.outputs[0], group_005.inputs[2])
    martianrockshader.links.new(combine_color_006.outputs[0], group_006.inputs[1])
    martianrockshader.links.new(combine_color_006.outputs[0], group_005.inputs[3])
    martianrockshader.links.new(map_range_008.outputs[0], combine_color_003.inputs[2])
    martianrockshader.links.new(map_range_011_2.outputs[0], combine_color_006.inputs[2])
    martianrockshader.links.new(group.outputs[4], map_range_011_2.inputs[0])
    return martianrockshader


martianrockshader = martianrockshader_node_group()


def martianrock_node_group():
    martianrock = mat.node_tree
    for node in martianrock.nodes:
        martianrock.nodes.remove(node)
    martianrock.color_tag = "NONE"
    material_output = martianrock.nodes.new("ShaderNodeOutputMaterial")
    material_output.is_active_output = True
    material_output.target = "ALL"
    material_output.inputs[2].default_value = (0.0, 0.0, 0.0)
    material_output.inputs[3].default_value = 0.0
    group_003 = martianrock.nodes.new("ShaderNodeGroup")
    group_003.node_tree = martianrockshader
    material_output.location = (-100.81094360351562, 3.1049346923828125)
    group_003.location = (-402.0459899902344, -16.092731475830078)
    martianrock.links.new(group_003.outputs[0], material_output.inputs[0])
    return martianrock


martianrock = martianrock_node_group()

import bpy


def random__uniform__node_group():
    random__uniform_ = bpy.data.node_groups.new(
        type="GeometryNodeTree", name="Random (Uniform)"
    )
    random__uniform_.color_tag = "NONE"
    value_socket = random__uniform_.interface.new_socket(
        name="Value", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    value_socket.default_value = 0.0
    value_socket.subtype = "NONE"
    value_socket.attribute_domain = "POINT"
    min_socket = random__uniform_.interface.new_socket(
        name="Min", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    min_socket.default_value = 0.0
    min_socket.subtype = "NONE"
    min_socket.attribute_domain = "POINT"
    max_socket = random__uniform_.interface.new_socket(
        name="Max", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    max_socket.default_value = 1.0
    max_socket.subtype = "NONE"
    max_socket.attribute_domain = "POINT"
    seed_socket = random__uniform_.interface.new_socket(
        name="Seed", in_out="INPUT", socket_type="NodeSocketInt"
    )
    seed_socket.default_value = 0
    seed_socket.subtype = "NONE"
    seed_socket.attribute_domain = "POINT"
    seed_socket.hide_value = True
    offset_socket = random__uniform_.interface.new_socket(
        name="Offset", in_out="INPUT", socket_type="NodeSocketInt"
    )
    offset_socket.default_value = 0
    offset_socket.subtype = "NONE"
    offset_socket.attribute_domain = "POINT"
    group_output = random__uniform_.nodes.new("NodeGroupOutput")
    group_output.is_active_output = True
    group_input = random__uniform_.nodes.new("NodeGroupInput")
    random_value_011 = random__uniform_.nodes.new("FunctionNodeRandomValue")
    random_value_011.data_type = "FLOAT"
    group_output.location = (190.0, 0.0)
    group_input.location = (-200.0, 0.0)
    random_value_011.location = (0.0, 0.0)
    random__uniform_.links.new(random_value_011.outputs[1], group_output.inputs[0])
    random__uniform_.links.new(group_input.outputs[0], random_value_011.inputs[2])
    random__uniform_.links.new(group_input.outputs[1], random_value_011.inputs[3])
    random__uniform_.links.new(group_input.outputs[3], random_value_011.inputs[7])
    random__uniform_.links.new(group_input.outputs[2], random_value_011.inputs[8])
    return random__uniform_


random__uniform_ = random__uniform__node_group()


def random__normal__node_group():
    random__normal_ = bpy.data.node_groups.new(
        type="GeometryNodeTree", name="Random (Normal)"
    )
    random__normal_.color_tag = "NONE"
    value_socket_1 = random__normal_.interface.new_socket(
        name="Value", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    value_socket_1.default_value = 0.0
    value_socket_1.subtype = "NONE"
    value_socket_1.attribute_domain = "POINT"
    non_negative_socket = random__normal_.interface.new_socket(
        name="Non-Negative", in_out="INPUT", socket_type="NodeSocketBool"
    )
    non_negative_socket.default_value = True
    non_negative_socket.attribute_domain = "POINT"
    mean_socket = random__normal_.interface.new_socket(
        name="Mean", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    mean_socket.default_value = 0.0
    mean_socket.subtype = "NONE"
    mean_socket.attribute_domain = "POINT"
    std__dev__socket = random__normal_.interface.new_socket(
        name="Std. Dev.", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    std__dev__socket.default_value = 1.0
    std__dev__socket.subtype = "NONE"
    std__dev__socket.attribute_domain = "POINT"
    seed_socket_1 = random__normal_.interface.new_socket(
        name="Seed", in_out="INPUT", socket_type="NodeSocketInt"
    )
    seed_socket_1.default_value = 0
    seed_socket_1.subtype = "NONE"
    seed_socket_1.attribute_domain = "POINT"
    seed_socket_1.hide_value = True
    offset_socket_1 = random__normal_.interface.new_socket(
        name="Offset", in_out="INPUT", socket_type="NodeSocketInt"
    )
    offset_socket_1.default_value = 0
    offset_socket_1.subtype = "NONE"
    offset_socket_1.attribute_domain = "POINT"
    frame = random__normal_.nodes.new("NodeFrame")
    frame.shrink = True
    frame_003 = random__normal_.nodes.new("NodeFrame")
    frame_003.shrink = True
    frame_001 = random__normal_.nodes.new("NodeFrame")
    frame_001.shrink = True
    math_002 = random__normal_.nodes.new("ShaderNodeMath")
    math_002.operation = "MULTIPLY"
    math_002.use_clamp = False
    math_002.inputs[1].default_value = 6.2831854820251465
    random_value_001 = random__normal_.nodes.new("FunctionNodeRandomValue")
    random_value_001.data_type = "FLOAT"
    random_value_001.inputs[2].default_value = 0.0
    random_value_001.inputs[3].default_value = 1.0
    math_010 = random__normal_.nodes.new("ShaderNodeMath")
    math_010.operation = "ADD"
    math_010.use_clamp = False
    math_010.inputs[1].hide = True
    math_010.inputs[2].hide = True
    math_010.inputs[1].default_value = 1.0
    math_005 = random__normal_.nodes.new("ShaderNodeMath")
    math_005.operation = "MULTIPLY"
    math_005.use_clamp = False
    math_004 = random__normal_.nodes.new("ShaderNodeMath")
    math_004.operation = "COSINE"
    math_004.use_clamp = False
    math_008 = random__normal_.nodes.new("ShaderNodeMath")
    math_008.operation = "MULTIPLY"
    math_008.use_clamp = False
    math_007 = random__normal_.nodes.new("ShaderNodeMath")
    math_007.operation = "ADD"
    math_007.use_clamp = False
    math = random__normal_.nodes.new("ShaderNodeMath")
    math.operation = "LOGARITHM"
    math.use_clamp = False
    math.inputs[1].default_value = 2.7182817459106445
    random_value_002 = random__normal_.nodes.new("FunctionNodeRandomValue")
    random_value_002.data_type = "FLOAT"
    random_value_002.inputs[2].default_value = 0.0
    random_value_002.inputs[3].default_value = 1.0
    math_001 = random__normal_.nodes.new("ShaderNodeMath")
    math_001.operation = "MULTIPLY"
    math_001.use_clamp = False
    math_001.inputs[1].default_value = -2.0
    math_003 = random__normal_.nodes.new("ShaderNodeMath")
    math_003.operation = "SQRT"
    math_003.use_clamp = False
    group_output_1 = random__normal_.nodes.new("NodeGroupOutput")
    group_output_1.is_active_output = True
    group_input_1 = random__normal_.nodes.new("NodeGroupInput")
    switch = random__normal_.nodes.new("GeometryNodeSwitch")
    switch.input_type = "FLOAT"
    math_006 = random__normal_.nodes.new("ShaderNodeMath")
    math_006.operation = "MAXIMUM"
    math_006.use_clamp = False
    math_006.inputs[1].default_value = 0.0
    math_002.parent = frame
    random_value_001.parent = frame
    math_010.parent = frame
    math_005.parent = frame_003
    math_004.parent = frame_003
    math.parent = frame_001
    random_value_002.parent = frame_001
    math_001.parent = frame_001
    math_003.parent = frame_001
    frame.location = (-789.8994140625, -466.3039245605469)
    frame_003.location = (-189.27259826660156, -312.6241455078125)
    frame_001.location = (-1020.868408203125, -235.92041015625)
    math_002.location = (138.8717041015625, -30.349945068359375)
    random_value_001.location = (-51.1282958984375, -20.849945068359375)
    math_010.location = (-241.1282958984375, -137.92230224609375)
    math_005.location = (197.912353515625, -20.78594970703125)
    math_004.location = (7.912353515625, -110.67535400390625)
    math_008.location = (210.5360565185547, -105.03559112548828)
    math_007.location = (400.53607177734375, 29.03577995300293)
    math.location = (177.51742553710938, -9.365585327148438)
    random_value_002.location = (-12.482574462890625, 0.1344146728515625)
    math_001.location = (367.5174255371094, -9.365585327148438)
    math_003.location = (557.5174560546875, -20.365585327148438)
    group_output_1.location = (970.5360717773438, -8.96422004699707)
    group_input_1.location = (-1399.3758544921875, -91.58724975585938)
    switch.location = (780.5360717773438, 26.53577995300293)
    math_006.location = (590.5360717773438, -88.39610290527344)
    random__normal_.links.new(random_value_002.outputs[1], math.inputs[0])
    random__normal_.links.new(math.outputs[0], math_001.inputs[0])
    random__normal_.links.new(random_value_001.outputs[1], math_002.inputs[0])
    random__normal_.links.new(math_002.outputs[0], math_004.inputs[0])
    random__normal_.links.new(math_003.outputs[0], math_005.inputs[0])
    random__normal_.links.new(group_input_1.outputs[3], random_value_002.inputs[8])
    random__normal_.links.new(group_input_1.outputs[3], math_010.inputs[0])
    random__normal_.links.new(math_010.outputs[0], random_value_001.inputs[8])
    random__normal_.links.new(group_input_1.outputs[2], math_008.inputs[0])
    random__normal_.links.new(group_input_1.outputs[1], math_007.inputs[0])
    random__normal_.links.new(math_008.outputs[0], math_007.inputs[1])
    random__normal_.links.new(math_005.outputs[0], math_008.inputs[1])
    random__normal_.links.new(math_004.outputs[0], math_005.inputs[1])
    random__normal_.links.new(math_001.outputs[0], math_003.inputs[0])
    random__normal_.links.new(group_input_1.outputs[4], random_value_001.inputs[7])
    random__normal_.links.new(group_input_1.outputs[4], random_value_002.inputs[7])
    random__normal_.links.new(group_input_1.outputs[0], switch.inputs[0])
    random__normal_.links.new(math_007.outputs[0], math_006.inputs[0])
    random__normal_.links.new(switch.outputs[0], group_output_1.inputs[0])
    random__normal_.links.new(math_007.outputs[0], switch.inputs[1])
    random__normal_.links.new(math_006.outputs[0], switch.inputs[2])
    return random__normal_


random__normal_ = random__normal__node_group()


def lunarrock_node_group():
    lunarrock = bpy.data.node_groups.new(type="GeometryNodeTree", name="LunarRock")
    lunarrock.color_tag = "GEOMETRY"
    lunarrock.is_modifier = True
    geometry_socket = lunarrock.interface.new_socket(
        name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry"
    )
    geometry_socket.attribute_domain = "POINT"
    seed_socket_2 = lunarrock.interface.new_socket(
        name="Seed", in_out="INPUT", socket_type="NodeSocketInt"
    )
    seed_socket_2.default_value = 0
    seed_socket_2.subtype = "NONE"
    seed_socket_2.attribute_domain = "POINT"
    seed_socket_2.force_non_field = True
    subdivisions_socket = lunarrock.interface.new_socket(
        name="Subdivisions", in_out="INPUT", socket_type="NodeSocketInt"
    )
    subdivisions_socket.default_value = 4
    subdivisions_socket.subtype = "NONE"
    subdivisions_socket.attribute_domain = "POINT"
    subdivisions_socket.force_non_field = True
    scale_socket = lunarrock.interface.new_socket(
        name="Scale", in_out="INPUT", socket_type="NodeSocketVector"
    )
    scale_socket.default_value = (1.0, 1.0, 1.0)
    scale_socket.subtype = "XYZ"
    scale_socket.attribute_domain = "POINT"
    scale_socket.force_non_field = True
    scale_std_socket = lunarrock.interface.new_socket(
        name="Scale STD", in_out="INPUT", socket_type="NodeSocketVector"
    )
    scale_std_socket.default_value = (0.0, 0.0, 0.0)
    scale_std_socket.subtype = "XYZ"
    scale_std_socket.attribute_domain = "POINT"
    scale_std_socket.force_non_field = True
    horizontal_cut_socket = lunarrock.interface.new_socket(
        name="Horizontal Cut", in_out="INPUT", socket_type="NodeSocketBool"
    )
    horizontal_cut_socket.default_value = False
    horizontal_cut_socket.attribute_domain = "POINT"
    horizontal_cut_socket.force_non_field = True
    horizontal_cut_offset_socket = lunarrock.interface.new_socket(
        name="Horizontal Cut Offset", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    horizontal_cut_offset_socket.default_value = 0.0
    horizontal_cut_offset_socket.subtype = "DISTANCE"
    horizontal_cut_offset_socket.attribute_domain = "POINT"
    horizontal_cut_offset_socket.force_non_field = True
    group_input_2 = lunarrock.nodes.new("NodeGroupInput")
    group_output_2 = lunarrock.nodes.new("NodeGroupOutput")
    group_output_2.is_active_output = True
    set_material = lunarrock.nodes.new("GeometryNodeSetMaterial")
    set_material.inputs[1].default_value = True
    if "LunarRock" in bpy.data.materials:
        set_material.inputs[2].default_value = bpy.data.materials["LunarRock"]
    cube = lunarrock.nodes.new("GeometryNodeMeshCube")
    cube.inputs[0].default_value = (1.0, 1.0, 1.0)
    cube.inputs[1].default_value = 2
    cube.inputs[2].default_value = 2
    cube.inputs[3].default_value = 2
    subdivision_surface = lunarrock.nodes.new("GeometryNodeSubdivisionSurface")
    subdivision_surface.boundary_smooth = "ALL"
    subdivision_surface.uv_smooth = "PRESERVE_BOUNDARIES"
    set_position = lunarrock.nodes.new("GeometryNodeSetPosition")
    set_position.inputs[1].hide = True
    set_position.inputs[3].hide = True
    set_position.inputs[1].default_value = True
    set_position.inputs[3].default_value = (0.0, 0.0, 0.0)
    voronoi_texture = lunarrock.nodes.new("ShaderNodeTexVoronoi")
    voronoi_texture.distance = "EUCLIDEAN"
    voronoi_texture.feature = "SMOOTH_F1"
    voronoi_texture.normalize = True
    voronoi_texture.voronoi_dimensions = "4D"
    voronoi_texture.inputs[0].default_value = (0.0, 0.0, 0.0)
    voronoi_texture.inputs[6].default_value = 0.0
    voronoi_texture.inputs[8].default_value = 1.0
    vector_math = lunarrock.nodes.new("ShaderNodeVectorMath")
    vector_math.operation = "MULTIPLY"
    position = lunarrock.nodes.new("GeometryNodeInputPosition")
    map_range = lunarrock.nodes.new("ShaderNodeMapRange")
    map_range.clamp = False
    map_range.data_type = "FLOAT"
    map_range.interpolation_type = "LINEAR"
    map_range.inputs[1].default_value = 0.0
    map_range.inputs[2].default_value = 1.0
    map_range.inputs[3].default_value = 0.3333333432674408
    map_range.inputs[4].default_value = 1.0
    set_position_001 = lunarrock.nodes.new("GeometryNodeSetPosition")
    set_position_001.inputs[1].hide = True
    set_position_001.inputs[3].hide = True
    set_position_001.inputs[1].default_value = True
    set_position_001.inputs[3].default_value = (0.0, 0.0, 0.0)
    vector_math_001 = lunarrock.nodes.new("ShaderNodeVectorMath")
    vector_math_001.operation = "MULTIPLY"
    position_001 = lunarrock.nodes.new("GeometryNodeInputPosition")
    noise_texture = lunarrock.nodes.new("ShaderNodeTexNoise")
    noise_texture.noise_dimensions = "4D"
    noise_texture.noise_type = "FBM"
    noise_texture.normalize = True
    noise_texture.inputs[0].default_value = (0.0, 0.0, 0.0)
    noise_texture.inputs[3].default_value = 15.0
    set_shade_smooth = lunarrock.nodes.new("GeometryNodeSetShadeSmooth")
    set_shade_smooth.domain = "FACE"
    set_shade_smooth.inputs[1].default_value = True
    set_shade_smooth.inputs[2].default_value = True
    frame_1 = lunarrock.nodes.new("NodeFrame")
    frame_1.shrink = True
    frame_001_1 = lunarrock.nodes.new("NodeFrame")
    frame_001_1.hide = True
    frame_001_1.shrink = True
    frame_002 = lunarrock.nodes.new("NodeFrame")
    frame_002.shrink = True
    reroute_001 = lunarrock.nodes.new("NodeReroute")
    transform_geometry = lunarrock.nodes.new("GeometryNodeTransform")
    transform_geometry.mode = "COMPONENTS"
    transform_geometry.inputs[2].hide = True
    transform_geometry.inputs[4].hide = True
    transform_geometry.inputs[2].default_value = (0.0, 0.0, 0.0)
    reroute_002 = lunarrock.nodes.new("NodeReroute")
    attribute_statistic = lunarrock.nodes.new("GeometryNodeAttributeStatistic")
    attribute_statistic.data_type = "FLOAT_VECTOR"
    attribute_statistic.domain = "POINT"
    attribute_statistic.inputs[1].hide = True
    attribute_statistic.outputs[0].hide = True
    attribute_statistic.outputs[1].hide = True
    attribute_statistic.outputs[2].hide = True
    attribute_statistic.outputs[6].hide = True
    attribute_statistic.outputs[7].hide = True
    attribute_statistic.inputs[1].default_value = True
    position_002 = lunarrock.nodes.new("GeometryNodeInputPosition")
    reroute_003 = lunarrock.nodes.new("NodeReroute")
    vector_math_002 = lunarrock.nodes.new("ShaderNodeVectorMath")
    vector_math_002.operation = "DIVIDE"
    vector_math_002.inputs[0].default_value = (1.0, 1.0, 1.0)
    vector_math_003 = lunarrock.nodes.new("ShaderNodeVectorMath")
    vector_math_003.operation = "ADD"
    vector_math_004 = lunarrock.nodes.new("ShaderNodeVectorMath")
    vector_math_004.operation = "SCALE"
    vector_math_004.inputs[3].default_value = -0.5
    group = lunarrock.nodes.new("GeometryNodeGroup")
    group.node_tree = random__normal_
    group.inputs[0].default_value = True
    group.inputs[1].default_value = 2.25
    group.inputs[2].default_value = 0.3333333432674408
    group.inputs[4].default_value = 9799
    group_001 = lunarrock.nodes.new("GeometryNodeGroup")
    group_001.node_tree = random__uniform_
    group_001.inputs[0].default_value = -100000000.0
    group_001.inputs[1].default_value = 1000000000.0
    group_001.inputs[3].default_value = 10074
    group_002 = lunarrock.nodes.new("GeometryNodeGroup")
    group_002.node_tree = random__normal_
    group_002.inputs[0].default_value = True
    group_002.inputs[1].default_value = 1.0
    group_002.inputs[2].default_value = 0.25
    group_002.inputs[4].default_value = 8856
    group_004 = lunarrock.nodes.new("GeometryNodeGroup")
    group_004.node_tree = random__normal_
    group_004.inputs[0].default_value = True
    group_004.inputs[1].default_value = 1.25
    group_004.inputs[2].default_value = 0.25
    group_004.inputs[4].default_value = 2182
    float_curve = lunarrock.nodes.new("ShaderNodeFloatCurve")
    float_curve.mapping.extend = "EXTRAPOLATED"
    float_curve.mapping.tone = "STANDARD"
    float_curve.mapping.black_level = (0.0, 0.0, 0.0)
    float_curve.mapping.white_level = (1.0, 1.0, 1.0)
    float_curve.mapping.clip_min_x = 0.0
    float_curve.mapping.clip_min_y = 0.0
    float_curve.mapping.clip_max_x = 1.0
    float_curve.mapping.clip_max_y = 1.0
    float_curve.mapping.use_clip = True
    float_curve_curve_0 = float_curve.mapping.curves[0]
    float_curve_curve_0_point_0 = float_curve_curve_0.points[0]
    float_curve_curve_0_point_0.location = (0.0, 0.0)
    float_curve_curve_0_point_0.handle_type = "AUTO"
    float_curve_curve_0_point_1 = float_curve_curve_0.points[1]
    float_curve_curve_0_point_1.location = (0.3333333432674408, 0.10000000149011612)
    float_curve_curve_0_point_1.handle_type = "AUTO_CLAMPED"
    float_curve_curve_0_point_2 = float_curve_curve_0.points.new(1.0, 1.0)
    float_curve_curve_0_point_2.handle_type = "AUTO"
    float_curve.mapping.update()
    float_curve.inputs[0].default_value = 1.0
    group_005 = lunarrock.nodes.new("GeometryNodeGroup")
    group_005.node_tree = random__normal_
    group_005.inputs[0].default_value = True
    group_005.inputs[1].default_value = 0.25
    group_005.inputs[2].default_value = 0.10000000149011612
    group_005.inputs[4].default_value = 2227
    reroute_005 = lunarrock.nodes.new("NodeReroute")
    group_003 = lunarrock.nodes.new("GeometryNodeGroup")
    group_003.node_tree = random__normal_
    group_003.inputs[0].default_value = True
    group_003.inputs[1].default_value = 0.15000000596046448
    group_003.inputs[2].default_value = 0.02500000037252903
    group_003.inputs[4].default_value = 21973
    group_006 = lunarrock.nodes.new("GeometryNodeGroup")
    group_006.node_tree = random__normal_
    group_006.inputs[0].default_value = True
    group_006.inputs[1].default_value = 0.20000000298023224
    group_006.inputs[2].default_value = 0.05000000074505806
    group_006.inputs[4].default_value = 14855
    reroute_006 = lunarrock.nodes.new("NodeReroute")
    reroute = lunarrock.nodes.new("NodeReroute")
    group_007 = lunarrock.nodes.new("GeometryNodeGroup")
    group_007.node_tree = random__uniform_
    group_007.inputs[0].default_value = -100000000.0
    group_007.inputs[1].default_value = 1000000000.0
    group_007.inputs[3].default_value = 10781
    group_008 = lunarrock.nodes.new("GeometryNodeGroup")
    group_008.node_tree = random__normal_
    group_008.inputs[0].default_value = True
    group_008.inputs[1].default_value = 0.07500000298023224
    group_008.inputs[2].default_value = 0.02500000037252903
    group_008.inputs[4].default_value = 3267
    group_010 = lunarrock.nodes.new("GeometryNodeGroup")
    group_010.node_tree = random__normal_
    group_010.inputs[0].default_value = True
    group_010.inputs[1].default_value = 0.5600000023841858
    group_010.inputs[2].default_value = 0.019999999552965164
    group_010.inputs[4].default_value = 5038
    group_011 = lunarrock.nodes.new("GeometryNodeGroup")
    group_011.node_tree = random__normal_
    group_011.inputs[0].default_value = True
    group_011.inputs[1].default_value = 2.4000000953674316
    group_011.inputs[2].default_value = 0.20000000298023224
    group_011.inputs[4].default_value = 3147
    group_012 = lunarrock.nodes.new("GeometryNodeGroup")
    group_012.node_tree = random__normal_
    group_012.inputs[0].default_value = True
    group_012.inputs[1].default_value = 0.05000000074505806
    group_012.inputs[2].default_value = 0.009999999776482582
    group_012.inputs[4].default_value = 3622
    frame_003_1 = lunarrock.nodes.new("NodeFrame")
    frame_003_1.shrink = True
    transform_geometry_001 = lunarrock.nodes.new("GeometryNodeTransform")
    transform_geometry_001.mode = "COMPONENTS"
    transform_geometry_001.inputs[1].hide = True
    transform_geometry_001.inputs[3].hide = True
    transform_geometry_001.inputs[4].hide = True
    transform_geometry_001.inputs[1].default_value = (0.0, 0.0, 0.0)
    transform_geometry_001.inputs[3].default_value = (1.0, 1.0, 1.0)
    random_value = lunarrock.nodes.new("FunctionNodeRandomValue")
    random_value.data_type = "FLOAT_VECTOR"
    random_value.inputs[0].hide = True
    random_value.inputs[1].hide = True
    random_value.inputs[2].hide = True
    random_value.inputs[3].hide = True
    random_value.inputs[4].hide = True
    random_value.inputs[5].hide = True
    random_value.inputs[6].hide = True
    random_value.outputs[1].hide = True
    random_value.outputs[2].hide = True
    random_value.outputs[3].hide = True
    random_value.inputs[0].default_value = (
        -3.1415927410125732,
        -3.1415927410125732,
        -3.1415927410125732,
    )
    random_value.inputs[1].default_value = (
        3.1415927410125732,
        3.1415927410125732,
        3.1415927410125732,
    )
    integer = lunarrock.nodes.new("FunctionNodeInputInt")
    integer.integer = 424242
    delete_geometry = lunarrock.nodes.new("GeometryNodeDeleteGeometry")
    delete_geometry.domain = "FACE"
    delete_geometry.mode = "ALL"
    compare = lunarrock.nodes.new("FunctionNodeCompare")
    compare.data_type = "FLOAT"
    compare.mode = "ELEMENT"
    compare.operation = "EQUAL"
    compare.inputs[12].default_value = 0.0010000000474974513
    position_004 = lunarrock.nodes.new("GeometryNodeInputPosition")
    separate_xyz_001 = lunarrock.nodes.new("ShaderNodeSeparateXYZ")
    separate_xyz_001.outputs[0].hide = True
    separate_xyz_001.outputs[1].hide = True
    normal_001 = lunarrock.nodes.new("GeometryNodeInputNormal")
    boolean_math = lunarrock.nodes.new("FunctionNodeBooleanMath")
    boolean_math.operation = "AND"
    separate_xyz_002 = lunarrock.nodes.new("ShaderNodeSeparateXYZ")
    separate_xyz_002.outputs[0].hide = True
    separate_xyz_002.outputs[1].hide = True
    compare_001 = lunarrock.nodes.new("FunctionNodeCompare")
    compare_001.data_type = "FLOAT"
    compare_001.mode = "ELEMENT"
    compare_001.operation = "EQUAL"
    compare_001.inputs[1].default_value = -1.0
    compare_001.inputs[12].default_value = 0.0010000000474974513
    mesh_boolean = lunarrock.nodes.new("GeometryNodeMeshBoolean")
    mesh_boolean.operation = "DIFFERENCE"
    mesh_boolean.solver = "FLOAT"
    mesh_boolean.inputs[2].default_value = False
    mesh_boolean.inputs[3].default_value = False
    switch_1 = lunarrock.nodes.new("GeometryNodeSwitch")
    switch_1.input_type = "GEOMETRY"
    transform_geometry_002 = lunarrock.nodes.new("GeometryNodeTransform")
    transform_geometry_002.mode = "COMPONENTS"
    transform_geometry_002.inputs[2].hide = True
    transform_geometry_002.inputs[3].hide = True
    transform_geometry_002.inputs[4].hide = True
    transform_geometry_002.inputs[2].default_value = (0.0, 0.0, 0.0)
    transform_geometry_002.inputs[3].default_value = (1.0, 1.0, 1.0)
    combine_xyz = lunarrock.nodes.new("ShaderNodeCombineXYZ")
    combine_xyz.inputs[0].default_value = 0.0
    combine_xyz.inputs[1].default_value = 0.0
    reroute_010 = lunarrock.nodes.new("NodeReroute")
    cube_001 = lunarrock.nodes.new("GeometryNodeMeshCube")
    cube_001.inputs[0].default_value = (2.0, 2.0, 2.0)
    cube_001.inputs[1].default_value = 2
    cube_001.inputs[2].default_value = 2
    cube_001.inputs[3].default_value = 2
    math_1 = lunarrock.nodes.new("ShaderNodeMath")
    math_1.operation = "SUBTRACT"
    math_1.use_clamp = False
    math_1.inputs[1].default_value = 1.0
    reroute_004 = lunarrock.nodes.new("NodeReroute")
    frame_004 = lunarrock.nodes.new("NodeFrame")
    frame_004.shrink = True
    reroute_012 = lunarrock.nodes.new("NodeReroute")
    reroute_013 = lunarrock.nodes.new("NodeReroute")
    transform_geometry_003 = lunarrock.nodes.new("GeometryNodeTransform")
    transform_geometry_003.mode = "COMPONENTS"
    transform_geometry_003.inputs[1].hide = True
    transform_geometry_003.inputs[2].hide = True
    transform_geometry_003.inputs[4].hide = True
    transform_geometry_003.inputs[1].default_value = (0.0, 0.0, 0.0)
    transform_geometry_003.inputs[2].default_value = (0.0, 0.0, 0.0)
    combine_xyz_001 = lunarrock.nodes.new("ShaderNodeCombineXYZ")
    group_009 = lunarrock.nodes.new("GeometryNodeGroup")
    group_009.node_tree = random__normal_
    group_009.inputs[0].default_value = True
    group_009.inputs[4].default_value = 31680
    group_013 = lunarrock.nodes.new("GeometryNodeGroup")
    group_013.node_tree = random__normal_
    group_013.inputs[0].default_value = True
    group_013.inputs[4].default_value = 32260
    group_014 = lunarrock.nodes.new("GeometryNodeGroup")
    group_014.node_tree = random__normal_
    group_014.inputs[0].default_value = True
    group_014.inputs[4].default_value = 40590
    reroute_015 = lunarrock.nodes.new("NodeReroute")
    separate_xyz = lunarrock.nodes.new("ShaderNodeSeparateXYZ")
    separate_xyz_003 = lunarrock.nodes.new("ShaderNodeSeparateXYZ")
    reroute_017 = lunarrock.nodes.new("NodeReroute")
    reroute_018 = lunarrock.nodes.new("NodeReroute")
    reroute_019 = lunarrock.nodes.new("NodeReroute")
    reroute_020 = lunarrock.nodes.new("NodeReroute")
    reroute_021 = lunarrock.nodes.new("NodeReroute")
    reroute_022 = lunarrock.nodes.new("NodeReroute")
    frame_005 = lunarrock.nodes.new("NodeFrame")
    frame_005.shrink = True
    math_001_1 = lunarrock.nodes.new("ShaderNodeMath")
    math_001_1.operation = "ADD"
    math_001_1.use_clamp = False
    integer_001 = lunarrock.nodes.new("FunctionNodeInputInt")
    integer_001.integer = 0
    cube.parent = frame_002
    subdivision_surface.parent = frame_002
    set_position.parent = frame_1
    voronoi_texture.parent = frame_1
    vector_math.parent = frame_1
    position.parent = frame_1
    map_range.parent = frame_1
    set_position_001.parent = frame_001_1
    vector_math_001.parent = frame_001_1
    position_001.parent = frame_001_1
    noise_texture.parent = frame_001_1
    reroute_001.parent = frame_001_1
    transform_geometry.parent = frame_003_1
    reroute_002.parent = frame_002
    attribute_statistic.parent = frame_003_1
    position_002.parent = frame_003_1
    reroute_003.parent = frame_003_1
    vector_math_002.parent = frame_003_1
    vector_math_003.parent = frame_003_1
    vector_math_004.parent = frame_003_1
    group.parent = frame_1
    group_001.parent = frame_1
    group_002.parent = frame_1
    group_004.parent = frame_1
    float_curve.parent = frame_1
    group_005.parent = frame_1
    reroute_005.parent = frame_1
    group_003.parent = frame_002
    group_006.parent = frame_002
    reroute_006.parent = frame_002
    group_007.parent = frame_001_1
    group_008.parent = frame_001_1
    group_010.parent = frame_001_1
    group_011.parent = frame_001_1
    group_012.parent = frame_001_1
    transform_geometry_001.parent = frame_002
    random_value.parent = frame_002
    integer.parent = frame_002
    delete_geometry.parent = frame_004
    compare.parent = frame_004
    position_004.parent = frame_004
    separate_xyz_001.parent = frame_004
    normal_001.parent = frame_004
    boolean_math.parent = frame_004
    separate_xyz_002.parent = frame_004
    compare_001.parent = frame_004
    mesh_boolean.parent = frame_004
    switch_1.parent = frame_004
    transform_geometry_002.parent = frame_004
    combine_xyz.parent = frame_004
    cube_001.parent = frame_004
    math_1.parent = frame_004
    reroute_004.parent = frame_004
    reroute_012.parent = frame_004
    transform_geometry_003.parent = frame_005
    combine_xyz_001.parent = frame_005
    group_009.parent = frame_005
    group_013.parent = frame_005
    group_014.parent = frame_005
    separate_xyz.parent = frame_005
    separate_xyz_003.parent = frame_005
    group_input_2.location = (-4957.7626953125, -82.99918365478516)
    group_output_2.location = (2629.999755859375, 0.0)
    set_material.location = (2439.999755859375, 23.5)
    cube.location = (-1513.1207275390625, 915.7615966796875)
    subdivision_surface.location = (-340.2048034667969, 1023.5044555664062)
    set_position.location = (726.622314453125, 908.64501953125)
    voronoi_texture.location = (-133.37771606445312, 678.8929443359375)
    vector_math.location = (536.622314453125, 703.8929443359375)
    position.location = (346.6222839355469, 666.3929443359375)
    map_range.location = (346.6222839355469, 601.3929443359375)
    set_position_001.location = (1584.2333984375, 260.2159423828125)
    vector_math_001.location = (1394.2333984375, 108.06204986572266)
    position_001.location = (1204.2333984375, 70.56206512451172)
    noise_texture.location = (1001.9918212890625, 32.7781982421875)
    set_shade_smooth.location = (2249.999755859375, 35.5)
    frame_1.location = (-2794.595703125, 186.608642578125)
    frame_001_1.location = (-3649.0771484375, -318.5823059082031)
    frame_002.location = (-3290.830078125, 71.48291015625)
    reroute_001.location = (666.5570068359375, -219.35711669921875)
    transform_geometry.location = (1809.3837890625, -34.2188835144043)
    reroute_002.location = (-688.463623046875, 896.0345458984375)
    attribute_statistic.location = (1183.53466796875, -268.6141662597656)
    position_002.location = (993.53466796875, -339.6141662597656)
    reroute_003.location = (1086.59716796875, -128.1580352783203)
    vector_math_002.location = (1373.53466796875, -337.17041015625)
    vector_math_003.location = (1373.53466796875, -197.17041015625)
    vector_math_004.location = (1563.53466796875, -198.17041015625)
    group.location = (-434.4251708984375, 622.037353515625)
    group_001.location = (-434.4251708984375, 800.037353515625)
    group_002.location = (-434.4251708984375, 422.037353515625)
    group_004.location = (-434.4251708984375, 22.037353515625)
    float_curve.location = (56.622283935546875, 636.3929443359375)
    group_005.location = (-434.4251708984375, 222.037353515625)
    reroute_005.location = (-594.6759643554688, 278.4246826171875)
    group_003.location = (-654.701904296875, 678.068603515625)
    group_006.location = (-654.701904296875, 878.068603515625)
    reroute_006.location = (-694.4317626953125, 732.759765625)
    reroute.location = (-3415.009765625, -1250.500732421875)
    group_007.location = (733.760498046875, 245.83392333984375)
    group_008.location = (733.760498046875, 67.83392333984375)
    group_010.location = (733.760498046875, -132.16607666015625)
    group_011.location = (733.760498046875, -332.16607666015625)
    group_012.location = (733.760498046875, -532.1660766601562)
    frame_003_1.location = (-2665.356201171875, 28.966693878173828)
    transform_geometry_001.location = (-943.1207275390625, 961.654052734375)
    random_value.location = (-1133.1207275390625, 838.7128295898438)
    integer.location = (-1323.1207275390625, 816.7128295898438)
    delete_geometry.location = (1346.2703857421875, -536.0291137695312)
    compare.location = (966.2703857421875, -950.8118896484375)
    position_004.location = (586.2703857421875, -1009.8118896484375)
    separate_xyz_001.location = (776.2703857421875, -997.3118896484375)
    normal_001.location = (586.2703857421875, -828.6788330078125)
    boolean_math.location = (1156.2703857421875, -769.6106567382812)
    separate_xyz_002.location = (776.2703857421875, -816.1788330078125)
    compare_001.location = (966.2703857421875, -769.6788330078125)
    mesh_boolean.location = (965.2637939453125, -511.6483154296875)
    switch_1.location = (1665.2486572265625, -268.939453125)
    transform_geometry_002.location = (775.2637939453125, -585.58447265625)
    combine_xyz.location = (585.2637939453125, -656.6286010742188)
    reroute_010.location = (-3415.009765625, -1287.8094482421875)
    cube_001.location = (395.2638244628906, -535.58447265625)
    math_1.location = (395.2638244628906, -773.58447265625)
    reroute_004.location = (702.1375732421875, -330.8438720703125)
    frame_004.location = (-904.48779296875, 272.091064453125)
    reroute_012.location = (368.2854309082031, -1095.044189453125)
    reroute_013.location = (-3415.009765625, -1303.5126953125)
    transform_geometry_003.location = (1706.2198486328125, -32.31362533569336)
    combine_xyz_001.location = (1538.015380859375, -398.0220642089844)
    group_009.location = (1308.17041015625, -245.8809814453125)
    group_013.location = (1308.17041015625, -445.8809814453125)
    group_014.location = (1308.17041015625, -645.8809814453125)
    reroute_015.location = (-3415.009765625, -1234.2786865234375)
    separate_xyz.location = (949.9049682617188, -448.3955383300781)
    separate_xyz_003.location = (949.9049682617188, -582.3955078125)
    reroute_017.location = (-3415.009765625, -1270.66552734375)
    reroute_018.location = (871.9225463867188, -1251.0347900390625)
    reroute_019.location = (871.9225463867188, -1271.1376953125)
    reroute_020.location = (1295.392578125, -1234.255126953125)
    reroute_021.location = (559.1981201171875, -1287.5440673828125)
    reroute_022.location = (-1005.0067138671875, -1303.1754150390625)
    frame_005.location = (145.8409423828125, 25.487651824951172)
    math_001_1.location = (-4690.5810546875, -4.762271881103516)
    integer_001.location = (-4956.11669921875, 50.9406852722168)
    lunarrock.links.new(set_material.outputs[0], group_output_2.inputs[0])
    lunarrock.links.new(set_shade_smooth.outputs[0], set_material.inputs[0])
    lunarrock.links.new(reroute_002.outputs[0], subdivision_surface.inputs[1])
    lunarrock.links.new(position.outputs[0], vector_math.inputs[0])
    lunarrock.links.new(map_range.outputs[0], vector_math.inputs[1])
    lunarrock.links.new(vector_math.outputs[0], set_position.inputs[2])
    lunarrock.links.new(position_001.outputs[0], vector_math_001.inputs[0])
    lunarrock.links.new(noise_texture.outputs[0], vector_math_001.inputs[1])
    lunarrock.links.new(reroute_003.outputs[0], transform_geometry.inputs[0])
    lunarrock.links.new(math_001_1.outputs[0], reroute_001.inputs[0])
    lunarrock.links.new(position_002.outputs[0], attribute_statistic.inputs[2])
    lunarrock.links.new(set_position_001.outputs[0], reroute_003.inputs[0])
    lunarrock.links.new(reroute_003.outputs[0], attribute_statistic.inputs[0])
    lunarrock.links.new(vector_math_002.outputs[0], transform_geometry.inputs[3])
    lunarrock.links.new(attribute_statistic.outputs[5], vector_math_002.inputs[1])
    lunarrock.links.new(vector_math_004.outputs[0], transform_geometry.inputs[1])
    lunarrock.links.new(vector_math_003.outputs[0], vector_math_004.inputs[0])
    lunarrock.links.new(attribute_statistic.outputs[3], vector_math_003.inputs[0])
    lunarrock.links.new(attribute_statistic.outputs[4], vector_math_003.inputs[1])
    lunarrock.links.new(group_001.outputs[0], voronoi_texture.inputs[1])
    lunarrock.links.new(reroute_005.outputs[0], group_001.inputs[2])
    lunarrock.links.new(group.outputs[0], voronoi_texture.inputs[2])
    lunarrock.links.new(group_002.outputs[0], voronoi_texture.inputs[3])
    lunarrock.links.new(group_004.outputs[0], voronoi_texture.inputs[5])
    lunarrock.links.new(reroute_005.outputs[0], group.inputs[3])
    lunarrock.links.new(reroute_005.outputs[0], group_002.inputs[3])
    lunarrock.links.new(reroute_005.outputs[0], group_004.inputs[3])
    lunarrock.links.new(subdivision_surface.outputs[0], set_position.inputs[0])
    lunarrock.links.new(set_position.outputs[0], set_position_001.inputs[0])
    lunarrock.links.new(float_curve.outputs[0], map_range.inputs[0])
    lunarrock.links.new(voronoi_texture.outputs[0], float_curve.inputs[1])
    lunarrock.links.new(reroute_005.outputs[0], group_005.inputs[3])
    lunarrock.links.new(group_005.outputs[0], voronoi_texture.inputs[4])
    lunarrock.links.new(math_001_1.outputs[0], reroute_005.inputs[0])
    lunarrock.links.new(reroute_006.outputs[0], group_003.inputs[3])
    lunarrock.links.new(reroute_006.outputs[0], group_006.inputs[3])
    lunarrock.links.new(group_003.outputs[0], subdivision_surface.inputs[3])
    lunarrock.links.new(group_006.outputs[0], subdivision_surface.inputs[2])
    lunarrock.links.new(math_001_1.outputs[0], reroute_006.inputs[0])
    lunarrock.links.new(group_input_2.outputs[2], reroute.inputs[0])
    lunarrock.links.new(group_input_2.outputs[1], reroute_002.inputs[0])
    lunarrock.links.new(vector_math_001.outputs[0], set_position_001.inputs[2])
    lunarrock.links.new(reroute_001.outputs[0], group_007.inputs[2])
    lunarrock.links.new(group_007.outputs[0], noise_texture.inputs[1])
    lunarrock.links.new(reroute_001.outputs[0], group_008.inputs[3])
    lunarrock.links.new(reroute_001.outputs[0], group_010.inputs[3])
    lunarrock.links.new(reroute_001.outputs[0], group_011.inputs[3])
    lunarrock.links.new(reroute_001.outputs[0], group_012.inputs[3])
    lunarrock.links.new(group_012.outputs[0], noise_texture.inputs[8])
    lunarrock.links.new(group_010.outputs[0], noise_texture.inputs[4])
    lunarrock.links.new(group_011.outputs[0], noise_texture.inputs[5])
    lunarrock.links.new(group_008.outputs[0], noise_texture.inputs[2])
    lunarrock.links.new(integer.outputs[0], random_value.inputs[7])
    lunarrock.links.new(random_value.outputs[0], transform_geometry_001.inputs[2])
    lunarrock.links.new(
        transform_geometry_001.outputs[0], subdivision_surface.inputs[0]
    )
    lunarrock.links.new(cube.outputs[0], transform_geometry_001.inputs[0])
    lunarrock.links.new(math_001_1.outputs[0], random_value.inputs[8])
    lunarrock.links.new(mesh_boolean.outputs[0], delete_geometry.inputs[0])
    lunarrock.links.new(position_004.outputs[0], compare.inputs[4])
    lunarrock.links.new(separate_xyz_001.outputs[2], compare.inputs[0])
    lunarrock.links.new(normal_001.outputs[0], separate_xyz_002.inputs[0])
    lunarrock.links.new(separate_xyz_002.outputs[2], compare_001.inputs[0])
    lunarrock.links.new(boolean_math.outputs[0], delete_geometry.inputs[1])
    lunarrock.links.new(reroute_004.outputs[0], mesh_boolean.inputs[0])
    lunarrock.links.new(transform_geometry_002.outputs[0], mesh_boolean.inputs[1])
    lunarrock.links.new(position_004.outputs[0], separate_xyz_001.inputs[0])
    lunarrock.links.new(reroute_021.outputs[0], switch_1.inputs[0])
    lunarrock.links.new(transform_geometry_003.outputs[0], set_shade_smooth.inputs[0])
    lunarrock.links.new(reroute_004.outputs[0], switch_1.inputs[1])
    lunarrock.links.new(delete_geometry.outputs[0], switch_1.inputs[2])
    lunarrock.links.new(math_1.outputs[0], combine_xyz.inputs[2])
    lunarrock.links.new(reroute_012.outputs[0], compare.inputs[1])
    lunarrock.links.new(group_input_2.outputs[4], reroute_010.inputs[0])
    lunarrock.links.new(compare_001.outputs[0], boolean_math.inputs[0])
    lunarrock.links.new(compare.outputs[0], boolean_math.inputs[1])
    lunarrock.links.new(cube_001.outputs[0], transform_geometry_002.inputs[0])
    lunarrock.links.new(combine_xyz.outputs[0], transform_geometry_002.inputs[1])
    lunarrock.links.new(reroute_012.outputs[0], math_1.inputs[0])
    lunarrock.links.new(transform_geometry.outputs[0], reroute_004.inputs[0])
    lunarrock.links.new(reroute_022.outputs[0], reroute_012.inputs[0])
    lunarrock.links.new(group_input_2.outputs[5], reroute_013.inputs[0])
    lunarrock.links.new(switch_1.outputs[0], transform_geometry_003.inputs[0])
    lunarrock.links.new(group_009.outputs[0], combine_xyz_001.inputs[0])
    lunarrock.links.new(group_013.outputs[0], combine_xyz_001.inputs[1])
    lunarrock.links.new(group_014.outputs[0], combine_xyz_001.inputs[2])
    lunarrock.links.new(combine_xyz_001.outputs[0], transform_geometry_003.inputs[3])
    lunarrock.links.new(math_001_1.outputs[0], reroute_015.inputs[0])
    lunarrock.links.new(reroute_020.outputs[0], group_013.inputs[3])
    lunarrock.links.new(reroute_020.outputs[0], group_009.inputs[3])
    lunarrock.links.new(reroute_020.outputs[0], group_014.inputs[3])
    lunarrock.links.new(reroute_019.outputs[0], separate_xyz_003.inputs[0])
    lunarrock.links.new(separate_xyz_003.outputs[0], group_009.inputs[2])
    lunarrock.links.new(separate_xyz_003.outputs[1], group_013.inputs[2])
    lunarrock.links.new(separate_xyz_003.outputs[2], group_014.inputs[2])
    lunarrock.links.new(separate_xyz.outputs[0], group_009.inputs[1])
    lunarrock.links.new(separate_xyz.outputs[1], group_013.inputs[1])
    lunarrock.links.new(separate_xyz.outputs[2], group_014.inputs[1])
    lunarrock.links.new(reroute_018.outputs[0], separate_xyz.inputs[0])
    lunarrock.links.new(group_input_2.outputs[3], reroute_017.inputs[0])
    lunarrock.links.new(reroute.outputs[0], reroute_018.inputs[0])
    lunarrock.links.new(reroute_017.outputs[0], reroute_019.inputs[0])
    lunarrock.links.new(reroute_015.outputs[0], reroute_020.inputs[0])
    lunarrock.links.new(reroute_010.outputs[0], reroute_021.inputs[0])
    lunarrock.links.new(reroute_013.outputs[0], reroute_022.inputs[0])
    lunarrock.links.new(group_input_2.outputs[0], math_001_1.inputs[0])
    lunarrock.links.new(integer_001.outputs[0], math_001_1.inputs[1])
    return lunarrock


lunarrock = lunarrock_node_group()


def crater_profile_node_group():
    crater_profile = bpy.data.node_groups.new(
        type="GeometryNodeTree", name="Crater Profile"
    )
    crater_profile.color_tag = "NONE"
    value_socket_2 = crater_profile.interface.new_socket(
        name="Value", in_out="OUTPUT", socket_type="NodeSocketFloat"
    )
    value_socket_2.default_value = 0.0
    value_socket_2.subtype = "NONE"
    value_socket_2.attribute_domain = "POINT"
    value_socket_3 = crater_profile.interface.new_socket(
        name="Value", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    value_socket_3.default_value = 1.0
    value_socket_3.subtype = "NONE"
    value_socket_3.attribute_domain = "POINT"
    crater_radius_fraction_socket = crater_profile.interface.new_socket(
        name="Crater Radius Fraction", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    crater_radius_fraction_socket.default_value = 0.0
    crater_radius_fraction_socket.subtype = "FACTOR"
    crater_radius_fraction_socket.attribute_domain = "POINT"
    max_crater_radius_socket = crater_profile.interface.new_socket(
        name="Max Crater Radius", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    max_crater_radius_socket.default_value = 0.0
    max_crater_radius_socket.subtype = "NONE"
    max_crater_radius_socket.attribute_domain = "POINT"
    seed_socket_3 = crater_profile.interface.new_socket(
        name="Seed", in_out="INPUT", socket_type="NodeSocketInt"
    )
    seed_socket_3.default_value = 0
    seed_socket_3.subtype = "NONE"
    seed_socket_3.attribute_domain = "POINT"
    seed_socket_3.force_non_field = True
    group_output_3 = crater_profile.nodes.new("NodeGroupOutput")
    group_output_3.is_active_output = True
    group_input_3 = crater_profile.nodes.new("NodeGroupInput")
    noise_texture_011 = crater_profile.nodes.new("ShaderNodeTexNoise")
    noise_texture_011.noise_dimensions = "4D"
    noise_texture_011.noise_type = "FBM"
    noise_texture_011.normalize = True
    noise_texture_011.inputs[0].default_value = (0.0, 0.0, 0.0)
    noise_texture_011.inputs[3].default_value = 15.0
    noise_texture_011.inputs[8].default_value = 0.0
    group_019 = crater_profile.nodes.new("GeometryNodeGroup")
    group_019.node_tree = random__uniform_
    group_019.inputs[0].default_value = -100000000.0
    group_019.inputs[1].default_value = 1000000000.0
    group_019.inputs[3].default_value = 46364
    group_022 = crater_profile.nodes.new("GeometryNodeGroup")
    group_022.node_tree = random__normal_
    group_022.inputs[0].default_value = False
    group_022.inputs[4].default_value = 2808
    group_023 = crater_profile.nodes.new("GeometryNodeGroup")
    group_023.node_tree = random__normal_
    group_023.inputs[0].default_value = True
    group_023.inputs[1].default_value = 0.30000001192092896
    group_023.inputs[2].default_value = 0.02500000037252903
    group_023.inputs[4].default_value = 8508
    group_024 = crater_profile.nodes.new("GeometryNodeGroup")
    group_024.node_tree = random__normal_
    group_024.inputs[0].default_value = True
    group_024.inputs[1].default_value = 2.25
    group_024.inputs[2].default_value = 0.25
    group_024.inputs[4].default_value = 141
    float_to_integer = crater_profile.nodes.new("FunctionNodeFloatToInt")
    float_to_integer.rounding_mode = "ROUND"
    math_001_2 = crater_profile.nodes.new("ShaderNodeMath")
    math_001_2.operation = "MULTIPLY"
    math_001_2.use_clamp = False
    math_001_2.inputs[2].hide = True
    reroute_002_1 = crater_profile.nodes.new("NodeReroute")
    integer_1 = crater_profile.nodes.new("FunctionNodeInputInt")
    integer_1.integer = 4
    math_003_1 = crater_profile.nodes.new("ShaderNodeMath")
    math_003_1.operation = "ADD"
    math_003_1.use_clamp = False
    map_range_1 = crater_profile.nodes.new("ShaderNodeMapRange")
    map_range_1.clamp = True
    map_range_1.data_type = "FLOAT"
    map_range_1.interpolation_type = "LINEAR"
    map_range_1.inputs[1].default_value = 0.0
    map_range_1.inputs[2].default_value = 1.0
    map_range_1.inputs[3].default_value = 0.0
    group_1 = crater_profile.nodes.new("GeometryNodeGroup")
    group_1.node_tree = random__normal_
    group_1.inputs[0].default_value = True
    group_1.inputs[1].default_value = 0.0
    group_1.inputs[2].default_value = 0.25
    group_1.inputs[4].default_value = 24183
    float_curve_004 = crater_profile.nodes.new("ShaderNodeFloatCurve")
    float_curve_004.mapping.extend = "EXTRAPOLATED"
    float_curve_004.mapping.tone = "STANDARD"
    float_curve_004.mapping.black_level = (0.0, 0.0, 0.0)
    float_curve_004.mapping.white_level = (1.0, 1.0, 1.0)
    float_curve_004.mapping.clip_min_x = 0.0
    float_curve_004.mapping.clip_min_y = 0.0
    float_curve_004.mapping.clip_max_x = 1.0
    float_curve_004.mapping.clip_max_y = 1.0
    float_curve_004.mapping.use_clip = True
    float_curve_004_curve_0 = float_curve_004.mapping.curves[0]
    float_curve_004_curve_0_point_0 = float_curve_004_curve_0.points[0]
    float_curve_004_curve_0_point_0.location = (0.0, 0.6302875280380249)
    float_curve_004_curve_0_point_0.handle_type = "AUTO"
    float_curve_004_curve_0_point_1 = float_curve_004_curve_0.points[1]
    float_curve_004_curve_0_point_1.location = (0.20408575236797333, 0.6508127450942993)
    float_curve_004_curve_0_point_1.handle_type = "AUTO"
    float_curve_004_curve_0_point_2 = float_curve_004_curve_0.points.new(
        0.3318162262439728, 0.7192298769950867
    )
    float_curve_004_curve_0_point_2.handle_type = "AUTO"
    float_curve_004_curve_0_point_3 = float_curve_004_curve_0.points.new(
        0.4255254566669464, 0.7699005007743835
    )
    float_curve_004_curve_0_point_3.handle_type = "AUTO"
    float_curve_004_curve_0_point_4 = float_curve_004_curve_0.points.new(
        0.5083944797515869, 0.5437449216842651
    )
    float_curve_004_curve_0_point_4.handle_type = "AUTO"
    float_curve_004_curve_0_point_5 = float_curve_004_curve_0.points.new(
        0.7643035054206848, 0.13317757844924927
    )
    float_curve_004_curve_0_point_5.handle_type = "AUTO"
    float_curve_004_curve_0_point_6 = float_curve_004_curve_0.points.new(
        1.0, 0.056249916553497314
    )
    float_curve_004_curve_0_point_6.handle_type = "AUTO"
    float_curve_004.mapping.update()
    float_curve_004.inputs[0].default_value = 1.0
    float_curve_005 = crater_profile.nodes.new("ShaderNodeFloatCurve")
    float_curve_005.mapping.extend = "EXTRAPOLATED"
    float_curve_005.mapping.tone = "STANDARD"
    float_curve_005.mapping.black_level = (0.0, 0.0, 0.0)
    float_curve_005.mapping.white_level = (1.0, 1.0, 1.0)
    float_curve_005.mapping.clip_min_x = 0.0
    float_curve_005.mapping.clip_min_y = 0.0
    float_curve_005.mapping.clip_max_x = 1.0
    float_curve_005.mapping.clip_max_y = 1.0
    float_curve_005.mapping.use_clip = True
    float_curve_005_curve_0 = float_curve_005.mapping.curves[0]
    float_curve_005_curve_0_point_0 = float_curve_005_curve_0.points[0]
    float_curve_005_curve_0_point_0.location = (0.0, 0.6241841912269592)
    float_curve_005_curve_0_point_0.handle_type = "AUTO"
    float_curve_005_curve_0_point_1 = float_curve_005_curve_0.points[1]
    float_curve_005_curve_0_point_1.location = (0.20329293608665466, 0.6318109631538391)
    float_curve_005_curve_0_point_1.handle_type = "AUTO"
    float_curve_005_curve_0_point_2 = float_curve_005_curve_0.points.new(
        0.3409092426300049, 0.6676813960075378
    )
    float_curve_005_curve_0_point_2.handle_type = "AUTO_CLAMPED"
    float_curve_005_curve_0_point_3 = float_curve_005_curve_0.points.new(
        0.5181822776794434, 0.4260922968387604
    )
    float_curve_005_curve_0_point_3.handle_type = "AUTO"
    float_curve_005_curve_0_point_4 = float_curve_005_curve_0.points.new(
        0.7181820273399353, 0.2250000238418579
    )
    float_curve_005_curve_0_point_4.handle_type = "AUTO"
    float_curve_005_curve_0_point_5 = float_curve_005_curve_0.points.new(
        1.0, 0.16875003278255463
    )
    float_curve_005_curve_0_point_5.handle_type = "AUTO"
    float_curve_005.mapping.update()
    float_curve_005.inputs[0].default_value = 1.0
    float_curve_006 = crater_profile.nodes.new("ShaderNodeFloatCurve")
    float_curve_006.mapping.extend = "EXTRAPOLATED"
    float_curve_006.mapping.tone = "STANDARD"
    float_curve_006.mapping.black_level = (0.0, 0.0, 0.0)
    float_curve_006.mapping.white_level = (1.0, 1.0, 1.0)
    float_curve_006.mapping.clip_min_x = 0.0
    float_curve_006.mapping.clip_min_y = 0.0
    float_curve_006.mapping.clip_max_x = 1.0
    float_curve_006.mapping.clip_max_y = 1.0
    float_curve_006.mapping.use_clip = True
    float_curve_006_curve_0 = float_curve_006.mapping.curves[0]
    float_curve_006_curve_0_point_0 = float_curve_006_curve_0.points[0]
    float_curve_006_curve_0_point_0.location = (0.0, 0.6725000739097595)
    float_curve_006_curve_0_point_0.handle_type = "AUTO"
    float_curve_006_curve_0_point_1 = float_curve_006_curve_0.points[1]
    float_curve_006_curve_0_point_1.location = (0.18717996776103973, 0.6866194605827332)
    float_curve_006_curve_0_point_1.handle_type = "AUTO"
    float_curve_006_curve_0_point_2 = float_curve_006_curve_0.points.new(
        0.38181814551353455, 0.7312501072883606
    )
    float_curve_006_curve_0_point_2.handle_type = "AUTO"
    float_curve_006_curve_0_point_3 = float_curve_006_curve_0.points.new(
        0.47272729873657227, 0.7426979541778564
    )
    float_curve_006_curve_0_point_3.handle_type = "AUTO"
    float_curve_006_curve_0_point_4 = float_curve_006_curve_0.points.new(
        0.6454547047615051, 0.24985311925411224
    )
    float_curve_006_curve_0_point_4.handle_type = "AUTO"
    float_curve_006_curve_0_point_5 = float_curve_006_curve_0.points.new(
        1.0, 0.13730427622795105
    )
    float_curve_006_curve_0_point_5.handle_type = "AUTO"
    float_curve_006.mapping.update()
    float_curve_006.inputs[0].default_value = 1.0
    float_curve_007 = crater_profile.nodes.new("ShaderNodeFloatCurve")
    float_curve_007.mapping.extend = "EXTRAPOLATED"
    float_curve_007.mapping.tone = "STANDARD"
    float_curve_007.mapping.black_level = (0.0, 0.0, 0.0)
    float_curve_007.mapping.white_level = (1.0, 1.0, 1.0)
    float_curve_007.mapping.clip_min_x = 0.0
    float_curve_007.mapping.clip_min_y = 0.0
    float_curve_007.mapping.clip_max_x = 1.0
    float_curve_007.mapping.clip_max_y = 1.0
    float_curve_007.mapping.use_clip = True
    float_curve_007_curve_0 = float_curve_007.mapping.curves[0]
    float_curve_007_curve_0_point_0 = float_curve_007_curve_0.points[0]
    float_curve_007_curve_0_point_0.location = (0.0, 0.7124999761581421)
    float_curve_007_curve_0_point_0.handle_type = "AUTO"
    float_curve_007_curve_0_point_1 = float_curve_007_curve_0.points[1]
    float_curve_007_curve_0_point_1.location = (0.2611362040042877, 0.7326563000679016)
    float_curve_007_curve_0_point_1.handle_type = "AUTO"
    float_curve_007_curve_0_point_2 = float_curve_007_curve_0.points.new(
        0.4363635778427124, 0.7750000953674316
    )
    float_curve_007_curve_0_point_2.handle_type = "AUTO"
    float_curve_007_curve_0_point_3 = float_curve_007_curve_0.points.new(
        0.5954543352127075, 0.8114726543426514
    )
    float_curve_007_curve_0_point_3.handle_type = "AUTO"
    float_curve_007_curve_0_point_4 = float_curve_007_curve_0.points.new(
        0.6954542994499207, 0.5309724807739258
    )
    float_curve_007_curve_0_point_4.handle_type = "AUTO"
    float_curve_007_curve_0_point_5 = float_curve_007_curve_0.points.new(
        0.8590908646583557, 0.2937498986721039
    )
    float_curve_007_curve_0_point_5.handle_type = "AUTO"
    float_curve_007_curve_0_point_6 = float_curve_007_curve_0.points.new(
        1.0, 0.24999989569187164
    )
    float_curve_007_curve_0_point_6.handle_type = "AUTO"
    float_curve_007.mapping.update()
    float_curve_007.inputs[0].default_value = 1.0
    reroute_003_1 = crater_profile.nodes.new("NodeReroute")
    math_005_1 = crater_profile.nodes.new("ShaderNodeMath")
    math_005_1.operation = "MULTIPLY"
    math_005_1.use_clamp = False
    index_switch_001 = crater_profile.nodes.new("GeometryNodeIndexSwitch")
    index_switch_001.data_type = "FLOAT"
    index_switch_001.index_switch_items.clear()
    index_switch_001.index_switch_items.new()
    index_switch_001.index_switch_items.new()
    index_switch_001.index_switch_items.new()
    index_switch_001.index_switch_items.new()
    index_switch_001.index_switch_items.new()
    math_2 = crater_profile.nodes.new("ShaderNodeMath")
    math_2.operation = "MULTIPLY"
    math_2.use_clamp = False
    math_2.inputs[1].default_value = 1.25
    math_002_1 = crater_profile.nodes.new("ShaderNodeMath")
    math_002_1.operation = "MULTIPLY"
    math_002_1.use_clamp = False
    math_002_1.inputs[1].default_value = 0.019999999552965164
    float_curve_008 = crater_profile.nodes.new("ShaderNodeFloatCurve")
    float_curve_008.mapping.extend = "EXTRAPOLATED"
    float_curve_008.mapping.tone = "STANDARD"
    float_curve_008.mapping.black_level = (0.0, 0.0, 0.0)
    float_curve_008.mapping.white_level = (1.0, 1.0, 1.0)
    float_curve_008.mapping.clip_min_x = 0.0
    float_curve_008.mapping.clip_min_y = 0.0
    float_curve_008.mapping.clip_max_x = 1.0
    float_curve_008.mapping.clip_max_y = 1.0
    float_curve_008.mapping.use_clip = True
    float_curve_008_curve_0 = float_curve_008.mapping.curves[0]
    float_curve_008_curve_0_point_0 = float_curve_008_curve_0.points[0]
    float_curve_008_curve_0_point_0.location = (0.0, 0.6662500500679016)
    float_curve_008_curve_0_point_0.handle_type = "AUTO"
    float_curve_008_curve_0_point_1 = float_curve_008_curve_0.points[1]
    float_curve_008_curve_0_point_1.location = (0.1954544186592102, 0.672716498374939)
    float_curve_008_curve_0_point_1.handle_type = "AUTO"
    float_curve_008_curve_0_point_2 = float_curve_008_curve_0.points.new(
        0.38636353611946106, 0.7116192579269409
    )
    float_curve_008_curve_0_point_2.handle_type = "AUTO"
    float_curve_008_curve_0_point_3 = float_curve_008_curve_0.points.new(
        0.7363638877868652, 0.3500000238418579
    )
    float_curve_008_curve_0_point_3.handle_type = "AUTO"
    float_curve_008_curve_0_point_4 = float_curve_008_curve_0.points.new(
        1.0, 0.29374992847442627
    )
    float_curve_008_curve_0_point_4.handle_type = "AUTO"
    float_curve_008.mapping.update()
    float_curve_008.inputs[0].default_value = 1.0
    math_004_1 = crater_profile.nodes.new("ShaderNodeMath")
    math_004_1.operation = "INVERSE_SQRT"
    math_004_1.use_clamp = False
    group_output_3.location = (712.6826171875, 17.99111557006836)
    group_input_3.location = (-3242.14990234375, -6.254646301269531)
    noise_texture_011.location = (-512.7152709960938, -439.41162109375)
    group_019.location = (-767.8524780273438, -273.099609375)
    group_022.location = (-767.8524780273438, -451.099609375)
    group_023.location = (-767.8524780273438, -651.0993041992188)
    group_024.location = (-767.8524780273438, -851.0995483398438)
    float_to_integer.location = (-1804.8189697265625, -236.50123596191406)
    math_001_2.location = (-1546.259033203125, 83.0434799194336)
    reroute_002_1.location = (-996.7823486328125, -703.491455078125)
    integer_1.location = (-1547.5233154296875, -86.2983169555664)
    math_003_1.location = (-2346.594970703125, -119.19635772705078)
    map_range_1.location = (-2156.594970703125, -72.69635772705078)
    group_1.location = (-2596.355712890625, -310.373046875)
    float_curve_004.location = (-2345.31396484375, -817.3238525390625)
    float_curve_005.location = (-2345.31396484375, -1143.3238525390625)
    float_curve_006.location = (-2345.31396484375, -1469.3238525390625)
    float_curve_007.location = (-2345.31396484375, -491.3238525390625)
    reroute_003_1.location = (-2886.522216796875, -372.5995788574219)
    math_005_1.location = (-205.45945739746094, -92.80597686767578)
    index_switch_001.location = (-1547.0994873046875, -216.50979614257812)
    math_2.location = (-1226.86962890625, -299.31927490234375)
    math_002_1.location = (-1226.86962890625, -455.403564453125)
    float_curve_008.location = (-2345.31396484375, -1795.3238525390625)
    math_004_1.location = (-1004.6797485351562, -321.485595703125)
    crater_profile.links.new(group_019.outputs[0], noise_texture_011.inputs[1])
    crater_profile.links.new(reroute_002_1.outputs[0], group_019.inputs[2])
    crater_profile.links.new(reroute_002_1.outputs[0], group_022.inputs[3])
    crater_profile.links.new(reroute_002_1.outputs[0], group_023.inputs[3])
    crater_profile.links.new(reroute_002_1.outputs[0], group_024.inputs[3])
    crater_profile.links.new(group_input_3.outputs[1], math_001_2.inputs[0])
    crater_profile.links.new(group_input_3.outputs[2], math_001_2.inputs[1])
    crater_profile.links.new(math_005_1.outputs[0], group_output_3.inputs[0])
    crater_profile.links.new(group_input_3.outputs[3], reroute_002_1.inputs[0])
    crater_profile.links.new(integer_1.outputs[0], map_range_1.inputs[4])
    crater_profile.links.new(map_range_1.outputs[0], float_to_integer.inputs[0])
    crater_profile.links.new(group_input_3.outputs[3], group_1.inputs[3])
    crater_profile.links.new(group_1.outputs[0], math_003_1.inputs[1])
    crater_profile.links.new(math_003_1.outputs[0], map_range_1.inputs[0])
    crater_profile.links.new(group_input_3.outputs[1], math_003_1.inputs[0])
    crater_profile.links.new(group_022.outputs[0], noise_texture_011.inputs[2])
    crater_profile.links.new(group_023.outputs[0], noise_texture_011.inputs[4])
    crater_profile.links.new(group_024.outputs[0], noise_texture_011.inputs[5])
    crater_profile.links.new(reroute_003_1.outputs[0], float_curve_004.inputs[1])
    crater_profile.links.new(reroute_003_1.outputs[0], float_curve_005.inputs[1])
    crater_profile.links.new(reroute_003_1.outputs[0], float_curve_006.inputs[1])
    crater_profile.links.new(reroute_003_1.outputs[0], float_curve_007.inputs[1])
    crater_profile.links.new(group_input_3.outputs[0], reroute_003_1.inputs[0])
    crater_profile.links.new(noise_texture_011.outputs[0], math_005_1.inputs[1])
    crater_profile.links.new(float_curve_007.outputs[0], index_switch_001.inputs[1])
    crater_profile.links.new(float_curve_004.outputs[0], index_switch_001.inputs[2])
    crater_profile.links.new(float_curve_005.outputs[0], index_switch_001.inputs[3])
    crater_profile.links.new(float_curve_006.outputs[0], index_switch_001.inputs[4])
    crater_profile.links.new(index_switch_001.outputs[0], math_005_1.inputs[0])
    crater_profile.links.new(math_004_1.outputs[0], group_022.inputs[1])
    crater_profile.links.new(math_001_2.outputs[0], math_2.inputs[0])
    crater_profile.links.new(math_2.outputs[0], math_002_1.inputs[0])
    crater_profile.links.new(math_002_1.outputs[0], group_022.inputs[2])
    crater_profile.links.new(reroute_003_1.outputs[0], float_curve_008.inputs[1])
    crater_profile.links.new(float_curve_008.outputs[0], index_switch_001.inputs[5])
    crater_profile.links.new(float_to_integer.outputs[0], index_switch_001.inputs[0])
    crater_profile.links.new(math_2.outputs[0], math_004_1.inputs[0])
    return crater_profile


crater_profile = crater_profile_node_group()


def lunarterrain_node_group():
    lunarterrain = bpy.data.node_groups.new(
        type="GeometryNodeTree", name="LunarTerrain"
    )
    lunarterrain.color_tag = "NONE"
    lunarterrain.is_modifier = True
    geometry_socket_1 = lunarterrain.interface.new_socket(
        name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry"
    )
    geometry_socket_1.attribute_domain = "POINT"
    seed_socket_4 = lunarterrain.interface.new_socket(
        name="Seed", in_out="INPUT", socket_type="NodeSocketInt"
    )
    seed_socket_4.default_value = 0
    seed_socket_4.subtype = "NONE"
    seed_socket_4.attribute_domain = "POINT"
    seed_socket_4.force_non_field = True
    scale_socket_1 = lunarterrain.interface.new_socket(
        name="Scale", in_out="INPUT", socket_type="NodeSocketVector"
    )
    scale_socket_1.default_value = (10.0, 10.0, 1.0)
    scale_socket_1.subtype = "XYZ"
    scale_socket_1.attribute_domain = "POINT"
    scale_socket_1.force_non_field = True
    density_socket = lunarterrain.interface.new_socket(
        name="Density", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    density_socket.default_value = 0.10000000149011612
    density_socket.subtype = "NONE"
    density_socket.attribute_domain = "POINT"
    density_socket.force_non_field = True
    flat_area_size_socket = lunarterrain.interface.new_socket(
        name="Flat Area Size", in_out="INPUT", socket_type="NodeSocketFloat"
    )
    flat_area_size_socket.default_value = 0.0
    flat_area_size_socket.subtype = "NONE"
    flat_area_size_socket.attribute_domain = "POINT"
    rock_mesh_boolean_socket = lunarterrain.interface.new_socket(
        name="Rock Mesh Boolean", in_out="INPUT", socket_type="NodeSocketBool"
    )
    rock_mesh_boolean_socket.default_value = False
    rock_mesh_boolean_socket.attribute_domain = "POINT"
    rock_mesh_boolean_socket.force_non_field = True
    group_input_4 = lunarterrain.nodes.new("NodeGroupInput")
    group_output_4 = lunarterrain.nodes.new("NodeGroupOutput")
    group_output_4.is_active_output = True
    grid = lunarterrain.nodes.new("GeometryNodeMeshGrid")
    set_material_1 = lunarterrain.nodes.new("GeometryNodeSetMaterial")
    set_material_1.inputs[1].default_value = True
    if "LunarSurface" in bpy.data.materials:
        set_material_1.inputs[2].default_value = bpy.data.materials["LunarSurface"]
    set_shade_smooth_1 = lunarterrain.nodes.new("GeometryNodeSetShadeSmooth")
    set_shade_smooth_1.domain = "FACE"
    set_shade_smooth_1.inputs[1].default_value = True
    set_shade_smooth_1.inputs[2].default_value = True
    vector_math_012 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_012.operation = "SCALE"
    vector_math_012.inputs[3].default_value = -1.0
    raycast = lunarterrain.nodes.new("GeometryNodeRaycast")
    raycast.data_type = "FLOAT"
    raycast.mapping = "NEAREST"
    raycast.inputs[1].default_value = 0.0
    raycast.inputs[3].default_value = (0.0, 0.0, -1.0)
    raycast.inputs[4].default_value = 10.0
    frame_002_1 = lunarterrain.nodes.new("NodeFrame")
    frame_002_1.shrink = True
    vector_math_017 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_017.operation = "MULTIPLY"
    gradient_texture_001 = lunarterrain.nodes.new("ShaderNodeTexGradient")
    gradient_texture_001.gradient_type = "QUADRATIC_SPHERE"
    position_002_1 = lunarterrain.nodes.new("GeometryNodeInputPosition")
    vector_math_019 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_019.operation = "DIVIDE"
    set_position_001_1 = lunarterrain.nodes.new("GeometryNodeSetPosition")
    set_position_001_1.inputs[3].default_value = (0.0, 0.0, 0.0)
    position_003 = lunarterrain.nodes.new("GeometryNodeInputPosition")
    combine_xyz_1 = lunarterrain.nodes.new("ShaderNodeCombineXYZ")
    combine_xyz_1.inputs[0].default_value = 1.0
    combine_xyz_1.inputs[1].default_value = 1.0
    math_3 = lunarterrain.nodes.new("ShaderNodeMath")
    math_3.operation = "MULTIPLY"
    math_3.use_clamp = False
    math_3.inputs[1].default_value = 1.0
    frame_2 = lunarterrain.nodes.new("NodeFrame")
    frame_2.shrink = True
    reroute_001_1 = lunarterrain.nodes.new("NodeReroute")
    vector_math_002_1 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_002_1.operation = "DIVIDE"
    vector_math_021 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_021.operation = "CEIL"
    separate_xyz_1 = lunarterrain.nodes.new("ShaderNodeSeparateXYZ")
    separate_xyz_1.outputs[2].hide = True
    vector_math_023 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_023.operation = "MAXIMUM"
    vector_math_023.inputs[1].default_value = (2.0, 2.0, 0.0)
    frame_001_2 = lunarterrain.nodes.new("NodeFrame")
    frame_001_2.shrink = True
    reroute_003_2 = lunarterrain.nodes.new("NodeReroute")
    compare_1 = lunarterrain.nodes.new("FunctionNodeCompare")
    compare_1.data_type = "FLOAT"
    compare_1.mode = "ELEMENT"
    compare_1.operation = "NOT_EQUAL"
    compare_1.inputs[1].default_value = 0.0
    compare_1.inputs[12].default_value = 0.0010000000474974513
    math_001_3 = lunarterrain.nodes.new("ShaderNodeMath")
    math_001_3.operation = "ADD"
    math_001_3.use_clamp = False
    math_001_3.inputs[2].hide = True
    integer_012 = lunarterrain.nodes.new("FunctionNodeInputInt")
    integer_012.integer = 0
    reroute_005_1 = lunarterrain.nodes.new("NodeReroute")
    float_to_integer_1 = lunarterrain.nodes.new("FunctionNodeFloatToInt")
    float_to_integer_1.rounding_mode = "FLOOR"
    transform_geometry_001_1 = lunarterrain.nodes.new("GeometryNodeTransform")
    transform_geometry_001_1.mode = "COMPONENTS"
    transform_geometry_001_1.inputs[1].hide = True
    transform_geometry_001_1.inputs[2].hide = True
    transform_geometry_001_1.inputs[4].hide = True
    transform_geometry_001_1.inputs[1].default_value = (0.0, 0.0, 0.0)
    transform_geometry_001_1.inputs[2].default_value = (0.0, 0.0, 0.0)
    attribute_statistic_001 = lunarterrain.nodes.new("GeometryNodeAttributeStatistic")
    attribute_statistic_001.data_type = "FLOAT_VECTOR"
    attribute_statistic_001.domain = "POINT"
    attribute_statistic_001.inputs[1].hide = True
    attribute_statistic_001.outputs[0].hide = True
    attribute_statistic_001.outputs[1].hide = True
    attribute_statistic_001.outputs[2].hide = True
    attribute_statistic_001.outputs[3].hide = True
    attribute_statistic_001.outputs[4].hide = True
    attribute_statistic_001.outputs[6].hide = True
    attribute_statistic_001.outputs[7].hide = True
    attribute_statistic_001.inputs[1].default_value = True
    position_004_1 = lunarterrain.nodes.new("GeometryNodeInputPosition")
    reroute_007 = lunarterrain.nodes.new("NodeReroute")
    vector_math_028 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_028.operation = "DIVIDE"
    frame_003_2 = lunarterrain.nodes.new("NodeFrame")
    frame_003_2.shrink = True
    reroute_008 = lunarterrain.nodes.new("NodeReroute")
    reroute_006_1 = lunarterrain.nodes.new("NodeReroute")
    reroute_004_1 = lunarterrain.nodes.new("NodeReroute")
    noise_texture_009 = lunarterrain.nodes.new("ShaderNodeTexNoise")
    noise_texture_009.noise_dimensions = "4D"
    noise_texture_009.noise_type = "MULTIFRACTAL"
    noise_texture_009.normalize = True
    noise_texture_009.inputs[0].default_value = (0.0, 0.0, 0.0)
    noise_texture_009.inputs[8].default_value = 0.0
    group_013_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_013_1.node_tree = random__uniform_
    group_013_1.inputs[0].default_value = -100000000.0
    group_013_1.inputs[1].default_value = 1000000000.0
    group_013_1.inputs[3].default_value = 90878
    reroute_009 = lunarterrain.nodes.new("NodeReroute")
    group_2 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_2.node_tree = random__normal_
    group_2.inputs[0].default_value = True
    group_2.inputs[1].default_value = 0.15000000596046448
    group_2.inputs[2].default_value = 0.02500000037252903
    group_2.inputs[4].default_value = 53330
    group_014_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_014_1.node_tree = random__normal_
    group_014_1.inputs[0].default_value = True
    group_014_1.inputs[1].default_value = 4.0
    group_014_1.inputs[2].default_value = 0.20000000298023224
    group_014_1.inputs[4].default_value = 48802
    group_015 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_015.node_tree = random__normal_
    group_015.inputs[0].default_value = True
    group_015.inputs[1].default_value = 0.699999988079071
    group_015.inputs[2].default_value = 0.10000000149011612
    group_015.inputs[4].default_value = 99201
    group_016 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_016.node_tree = random__normal_
    group_016.inputs[0].default_value = True
    group_016.inputs[1].default_value = 2.200000047683716
    group_016.inputs[2].default_value = 0.07500000298023224
    group_016.inputs[4].default_value = 6506
    frame_004_1 = lunarterrain.nodes.new("NodeFrame")
    frame_004_1.shrink = True
    noise_texture_010 = lunarterrain.nodes.new("ShaderNodeTexNoise")
    noise_texture_010.noise_dimensions = "4D"
    noise_texture_010.noise_type = "HETERO_TERRAIN"
    noise_texture_010.normalize = True
    noise_texture_010.inputs[0].default_value = (0.0, 0.0, 0.0)
    noise_texture_010.inputs[3].default_value = 15.0
    noise_texture_010.inputs[8].default_value = 0.0
    group_017 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_017.node_tree = random__uniform_
    group_017.inputs[0].default_value = -100000000.0
    group_017.inputs[1].default_value = 1000000000.0
    group_017.inputs[3].default_value = 7859
    reroute_010_1 = lunarterrain.nodes.new("NodeReroute")
    group_018 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_018.node_tree = random__normal_
    group_018.inputs[0].default_value = True
    group_018.inputs[1].default_value = 1.5
    group_018.inputs[2].default_value = 0.25
    group_018.inputs[4].default_value = 543
    group_020 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_020.node_tree = random__normal_
    group_020.inputs[0].default_value = True
    group_020.inputs[1].default_value = 0.22499999403953552
    group_020.inputs[2].default_value = 0.02500000037252903
    group_020.inputs[4].default_value = 10032
    group_021 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_021.node_tree = random__normal_
    group_021.inputs[0].default_value = True
    group_021.inputs[1].default_value = 3.0
    group_021.inputs[2].default_value = 0.5
    group_021.inputs[4].default_value = 6515
    frame_005_1 = lunarterrain.nodes.new("NodeFrame")
    frame_005_1.shrink = True
    noise_texture_011_1 = lunarterrain.nodes.new("ShaderNodeTexNoise")
    noise_texture_011_1.noise_dimensions = "4D"
    noise_texture_011_1.noise_type = "FBM"
    noise_texture_011_1.normalize = True
    noise_texture_011_1.inputs[0].default_value = (0.0, 0.0, 0.0)
    noise_texture_011_1.inputs[3].default_value = 15.0
    noise_texture_011_1.inputs[8].default_value = 0.0
    group_019_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_019_1.node_tree = random__uniform_
    group_019_1.inputs[0].default_value = -100000000.0
    group_019_1.inputs[1].default_value = 1000000000.0
    group_019_1.inputs[3].default_value = 76322
    reroute_011 = lunarterrain.nodes.new("NodeReroute")
    group_022_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_022_1.node_tree = random__normal_
    group_022_1.inputs[0].default_value = True
    group_022_1.inputs[1].default_value = 2.0
    group_022_1.inputs[2].default_value = 0.10000000149011612
    group_022_1.inputs[4].default_value = 23556
    group_023_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_023_1.node_tree = random__normal_
    group_023_1.inputs[0].default_value = True
    group_023_1.inputs[1].default_value = 0.18000000715255737
    group_023_1.inputs[2].default_value = 0.03999999910593033
    group_023_1.inputs[4].default_value = 8479
    group_024_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_024_1.node_tree = random__normal_
    group_024_1.inputs[0].default_value = True
    group_024_1.inputs[1].default_value = 3.25
    group_024_1.inputs[2].default_value = 0.25
    group_024_1.inputs[4].default_value = 12594
    frame_006 = lunarterrain.nodes.new("NodeFrame")
    frame_006.shrink = True
    group_026 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_026.node_tree = random__normal_
    group_026.inputs[0].default_value = True
    group_026.inputs[1].default_value = 0.5
    group_026.inputs[2].default_value = 0.20000000298023224
    group_026.inputs[4].default_value = 521
    set_position_005 = lunarterrain.nodes.new("GeometryNodeSetPosition")
    set_position_005.inputs[1].hide = True
    set_position_005.inputs[2].hide = True
    set_position_005.inputs[1].default_value = True
    set_position_005.inputs[2].default_value = (0.0, 0.0, 0.0)
    math_002_2 = lunarterrain.nodes.new("ShaderNodeMath")
    math_002_2.operation = "ADD"
    math_002_2.use_clamp = False
    math_003_2 = lunarterrain.nodes.new("ShaderNodeMath")
    math_003_2.operation = "ADD"
    math_003_2.use_clamp = False
    combine_xyz_002 = lunarterrain.nodes.new("ShaderNodeCombineXYZ")
    combine_xyz_002.inputs[0].default_value = 0.0
    combine_xyz_002.inputs[1].default_value = 0.0
    vector = lunarterrain.nodes.new("FunctionNodeInputVector")
    vector.vector = (0.0, 0.0, 5.0)
    transform_geometry_1 = lunarterrain.nodes.new("GeometryNodeTransform")
    transform_geometry_1.mode = "COMPONENTS"
    transform_geometry_1.inputs[2].hide = True
    transform_geometry_1.inputs[3].hide = True
    transform_geometry_1.inputs[4].hide = True
    transform_geometry_1.inputs[2].default_value = (0.0, 0.0, 0.0)
    transform_geometry_1.inputs[3].default_value = (1.0, 1.0, 1.0)
    float_curve_1 = lunarterrain.nodes.new("ShaderNodeFloatCurve")
    float_curve_1.mapping.extend = "EXTRAPOLATED"
    float_curve_1.mapping.tone = "STANDARD"
    float_curve_1.mapping.black_level = (0.0, 0.0, 0.0)
    float_curve_1.mapping.white_level = (1.0, 1.0, 1.0)
    float_curve_1.mapping.clip_min_x = 0.0
    float_curve_1.mapping.clip_min_y = 0.0
    float_curve_1.mapping.clip_max_x = 1.0
    float_curve_1.mapping.clip_max_y = 1.0
    float_curve_1.mapping.use_clip = True
    float_curve_1_curve_0 = float_curve_1.mapping.curves[0]
    float_curve_1_curve_0_point_0 = float_curve_1_curve_0.points[0]
    float_curve_1_curve_0_point_0.location = (0.0, 1.0)
    float_curve_1_curve_0_point_0.handle_type = "AUTO"
    float_curve_1_curve_0_point_1 = float_curve_1_curve_0.points[1]
    float_curve_1_curve_0_point_1.location = (0.02500000037252903, 0.9750000238418579)
    float_curve_1_curve_0_point_1.handle_type = "AUTO"
    float_curve_1_curve_0_point_2 = float_curve_1_curve_0.points.new(
        0.5, 0.10000000149011612
    )
    float_curve_1_curve_0_point_2.handle_type = "AUTO_CLAMPED"
    float_curve_1_curve_0_point_3 = float_curve_1_curve_0.points.new(1.0, 0.0)
    float_curve_1_curve_0_point_3.handle_type = "AUTO"
    float_curve_1.mapping.update()
    float_curve_1.inputs[0].default_value = 1.0
    reroute_1 = lunarterrain.nodes.new("NodeReroute")
    frame_007 = lunarterrain.nodes.new("NodeFrame")
    frame_007.shrink = True
    reroute_012_1 = lunarterrain.nodes.new("NodeReroute")
    transform_geometry_002_1 = lunarterrain.nodes.new("GeometryNodeTransform")
    transform_geometry_002_1.mode = "COMPONENTS"
    transform_geometry_002_1.inputs[1].hide = True
    transform_geometry_002_1.inputs[2].hide = True
    transform_geometry_002_1.inputs[4].hide = True
    transform_geometry_002_1.inputs[1].default_value = (0.0, 0.0, 0.0)
    transform_geometry_002_1.inputs[2].default_value = (0.0, 0.0, 0.0)
    attribute_statistic_002 = lunarterrain.nodes.new("GeometryNodeAttributeStatistic")
    attribute_statistic_002.data_type = "FLOAT_VECTOR"
    attribute_statistic_002.domain = "POINT"
    attribute_statistic_002.inputs[1].hide = True
    attribute_statistic_002.outputs[0].hide = True
    attribute_statistic_002.outputs[1].hide = True
    attribute_statistic_002.outputs[2].hide = True
    attribute_statistic_002.outputs[3].hide = True
    attribute_statistic_002.outputs[4].hide = True
    attribute_statistic_002.outputs[6].hide = True
    attribute_statistic_002.outputs[7].hide = True
    attribute_statistic_002.inputs[1].default_value = True
    position_005 = lunarterrain.nodes.new("GeometryNodeInputPosition")
    reroute_013_1 = lunarterrain.nodes.new("NodeReroute")
    vector_math_030 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_030.operation = "DIVIDE"
    vector_math_030.inputs[0].default_value = (1.0, 1.0, 1.0)
    frame_008 = lunarterrain.nodes.new("NodeFrame")
    frame_008.shrink = True
    separate_xyz_001_1 = lunarterrain.nodes.new("ShaderNodeSeparateXYZ")
    separate_xyz_001_1.outputs[2].hide = True
    math_006_1 = lunarterrain.nodes.new("ShaderNodeMath")
    math_006_1.operation = "MULTIPLY"
    math_006_1.use_clamp = False
    math_009 = lunarterrain.nodes.new("ShaderNodeMath")
    math_009.operation = "MAXIMUM"
    math_009.use_clamp = False
    math_010_1 = lunarterrain.nodes.new("ShaderNodeMath")
    math_010_1.operation = "DIVIDE"
    math_010_1.use_clamp = False
    math_011 = lunarterrain.nodes.new("ShaderNodeMath")
    math_011.operation = "DIVIDE"
    math_011.use_clamp = False
    mix = lunarterrain.nodes.new("ShaderNodeMix")
    mix.blend_type = "MIX"
    mix.clamp_factor = True
    mix.clamp_result = False
    mix.data_type = "FLOAT"
    mix.factor_mode = "UNIFORM"
    mix.inputs[0].hide = True
    mix.inputs[1].hide = True
    mix.inputs[4].hide = True
    mix.inputs[5].hide = True
    mix.inputs[6].hide = True
    mix.inputs[7].hide = True
    mix.inputs[8].hide = True
    mix.inputs[9].hide = True
    mix.outputs[1].hide = True
    mix.outputs[2].hide = True
    mix.outputs[3].hide = True
    mix.inputs[0].default_value = 0.5
    math_007_1 = lunarterrain.nodes.new("ShaderNodeMath")
    math_007_1.operation = "MULTIPLY"
    math_007_1.use_clamp = False
    reroute_002_2 = lunarterrain.nodes.new("NodeReroute")
    reroute_014 = lunarterrain.nodes.new("NodeReroute")
    reroute_015_1 = lunarterrain.nodes.new("NodeReroute")
    group_002_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_002_1.node_tree = random__normal_
    group_002_1.inputs[0].default_value = True
    group_002_1.inputs[1].default_value = 0.6000000238418579
    group_002_1.inputs[2].default_value = 0.20000000298023224
    group_002_1.inputs[4].default_value = 65241
    group_003_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_003_1.node_tree = random__normal_
    group_003_1.inputs[0].default_value = True
    group_003_1.inputs[1].default_value = 0.3333333432674408
    group_003_1.inputs[2].default_value = 0.0833333358168602
    group_003_1.inputs[4].default_value = 87654
    group_004_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_004_1.node_tree = random__normal_
    group_004_1.inputs[0].default_value = True
    group_004_1.inputs[1].default_value = 0.8999999761581421
    group_004_1.inputs[2].default_value = 0.20000000298023224
    group_004_1.inputs[4].default_value = 521
    group_005_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_005_1.node_tree = random__normal_
    group_005_1.inputs[0].default_value = True
    group_005_1.inputs[1].default_value = 0.75
    group_005_1.inputs[2].default_value = 0.25
    group_005_1.inputs[4].default_value = 215
    reroute_016 = lunarterrain.nodes.new("NodeReroute")
    group_001_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_001_1.node_tree = lunarrock
    group_001_1.inputs[2].default_value = (1.0, 1.0, 1.0)
    group_001_1.inputs[3].default_value = (0.25, 0.25, 0.10000000149011612)
    group_001_1.inputs[4].default_value = False
    group_001_1.inputs[5].default_value = 0.0
    distribute_points_on_faces = lunarterrain.nodes.new(
        "GeometryNodeDistributePointsOnFaces"
    )
    distribute_points_on_faces.distribute_method = "POISSON"
    distribute_points_on_faces.use_legacy_normal = False
    distribute_points_on_faces.inputs[1].default_value = True
    distribute_points_on_faces.inputs[3].default_value = 0.5
    repeat_input = lunarterrain.nodes.new("GeometryNodeRepeatInput")
    repeat_output = lunarterrain.nodes.new("GeometryNodeRepeatOutput")
    repeat_output.active_index = 1
    repeat_output.inspection_index = 0
    repeat_output.repeat_items.clear()
    repeat_output.repeat_items.new("GEOMETRY", "Geometry")
    repeat_output.repeat_items.new("INT", "Point Index")
    math_004_2 = lunarterrain.nodes.new("ShaderNodeMath")
    math_004_2.operation = "ADD"
    math_004_2.use_clamp = False
    math_004_2.inputs[1].default_value = 1.0
    domain_size = lunarterrain.nodes.new("GeometryNodeAttributeDomainSize")
    domain_size.component = "POINTCLOUD"
    join_geometry_001 = lunarterrain.nodes.new("GeometryNodeJoinGeometry")
    join_geometry = lunarterrain.nodes.new("GeometryNodeJoinGeometry")
    sample_index = lunarterrain.nodes.new("GeometryNodeSampleIndex")
    sample_index.clamp = False
    sample_index.data_type = "FLOAT_VECTOR"
    sample_index.domain = "POINT"
    position_1 = lunarterrain.nodes.new("GeometryNodeInputPosition")
    transform_geometry_003_1 = lunarterrain.nodes.new("GeometryNodeTransform")
    transform_geometry_003_1.mode = "COMPONENTS"
    transform_geometry_003_1.inputs[2].hide = True
    transform_geometry_003_1.inputs[4].hide = True
    transform_geometry_003_1.inputs[2].default_value = (0.0, 0.0, 0.0)
    transform_geometry_003_1.inputs[3].default_value = (1.0, 1.0, 1.0)
    group_006_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_006_1.node_tree = random__uniform_
    group_006_1.inputs[0].default_value = 0.0
    group_006_1.inputs[1].default_value = 100000.0
    group_006_1.inputs[3].default_value = 434
    float_to_integer_002 = lunarterrain.nodes.new("FunctionNodeFloatToInt")
    float_to_integer_002.rounding_mode = "ROUND"
    math_005_2 = lunarterrain.nodes.new("ShaderNodeMath")
    math_005_2.operation = "ADD"
    math_005_2.use_clamp = False
    reroute_018_1 = lunarterrain.nodes.new("NodeReroute")
    reroute_019_1 = lunarterrain.nodes.new("NodeReroute")
    group_007_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_007_1.node_tree = random__uniform_
    group_007_1.inputs[0].default_value = 0.0
    group_007_1.inputs[1].default_value = 1.0
    group_007_1.inputs[3].default_value = 76543
    float_curve_001 = lunarterrain.nodes.new("ShaderNodeFloatCurve")
    float_curve_001.mapping.extend = "EXTRAPOLATED"
    float_curve_001.mapping.tone = "STANDARD"
    float_curve_001.mapping.black_level = (0.0, 0.0, 0.0)
    float_curve_001.mapping.white_level = (1.0, 1.0, 1.0)
    float_curve_001.mapping.clip_min_x = 0.0
    float_curve_001.mapping.clip_min_y = 0.0
    float_curve_001.mapping.clip_max_x = 1.0
    float_curve_001.mapping.clip_max_y = 1.0
    float_curve_001.mapping.use_clip = True
    float_curve_001_curve_0 = float_curve_001.mapping.curves[0]
    float_curve_001_curve_0_point_0 = float_curve_001_curve_0.points[0]
    float_curve_001_curve_0_point_0.location = (0.0, 0.019999999552965164)
    float_curve_001_curve_0_point_0.handle_type = "AUTO_CLAMPED"
    float_curve_001_curve_0_point_1 = float_curve_001_curve_0.points[1]
    float_curve_001_curve_0_point_1.location = (0.25, 0.019999999552965164)
    float_curve_001_curve_0_point_1.handle_type = "AUTO_CLAMPED"
    float_curve_001_curve_0_point_2 = float_curve_001_curve_0.points.new(
        0.949999988079071, 0.20000000298023224
    )
    float_curve_001_curve_0_point_2.handle_type = "AUTO_CLAMPED"
    float_curve_001_curve_0_point_3 = float_curve_001_curve_0.points.new(1.0, 1.0)
    float_curve_001_curve_0_point_3.handle_type = "AUTO_CLAMPED"
    float_curve_001.mapping.update()
    float_curve_001.inputs[0].default_value = 1.0
    delete_geometry_1 = lunarterrain.nodes.new("GeometryNodeDeleteGeometry")
    delete_geometry_1.domain = "POINT"
    delete_geometry_1.mode = "ALL"
    position_001_1 = lunarterrain.nodes.new("GeometryNodeInputPosition")
    compare_001_1 = lunarterrain.nodes.new("FunctionNodeCompare")
    compare_001_1.data_type = "FLOAT"
    compare_001_1.mode = "ELEMENT"
    compare_001_1.operation = "GREATER_THAN"
    separate_xyz_002_1 = lunarterrain.nodes.new("ShaderNodeSeparateXYZ")
    separate_xyz_002_1.outputs[2].hide = True
    compare_002 = lunarterrain.nodes.new("FunctionNodeCompare")
    compare_002.data_type = "FLOAT"
    compare_002.mode = "ELEMENT"
    compare_002.operation = "GREATER_THAN"
    math_008_1 = lunarterrain.nodes.new("ShaderNodeMath")
    math_008_1.operation = "ABSOLUTE"
    math_008_1.use_clamp = False
    math_012 = lunarterrain.nodes.new("ShaderNodeMath")
    math_012.operation = "ABSOLUTE"
    math_012.use_clamp = False
    boolean_math_1 = lunarterrain.nodes.new("FunctionNodeBooleanMath")
    boolean_math_1.operation = "OR"
    separate_xyz_003_1 = lunarterrain.nodes.new("ShaderNodeSeparateXYZ")
    separate_xyz_003_1.outputs[2].hide = True
    math_013 = lunarterrain.nodes.new("ShaderNodeMath")
    math_013.operation = "MULTIPLY"
    math_013.use_clamp = False
    math_013.inputs[1].default_value = 0.44999998807907104
    math_014 = lunarterrain.nodes.new("ShaderNodeMath")
    math_014.operation = "MULTIPLY"
    math_014.use_clamp = False
    math_014.inputs[1].default_value = 0.3499999940395355
    vector_math_1 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_1.operation = "ADD"
    frame_010 = lunarterrain.nodes.new("NodeFrame")
    frame_010.shrink = True
    frame_011 = lunarterrain.nodes.new("NodeFrame")
    frame_011.shrink = True
    frame_012 = lunarterrain.nodes.new("NodeFrame")
    frame_012.shrink = True
    frame_013 = lunarterrain.nodes.new("NodeFrame")
    frame_013.shrink = True
    frame_014 = lunarterrain.nodes.new("NodeFrame")
    frame_014.shrink = True
    frame_015 = lunarterrain.nodes.new("NodeFrame")
    frame_015.shrink = True
    math_016 = lunarterrain.nodes.new("ShaderNodeMath")
    math_016.operation = "MULTIPLY"
    math_016.use_clamp = False
    math_016.inputs[1].default_value = 1.7999999523162842
    position_006 = lunarterrain.nodes.new("GeometryNodeInputPosition")
    math_017 = lunarterrain.nodes.new("ShaderNodeMath")
    math_017.operation = "MULTIPLY"
    math_017.use_clamp = False
    vector_math_001_1 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_001_1.operation = "DISTANCE"
    math_018 = lunarterrain.nodes.new("ShaderNodeMath")
    math_018.operation = "DIVIDE"
    math_018.use_clamp = False
    math_019 = lunarterrain.nodes.new("ShaderNodeMath")
    math_019.operation = "SUBTRACT"
    math_019.use_clamp = False
    math_019.inputs[0].default_value = 1.0
    set_position_1 = lunarterrain.nodes.new("GeometryNodeSetPosition")
    set_position_1.inputs[3].hide = True
    set_position_1.inputs[3].default_value = (0.0, 0.0, 0.0)
    math_020 = lunarterrain.nodes.new("ShaderNodeMath")
    math_020.operation = "MULTIPLY"
    math_020.use_clamp = False
    math_021 = lunarterrain.nodes.new("ShaderNodeMath")
    math_021.operation = "MULTIPLY"
    math_021.use_clamp = False
    compare_003 = lunarterrain.nodes.new("FunctionNodeCompare")
    compare_003.data_type = "FLOAT"
    compare_003.mode = "ELEMENT"
    compare_003.operation = "LESS_THAN"
    compare_003.inputs[1].default_value = 1.0
    group_008_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_008_1.node_tree = crater_profile
    math_022 = lunarterrain.nodes.new("ShaderNodeMath")
    math_022.operation = "SUBTRACT"
    math_022.use_clamp = False
    group_009_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_009_1.node_tree = crater_profile
    group_009_1.inputs[0].default_value = 0.0
    distribute_points_on_faces_001 = lunarterrain.nodes.new(
        "GeometryNodeDistributePointsOnFaces"
    )
    distribute_points_on_faces_001.distribute_method = "POISSON"
    distribute_points_on_faces_001.use_legacy_normal = True
    distribute_points_on_faces_001.inputs[1].default_value = True
    distribute_points_on_faces_001.inputs[3].default_value = 10.0
    random_value_1 = lunarterrain.nodes.new("FunctionNodeRandomValue")
    random_value_1.data_type = "FLOAT"
    random_value_1.inputs[2].default_value = 0.0
    random_value_1.inputs[3].default_value = 1.0
    random_value_1.inputs[7].default_value = 0
    sample_index_001 = lunarterrain.nodes.new("GeometryNodeSampleIndex")
    sample_index_001.clamp = False
    sample_index_001.data_type = "FLOAT_VECTOR"
    sample_index_001.domain = "POINT"
    sample_nearest = lunarterrain.nodes.new("GeometryNodeSampleNearest")
    sample_nearest.domain = "POINT"
    sample_nearest.inputs[1].default_value = (0.0, 0.0, 0.0)
    sample_index_002 = lunarterrain.nodes.new("GeometryNodeSampleIndex")
    sample_index_002.clamp = False
    sample_index_002.data_type = "FLOAT"
    sample_index_002.domain = "POINT"
    sample_nearest_001 = lunarterrain.nodes.new("GeometryNodeSampleNearest")
    sample_nearest_001.domain = "POINT"
    sample_nearest_001.inputs[1].default_value = (0.0, 0.0, 0.0)
    mix_001 = lunarterrain.nodes.new("ShaderNodeMix")
    mix_001.blend_type = "MIX"
    mix_001.clamp_factor = True
    mix_001.clamp_result = False
    mix_001.data_type = "VECTOR"
    mix_001.factor_mode = "UNIFORM"
    mix_001.inputs[4].default_value = (0.0, 0.0, 1.0)
    combine_xyz_003 = lunarterrain.nodes.new("ShaderNodeCombineXYZ")
    combine_xyz_003.inputs[2].hide = True
    combine_xyz_003.inputs[2].default_value = 0.0
    separate_xyz_004 = lunarterrain.nodes.new("ShaderNodeSeparateXYZ")
    separate_xyz_004.outputs[2].hide = True
    combine_xyz_004 = lunarterrain.nodes.new("ShaderNodeCombineXYZ")
    combine_xyz_004.inputs[2].hide = True
    combine_xyz_004.inputs[2].default_value = 0.0
    separate_xyz_005 = lunarterrain.nodes.new("ShaderNodeSeparateXYZ")
    separate_xyz_005.outputs[2].hide = True
    math_023 = lunarterrain.nodes.new("ShaderNodeMath")
    math_023.operation = "ADD"
    math_023.use_clamp = False
    math_023.inputs[1].default_value = 1.0
    vector_math_003_1 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_003_1.operation = "MULTIPLY"
    sample_index_003 = lunarterrain.nodes.new("GeometryNodeSampleIndex")
    sample_index_003.clamp = False
    sample_index_003.data_type = "FLOAT_VECTOR"
    sample_index_003.domain = "POINT"
    string = lunarterrain.nodes.new("FunctionNodeInputString")
    string.string = "crater_normal"
    reroute_017_1 = lunarterrain.nodes.new("NodeReroute")
    reroute_020_1 = lunarterrain.nodes.new("NodeReroute")
    reroute_021_1 = lunarterrain.nodes.new("NodeReroute")
    reroute_022_1 = lunarterrain.nodes.new("NodeReroute")
    reroute_023 = lunarterrain.nodes.new("NodeReroute")
    store_named_attribute = lunarterrain.nodes.new("GeometryNodeStoreNamedAttribute")
    store_named_attribute.data_type = "FLOAT_VECTOR"
    store_named_attribute.domain = "POINT"
    store_named_attribute.inputs[1].default_value = True
    store_named_attribute_001 = lunarterrain.nodes.new(
        "GeometryNodeStoreNamedAttribute"
    )
    store_named_attribute_001.data_type = "FLOAT"
    store_named_attribute_001.domain = "POINT"
    store_named_attribute_001.inputs[1].default_value = True
    named_attribute = lunarterrain.nodes.new("GeometryNodeInputNamedAttribute")
    named_attribute.data_type = "FLOAT"
    named_attribute_001 = lunarterrain.nodes.new("GeometryNodeInputNamedAttribute")
    named_attribute_001.data_type = "FLOAT_VECTOR"
    string_001 = lunarterrain.nodes.new("FunctionNodeInputString")
    string_001.string = "crater_radius"
    float_curve_002 = lunarterrain.nodes.new("ShaderNodeFloatCurve")
    float_curve_002.mapping.extend = "EXTRAPOLATED"
    float_curve_002.mapping.tone = "STANDARD"
    float_curve_002.mapping.black_level = (0.0, 0.0, 0.0)
    float_curve_002.mapping.white_level = (1.0, 1.0, 1.0)
    float_curve_002.mapping.clip_min_x = 0.0
    float_curve_002.mapping.clip_min_y = 0.0
    float_curve_002.mapping.clip_max_x = 1.0
    float_curve_002.mapping.clip_max_y = 1.0
    float_curve_002.mapping.use_clip = True
    float_curve_002_curve_0 = float_curve_002.mapping.curves[0]
    float_curve_002_curve_0_point_0 = float_curve_002_curve_0.points[0]
    float_curve_002_curve_0_point_0.location = (0.0, 0.02500000037252903)
    float_curve_002_curve_0_point_0.handle_type = "AUTO"
    float_curve_002_curve_0_point_1 = float_curve_002_curve_0.points[1]
    float_curve_002_curve_0_point_1.location = (0.800000011920929, 0.30000001192092896)
    float_curve_002_curve_0_point_1.handle_type = "AUTO"
    float_curve_002_curve_0_point_2 = float_curve_002_curve_0.points.new(1.0, 1.0)
    float_curve_002_curve_0_point_2.handle_type = "AUTO"
    float_curve_002.mapping.update()
    float_curve_002.inputs[0].default_value = 1.0
    reroute_025 = lunarterrain.nodes.new("NodeReroute")
    reroute_026 = lunarterrain.nodes.new("NodeReroute")
    position_007 = lunarterrain.nodes.new("GeometryNodeInputPosition")
    vector_math_004_1 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_004_1.operation = "ADD"
    reroute_027 = lunarterrain.nodes.new("NodeReroute")
    math_024 = lunarterrain.nodes.new("ShaderNodeMath")
    math_024.operation = "MULTIPLY"
    math_024.use_clamp = False
    math_025 = lunarterrain.nodes.new("ShaderNodeMath")
    math_025.operation = "SUBTRACT"
    math_025.use_clamp = False
    math_025.inputs[0].default_value = 1.0
    frame_016 = lunarterrain.nodes.new("NodeFrame")
    frame_016.shrink = True
    frame_017 = lunarterrain.nodes.new("NodeFrame")
    frame_017.shrink = True
    reroute_028 = lunarterrain.nodes.new("NodeReroute")
    reroute_031 = lunarterrain.nodes.new("NodeReroute")
    reroute_033 = lunarterrain.nodes.new("NodeReroute")
    reroute_032 = lunarterrain.nodes.new("NodeReroute")
    reroute_029 = lunarterrain.nodes.new("NodeReroute")
    reroute_030 = lunarterrain.nodes.new("NodeReroute")
    frame_009 = lunarterrain.nodes.new("NodeFrame")
    frame_009.use_custom_color = True
    frame_009.color = (0.6079999804496765, 0.0, 0.014633849263191223)
    frame_009.shrink = True
    frame_018 = lunarterrain.nodes.new("NodeFrame")
    frame_018.use_custom_color = True
    frame_018.color = (0.6079999804496765, 0.0, 0.043328672647476196)
    frame_018.shrink = True
    reroute_024 = lunarterrain.nodes.new("NodeReroute")
    frame_019 = lunarterrain.nodes.new("NodeFrame")
    frame_019.shrink = True
    reroute_035 = lunarterrain.nodes.new("NodeReroute")
    boolean_math_001 = lunarterrain.nodes.new("FunctionNodeBooleanMath")
    boolean_math_001.operation = "OR"
    combine_xyz_005 = lunarterrain.nodes.new("ShaderNodeCombineXYZ")
    combine_xyz_005.inputs[2].hide = True
    combine_xyz_005.inputs[2].default_value = 0.0
    vector_math_005 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_005.operation = "LENGTH"
    compare_004 = lunarterrain.nodes.new("FunctionNodeCompare")
    compare_004.data_type = "FLOAT"
    compare_004.mode = "ELEMENT"
    compare_004.operation = "LESS_THAN"
    reroute_034 = lunarterrain.nodes.new("NodeReroute")
    mix_002 = lunarterrain.nodes.new("ShaderNodeMix")
    mix_002.blend_type = "MIX"
    mix_002.clamp_factor = True
    mix_002.clamp_result = False
    mix_002.data_type = "FLOAT"
    mix_002.factor_mode = "UNIFORM"
    mix_002.inputs[0].default_value = 0.5
    math_026 = lunarterrain.nodes.new("ShaderNodeMath")
    math_026.operation = "MULTIPLY"
    math_026.use_clamp = False
    group_010_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_010_1.node_tree = random__normal_
    group_010_1.inputs[0].default_value = True
    group_010_1.inputs[1].default_value = 0.10000000149011612
    group_010_1.inputs[2].default_value = 0.02500000037252903
    group_010_1.inputs[3].default_value = 0
    group_010_1.inputs[4].default_value = 87702
    math_027 = lunarterrain.nodes.new("ShaderNodeMath")
    math_027.operation = "MINIMUM"
    math_027.use_clamp = False
    frame_020 = lunarterrain.nodes.new("NodeFrame")
    frame_020.shrink = True
    math_028 = lunarterrain.nodes.new("ShaderNodeMath")
    math_028.operation = "MULTIPLY"
    math_028.use_clamp = False
    math_028.inputs[1].default_value = 0.125
    group_011_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_011_1.node_tree = random__normal_
    group_011_1.inputs[0].default_value = True
    group_011_1.inputs[1].default_value = 0.75
    group_011_1.inputs[2].default_value = 0.125
    group_011_1.inputs[4].default_value = 6543
    reroute_036 = lunarterrain.nodes.new("NodeReroute")
    math_029 = lunarterrain.nodes.new("ShaderNodeMath")
    math_029.operation = "MULTIPLY"
    math_029.use_clamp = False
    attribute_statistic_1 = lunarterrain.nodes.new("GeometryNodeAttributeStatistic")
    attribute_statistic_1.data_type = "FLOAT_VECTOR"
    attribute_statistic_1.domain = "POINT"
    attribute_statistic_1.inputs[1].hide = True
    attribute_statistic_1.outputs[0].hide = True
    attribute_statistic_1.outputs[1].hide = True
    attribute_statistic_1.outputs[2].hide = True
    attribute_statistic_1.outputs[3].hide = True
    attribute_statistic_1.outputs[4].hide = True
    attribute_statistic_1.outputs[6].hide = True
    attribute_statistic_1.outputs[7].hide = True
    attribute_statistic_1.inputs[1].default_value = True
    position_008 = lunarterrain.nodes.new("GeometryNodeInputPosition")
    separate_xyz_006 = lunarterrain.nodes.new("ShaderNodeSeparateXYZ")
    separate_xyz_006.outputs[0].hide = True
    separate_xyz_006.outputs[1].hide = True
    combine_xyz_006 = lunarterrain.nodes.new("ShaderNodeCombineXYZ")
    combine_xyz_006.inputs[0].hide = True
    combine_xyz_006.inputs[1].hide = True
    combine_xyz_006.inputs[0].default_value = 0.0
    combine_xyz_006.inputs[1].default_value = 0.0
    transform_geometry_004 = lunarterrain.nodes.new("GeometryNodeTransform")
    transform_geometry_004.mode = "COMPONENTS"
    transform_geometry_004.inputs[1].hide = True
    transform_geometry_004.inputs[2].hide = True
    transform_geometry_004.inputs[4].hide = True
    transform_geometry_004.inputs[1].default_value = (0.0, 0.0, 0.0)
    transform_geometry_004.inputs[2].default_value = (0.0, 0.0, 0.0)
    vector_math_006 = lunarterrain.nodes.new("ShaderNodeVectorMath")
    vector_math_006.operation = "SCALE"
    group_012_1 = lunarterrain.nodes.new("GeometryNodeGroup")
    group_012_1.node_tree = random__uniform_
    group_012_1.inputs[0].default_value = 0.07500000298023224
    group_012_1.inputs[1].default_value = 0.25
    group_012_1.inputs[3].default_value = 214126
    math_015 = lunarterrain.nodes.new("ShaderNodeMath")
    math_015.operation = "DIVIDE"
    math_015.use_clamp = False
    reroute_040 = lunarterrain.nodes.new("NodeReroute")
    reroute_038 = lunarterrain.nodes.new("NodeReroute")
    math_030 = lunarterrain.nodes.new("ShaderNodeMath")
    math_030.operation = "POWER"
    math_030.use_clamp = False
    math_030.inputs[1].default_value = 0.5
    float_to_integer_001 = lunarterrain.nodes.new("FunctionNodeFloatToInt")
    float_to_integer_001.rounding_mode = "FLOOR"
    mesh_boolean_1 = lunarterrain.nodes.new("GeometryNodeMeshBoolean")
    mesh_boolean_1.operation = "UNION"
    mesh_boolean_1.solver = "EXACT"
    mesh_boolean_1.inputs[2].default_value = False
    mesh_boolean_1.inputs[3].default_value = False
    switch_2 = lunarterrain.nodes.new("GeometryNodeSwitch")
    switch_2.input_type = "GEOMETRY"
    reroute_037 = lunarterrain.nodes.new("NodeReroute")
    reroute_039 = lunarterrain.nodes.new("NodeReroute")
    reroute_041 = lunarterrain.nodes.new("NodeReroute")
    value = lunarterrain.nodes.new("ShaderNodeValue")
    value.outputs[0].default_value = 2.0
    repeat_input.pair_with_output(repeat_output)
    repeat_input.inputs[2].default_value = 0
    grid.parent = frame_001_2
    vector_math_012.parent = frame_002_1
    raycast.parent = frame_002_1
    frame_002_1.parent = frame_017
    vector_math_017.parent = frame_2
    gradient_texture_001.parent = frame_2
    position_002_1.parent = frame_2
    vector_math_019.parent = frame_2
    set_position_001_1.parent = frame_2
    position_003.parent = frame_2
    combine_xyz_1.parent = frame_2
    math_3.parent = frame_2
    reroute_001_1.parent = frame_002_1
    vector_math_002_1.parent = frame_001_2
    vector_math_021.parent = frame_001_2
    separate_xyz_1.parent = frame_001_2
    vector_math_023.parent = frame_001_2
    frame_001_2.parent = frame_017
    reroute_003_2.parent = frame_2
    compare_1.parent = frame_2
    transform_geometry_001_1.parent = frame_003_2
    attribute_statistic_001.parent = frame_003_2
    position_004_1.parent = frame_003_2
    reroute_007.parent = frame_003_2
    vector_math_028.parent = frame_003_2
    frame_003_2.parent = frame_017
    noise_texture_009.parent = frame_004_1
    group_013_1.parent = frame_004_1
    reroute_009.parent = frame_004_1
    group_2.parent = frame_004_1
    group_014_1.parent = frame_004_1
    group_015.parent = frame_004_1
    group_016.parent = frame_004_1
    frame_004_1.parent = frame_007
    noise_texture_010.parent = frame_005_1
    group_017.parent = frame_005_1
    reroute_010_1.parent = frame_005_1
    group_018.parent = frame_005_1
    group_020.parent = frame_005_1
    group_021.parent = frame_005_1
    frame_005_1.parent = frame_007
    noise_texture_011_1.parent = frame_006
    group_019_1.parent = frame_006
    reroute_011.parent = frame_006
    group_022_1.parent = frame_006
    group_023_1.parent = frame_006
    group_024_1.parent = frame_006
    frame_006.parent = frame_007
    group_026.parent = frame_005_1
    set_position_005.parent = frame_007
    math_002_2.parent = frame_004_1
    math_003_2.parent = frame_005_1
    combine_xyz_002.parent = frame_007
    vector.parent = frame_002_1
    transform_geometry_1.parent = frame_002_1
    float_curve_1.parent = frame_2
    reroute_1.parent = frame_003_2
    frame_007.parent = frame_017
    reroute_012_1.parent = frame_007
    transform_geometry_002_1.parent = frame_008
    attribute_statistic_002.parent = frame_008
    position_005.parent = frame_008
    reroute_013_1.parent = frame_008
    vector_math_030.parent = frame_008
    frame_008.parent = frame_017
    separate_xyz_001_1.parent = frame_019
    math_006_1.parent = frame_019
    math_009.parent = frame_001_2
    math_010_1.parent = frame_001_2
    math_011.parent = frame_001_2
    mix.parent = frame_019
    math_007_1.parent = frame_014
    reroute_002_2.parent = frame_016
    reroute_015_1.parent = frame_006
    group_002_1.parent = frame_015
    group_003_1.parent = frame_019
    group_004_1.parent = frame_014
    group_005_1.parent = frame_014
    reroute_016.parent = frame_016
    group_001_1.parent = frame_011
    distribute_points_on_faces.parent = frame_010
    repeat_input.parent = frame_011
    repeat_output.parent = frame_011
    math_004_2.parent = frame_011
    domain_size.parent = frame_010
    join_geometry_001.parent = frame_011
    join_geometry.parent = frame_011
    sample_index.parent = frame_011
    position_1.parent = frame_011
    transform_geometry_003_1.parent = frame_011
    group_006_1.parent = frame_011
    float_to_integer_002.parent = frame_011
    math_005_2.parent = frame_011
    reroute_018_1.parent = frame_011
    reroute_019_1.parent = frame_011
    group_007_1.parent = frame_011
    float_curve_001.parent = frame_018
    delete_geometry_1.parent = frame_010
    position_001_1.parent = frame_010
    compare_001_1.parent = frame_010
    separate_xyz_002_1.parent = frame_010
    compare_002.parent = frame_010
    math_008_1.parent = frame_010
    math_012.parent = frame_010
    boolean_math_1.parent = frame_010
    separate_xyz_003_1.parent = frame_010
    math_013.parent = frame_010
    math_014.parent = frame_010
    vector_math_1.parent = frame_011
    frame_010.parent = frame_011
    frame_012.parent = frame_016
    frame_013.parent = frame_016
    frame_014.parent = frame_016
    frame_015.parent = frame_016
    math_016.parent = frame_015
    position_006.parent = frame_012
    math_017.parent = frame_012
    vector_math_001_1.parent = frame_012
    math_018.parent = frame_012
    math_019.parent = frame_012
    set_position_1.parent = frame_014
    math_020.parent = frame_014
    math_021.parent = frame_014
    compare_003.parent = frame_014
    group_008_1.parent = frame_013
    math_022.parent = frame_013
    group_009_1.parent = frame_013
    distribute_points_on_faces_001.parent = frame_015
    random_value_1.parent = frame_015
    sample_index_001.parent = frame_012
    sample_nearest.parent = frame_012
    sample_index_002.parent = frame_015
    sample_nearest_001.parent = frame_015
    mix_001.parent = frame_014
    combine_xyz_003.parent = frame_012
    separate_xyz_004.parent = frame_012
    combine_xyz_004.parent = frame_012
    separate_xyz_005.parent = frame_012
    math_023.parent = frame_015
    vector_math_003_1.parent = frame_014
    sample_index_003.parent = frame_015
    string.parent = frame_015
    reroute_017_1.parent = frame_012
    reroute_020_1.parent = frame_012
    reroute_021_1.parent = frame_016
    reroute_022_1.parent = frame_014
    reroute_023.parent = frame_016
    store_named_attribute.parent = frame_015
    store_named_attribute_001.parent = frame_015
    named_attribute.parent = frame_015
    named_attribute_001.parent = frame_015
    string_001.parent = frame_015
    float_curve_002.parent = frame_009
    reroute_025.parent = frame_013
    reroute_026.parent = frame_013
    position_007.parent = frame_014
    vector_math_004_1.parent = frame_014
    reroute_027.parent = frame_013
    math_024.parent = frame_014
    math_025.parent = frame_014
    reroute_028.parent = frame_016
    reroute_033.parent = frame_010
    reroute_032.parent = frame_010
    reroute_029.parent = frame_016
    reroute_030.parent = frame_016
    frame_009.parent = frame_015
    frame_018.parent = frame_011
    reroute_024.parent = frame_016
    frame_019.parent = frame_016
    reroute_035.parent = frame_014
    boolean_math_001.parent = frame_010
    combine_xyz_005.parent = frame_010
    vector_math_005.parent = frame_010
    compare_004.parent = frame_010
    mix_002.parent = frame_020
    math_026.parent = frame_020
    group_010_1.parent = frame_020
    math_027.parent = frame_020
    frame_020.parent = frame_010
    math_028.parent = frame_010
    group_011_1.parent = frame_010
    reroute_036.parent = frame_020
    math_029.parent = frame_011
    attribute_statistic_1.parent = frame_011
    position_008.parent = frame_011
    separate_xyz_006.parent = frame_011
    combine_xyz_006.parent = frame_011
    transform_geometry_004.parent = frame_011
    vector_math_006.parent = frame_011
    group_012_1.parent = frame_011
    math_015.parent = frame_011
    math_030.parent = frame_011
    float_to_integer_001.parent = frame_011
    mesh_boolean_1.parent = frame_011
    switch_2.parent = frame_011
    reroute_037.parent = frame_011
    value.parent = frame_020
    group_input_4.location = (-20139.841796875, -110.3683853149414)
    group_output_4.location = (2688.79150390625, 0.0)
    grid.location = (-3688.64208984375, 698.6746215820312)
    set_material_1.location = (2465.73583984375, 28.000001907348633)
    set_shade_smooth_1.location = (2275.73583984375, 40.0)
    vector_math_012.location = (1723.7470703125, 52.151824951171875)
    raycast.location = (1523.7470703125, 52.1068115234375)
    frame_002_1.location = (-19502.08203125, -285.6419982910156)
    vector_math_017.location = (1293.6964111328125, -877.6812133789062)
    gradient_texture_001.location = (623.6964111328125, -966.2945556640625)
    position_002_1.location = (243.6964111328125, -887.7945556640625)
    vector_math_019.location = (433.6964111328125, -964.2945556640625)
    set_position_001_1.location = (1483.6964111328125, -588.5987548828125)
    position_003.location = (1103.6964111328125, -902.2945556640625)
    combine_xyz_1.location = (1103.6964111328125, -967.2945556640625)
    math_3.location = (243.6964111328125, -952.7945556640625)
    frame_2.location = (449.989990234375, 577.474853515625)
    reroute_001_1.location = (1091.744873046875, 84.72909545898438)
    vector_math_002_1.location = (-5025.19091796875, 679.6746215820312)
    vector_math_021.location = (-4835.19091796875, 668.6746215820312)
    separate_xyz_1.location = (-4455.19091796875, 676.6746215820312)
    vector_math_023.location = (-4645.19091796875, 709.6746215820312)
    frame_001_2.location = (-17219.966796875, -823.490966796875)
    reroute_003_2.location = (183.4259033203125, -785.5428466796875)
    compare_1.location = (1293.6964111328125, -694.6812133789062)
    math_001_3.location = (-19907.025390625, -346.03912353515625)
    integer_012.location = (-20097.025390625, -424.5093994140625)
    reroute_005_1.location = (-19012.7734375, 629.6964111328125)
    float_to_integer_1.location = (-19717.025390625, -370.53912353515625)
    transform_geometry_001_1.location = (1809.3837890625, -34.2188835144043)
    attribute_statistic_001.location = (1429.3837890625, -261.3872375488281)
    position_004_1.location = (1239.3837890625, -356.2337951660156)
    reroute_007.location = (1271.6622314453125, -105.46165466308594)
    vector_math_028.location = (1619.3837890625, -188.43307495117188)
    frame_003_2.location = (-18598.47265625, -89.21844482421875)
    reroute_008.location = (-14253.6787109375, 629.6964111328125)
    reroute_006_1.location = (633.416015625, 491.5299987792969)
    reroute_004_1.location = (-19012.7734375, 547.1276245117188)
    noise_texture_009.location = (-5233.6884765625, -202.95066833496094)
    group_013_1.location = (-5423.6884765625, 128.54934692382812)
    reroute_009.location = (-5485.20556640625, -395.0369873046875)
    group_2.location = (-5423.6884765625, -49.450653076171875)
    group_014_1.location = (-5423.6884765625, -249.45065307617188)
    group_015.location = (-5423.6884765625, -449.4506530761719)
    group_016.location = (-5423.6884765625, -649.45068359375)
    frame_004_1.location = (-549.12451171875, 255.1417236328125)
    noise_texture_010.location = (-5233.6884765625, 35.4810791015625)
    group_017.location = (-5423.6884765625, 238.87225341796875)
    reroute_010_1.location = (-5485.20556640625, -156.605224609375)
    group_018.location = (-5423.6884765625, 60.87225341796875)
    group_020.location = (-5423.6884765625, -139.12774658203125)
    group_021.location = (-5423.6884765625, -339.12774658203125)
    frame_005_1.location = (-549.12451171875, -904.7757568359375)
    noise_texture_011_1.location = (-5233.6884765625, 35.4810791015625)
    group_019_1.location = (-5423.6884765625, 236.93341064453125)
    reroute_011.location = (-5485.20556640625, -156.605224609375)
    group_022_1.location = (-5423.6884765625, 58.93341064453125)
    group_023_1.location = (-5423.6884765625, -141.06658935546875)
    group_024_1.location = (-5423.6884765625, -341.06658935546875)
    frame_006.location = (-549.12451171875, -2146.79296875)
    group_026.location = (-5423.6884765625, -539.1277465820312)
    set_position_005.location = (-5050.517578125, 510.89593505859375)
    math_002_2.location = (-5043.6884765625, -267.95068359375)
    math_003_2.location = (-5043.6884765625, -40.5189208984375)
    combine_xyz_002.location = (-5240.517578125, 417.0015869140625)
    vector.location = (1343.6845703125, -151.84811401367188)
    transform_geometry_1.location = (1913.7470703125, 160.67318725585938)
    float_curve_1.location = (823.550048828125, -862.5306396484375)
    reroute_1.location = (1271.14111328125, -50.05318832397461)
    frame_007.location = (-14472.8359375, -634.546142578125)
    reroute_012_1.location = (-6169.27001953125, -1060.4754638671875)
    transform_geometry_002_1.location = (1809.3837890625, -34.2188835144043)
    attribute_statistic_002.location = (1429.3837890625, -261.3872375488281)
    position_005.location = (1239.3837890625, -356.2337951660156)
    reroute_013_1.location = (1269.0, -74.0)
    vector_math_030.location = (1619.3837890625, -188.43307495117188)
    frame_008.location = (-20460.8359375, -89.54608154296875)
    separate_xyz_001_1.location = (-12082.7216796875, -14.499267578125)
    math_006_1.location = (-11626.6884765625, 19.28466796875)
    math_009.location = (-4126.19921875, 820.5626220703125)
    math_010_1.location = (-3936.19921875, 943.5841064453125)
    math_011.location = (-3936.19921875, 780.5841064453125)
    mix.location = (-11892.7216796875, 11.000732421875)
    math_007_1.location = (2495.16748046875, -107.586181640625)
    reroute_002_2.location = (-12353.2109375, 603.22412109375)
    reroute_014.location = (-19012.7734375, 455.10919189453125)
    reroute_015_1.location = (-5028.4130859375, -0.2525634765625)
    group_002_1.location = (-1929.091796875, 647.364501953125)
    group_003_1.location = (-11892.7216796875, -153.999267578125)
    group_004_1.location = (2187.833984375, -187.9140625)
    group_005_1.location = (2333.65234375, -481.912841796875)
    reroute_016.location = (-5319.115234375, 507.653076171875)
    group_001_1.location = (-819.889404296875, -1910.8944091796875)
    distribute_points_on_faces.location = (-3653.6591796875, -1898.399169921875)
    repeat_input.location = (-2826.2900390625, -1771.716796875)
    repeat_output.location = (538.6717529296875, -1682.5692138671875)
    math_004_2.location = (-2622.460693359375, -1839.3734130859375)
    domain_size.location = (-2693.789794921875, -1445.665771484375)
    join_geometry_001.location = (350.13262939453125, -1712.322998046875)
    join_geometry.location = (719.3307495117188, -1364.4150390625)
    sample_index.location = (-1012.987060546875, -1539.5640869140625)
    position_1.location = (-1249.7659912109375, -1682.3516845703125)
    transform_geometry_003_1.location = (181.31292724609375, -1813.9224853515625)
    group_006_1.location = (-2622.460693359375, -2002.3734130859375)
    float_to_integer_002.location = (-2431.11474609375, -2116.7001953125)
    math_005_2.location = (-2257.52294921875, -2077.4814453125)
    reroute_018_1.location = (-1411.2899169921875, -1857.4189453125)
    reroute_019_1.location = (-2063.73583984375, -2188.722412109375)
    group_007_1.location = (-2035.07275390625, -2228.327880859375)
    float_curve_001.location = (-1372.047119140625, -2293.55224609375)
    delete_geometry_1.location = (-2879.869384765625, -1799.16943359375)
    position_001_1.location = (-4157.068359375, -1436.1982421875)
    compare_001_1.location = (-3443.21533203125, -1393.573486328125)
    separate_xyz_002_1.location = (-3988.1025390625, -1381.181640625)
    compare_002.location = (-3443.21533203125, -1556.573486328125)
    math_008_1.location = (-3635.16259765625, -1406.525146484375)
    math_012.location = (-3635.16259765625, -1265.525146484375)
    boolean_math_1.location = (-3274.486328125, -1445.961181640625)
    separate_xyz_003_1.location = (-4400.02490234375, -1621.657470703125)
    math_013.location = (-3635.16259765625, -1547.525146484375)
    math_014.location = (-3635.16259765625, -1710.525146484375)
    vector_math_1.location = (-16.22247314453125, -1776.3204345703125)
    frame_010.location = (-341.00390625, -319.9088134765625)
    frame_011.location = (-531.6533203125, 1641.8719482421875)
    frame_012.location = (-8167.3359375, -548.3059692382812)
    frame_013.location = (-8267.5791015625, -530.7792358398438)
    frame_014.location = (-8980.685546875, -411.666748046875)
    frame_015.location = (-9216.7783203125, -706.154541015625)
    math_016.location = (-1929.228515625, 811.864990234375)
    position_006.location = (-908.1362915039062, -275.6748046875)
    math_017.location = (6.1168670654296875, -261.18353271484375)
    vector_math_001_1.location = (6.1168670654296875, -123.18353271484375)
    math_018.location = (222.644775390625, -51.94435119628906)
    math_019.location = (435.18646240234375, -276.8241271972656)
    set_position_1.location = (3694.345703125, 720.9320068359375)
    math_020.location = (2582.36279296875, -366.502197265625)
    math_021.location = (2766.23486328125, -300.075927734375)
    compare_003.location = (2956.23486328125, -148.575927734375)
    group_008_1.location = (872.8798828125, -219.51327514648438)
    math_022.location = (1104.73193359375, -328.0132751464844)
    group_009_1.location = (878.6954956054688, -402.7296142578125)
    distribute_points_on_faces_001.location = (-1553.310546875, 911.906494140625)
    random_value_1.location = (-1399.2080078125, 504.205078125)
    sample_index_001.location = (-576.4902954101562, -264.4415283203125)
    sample_nearest.location = (-908.1362915039062, -139.67478942871094)
    sample_index_002.location = (-196.78634643554688, 490.788818359375)
    sample_nearest_001.location = (-196.78634643554688, 282.788818359375)
    mix_001.location = (2766.23486328125, -463.075927734375)
    combine_xyz_003.location = (-195.4925537109375, -102.58157348632812)
    separate_xyz_004.location = (-385.4925537109375, -102.58157348632812)
    combine_xyz_004.location = (-195.4925537109375, -216.58157348632812)
    separate_xyz_005.location = (-385.4925537109375, -216.58157348632812)
    math_023.location = (-1613.025390625, 500.482666015625)
    vector_math_003_1.location = (2957.271484375, -329.651611328125)
    sample_index_003.location = (-196.78634643554688, 698.788818359375)
    string.location = (-766.786376953125, 641.788818359375)
    reroute_017_1.location = (-958.184814453125, -57.933448791503906)
    reroute_020_1.location = (-671.9747314453125, -63.218414306640625)
    reroute_021_1.location = (-8019.77734375, -315.61285400390625)
    reroute_022_1.location = (2671.4296875, 329.2344970703125)
    reroute_023.location = (-6974.6123046875, -337.19195556640625)
    store_named_attribute.location = (-576.786376953125, 697.288818359375)
    store_named_attribute_001.location = (-576.786376953125, 489.288818359375)
    named_attribute.location = (-386.786376953125, 454.788818359375)
    named_attribute_001.location = (-386.786376953125, 662.788818359375)
    string_001.location = (-766.786376953125, 433.788818359375)
    float_curve_002.location = (-1177.958984375, 551.424072265625)
    reroute_025.location = (815.8261108398438, -352.51995849609375)
    reroute_026.location = (815.8261108398438, -404.6067810058594)
    position_007.location = (3121.39794921875, -235.381591796875)
    vector_math_004_1.location = (3292.11669921875, -324.158935546875)
    reroute_027.location = (815.8261108398438, -459.7581481933594)
    math_024.location = (2592.224609375, -553.886962890625)
    math_025.location = (2122.2080078125, -639.906982421875)
    frame_016.location = (-392.5830078125, -138.754150390625)
    frame_017.location = (3337.47265625, 39.33642578125)
    reroute_028.location = (-12313.919921875, 232.63449096679688)
    reroute_031.location = (-4957.95703125, 105.62429809570312)
    reroute_033.location = (-3918.189453125, -936.4093627929688)
    reroute_032.location = (-4368.0498046875, -951.0938720703125)
    reroute_029.location = (-10929.2783203125, 576.5614013671875)
    reroute_030.location = (-7592.81689453125, 535.5804443359375)
    frame_009.location = (0.0, 0.0)
    frame_018.location = (-436.04638671875, -65.78271484375)
    reroute_024.location = (-8356.40625, 81.64697265625)
    frame_019.location = (0.0, 0.0)
    reroute_035.location = (2423.4267578125, 451.5533447265625)
    boolean_math_001.location = (-3093.355712890625, -1361.449951171875)
    combine_xyz_005.location = (-3639.12841796875, -1051.4248046875)
    vector_math_005.location = (-3448.10986328125, -1047.768310546875)
    compare_004.location = (-3235.63427734375, -1000.1389770507812)
    reroute_034.location = (-4273.96142578125, 492.3140563964844)
    mix_002.location = (-4238.23583984375, -1766.8883056640625)
    math_026.location = (-4076.0029296875, -1815.3262939453125)
    group_010_1.location = (-4249.86669921875, -1967.1490478515625)
    math_027.location = (-3915.177734375, -1962.415283203125)
    frame_020.location = (-90.3408203125, -2.58349609375)
    math_028.location = (-3836.5595703125, -2019.7752685546875)
    group_011_1.location = (-3877.59326171875, -1768.8453369140625)
    reroute_036.location = (-3849.791015625, -1913.984619140625)
    math_029.location = (-1449.9278564453125, -2302.19384765625)
    attribute_statistic_1.location = (-820.4967041015625, -1737.9093017578125)
    position_008.location = (-994.4837646484375, -1818.0142822265625)
    separate_xyz_006.location = (-626.9385986328125, -1786.962646484375)
    combine_xyz_006.location = (-456.7769775390625, -1812.57177734375)
    transform_geometry_004.location = (-70.228759765625, -2040.658935546875)
    vector_math_006.location = (-241.3167724609375, -1806.5072021484375)
    group_012_1.location = (-624.1229248046875, -1904.078857421875)
    math_015.location = (-1343.6787109375, -2083.6689453125)
    reroute_040.location = (-19012.7734375, 510.05523681640625)
    reroute_038.location = (-2611.3583984375, 618.0911865234375)
    math_030.location = (-1180.303955078125, -2087.87548828125)
    float_to_integer_001.location = (-993.7366943359375, -2032.217529296875)
    mesh_boolean_1.location = (728.8812255859375, -1529.47021484375)
    switch_2.location = (926.2166748046875, -1416.4339599609375)
    reroute_037.location = (561.221435546875, -1418.1854248046875)
    reroute_039.location = (-19012.7734375, 690.58740234375)
    reroute_041.location = (264.3801574707031, 689.6874389648438)
    value.location = (-4086.73583984375, -2041.174560546875)
    lunarterrain.links.new(set_material_1.outputs[0], group_output_4.inputs[0])
    lunarterrain.links.new(set_shade_smooth_1.outputs[0], set_material_1.inputs[0])
    lunarterrain.links.new(reroute_001_1.outputs[0], raycast.inputs[0])
    lunarterrain.links.new(raycast.outputs[1], vector_math_012.inputs[0])
    lunarterrain.links.new(vector_math_019.outputs[0], gradient_texture_001.inputs[0])
    lunarterrain.links.new(position_002_1.outputs[0], vector_math_019.inputs[0])
    lunarterrain.links.new(combine_xyz_1.outputs[0], vector_math_017.inputs[1])
    lunarterrain.links.new(position_003.outputs[0], vector_math_017.inputs[0])
    lunarterrain.links.new(float_curve_1.outputs[0], combine_xyz_1.inputs[2])
    lunarterrain.links.new(group_input_4.outputs[1], vector_math_002_1.inputs[0])
    lunarterrain.links.new(group_input_4.outputs[2], vector_math_002_1.inputs[1])
    lunarterrain.links.new(vector_math_002_1.outputs[0], vector_math_021.inputs[0])
    lunarterrain.links.new(separate_xyz_1.outputs[0], grid.inputs[2])
    lunarterrain.links.new(separate_xyz_1.outputs[1], grid.inputs[3])
    lunarterrain.links.new(vector_math_021.outputs[0], vector_math_023.inputs[0])
    lunarterrain.links.new(vector_math_023.outputs[0], separate_xyz_1.inputs[0])
    lunarterrain.links.new(reroute_003_2.outputs[0], math_3.inputs[0])
    lunarterrain.links.new(reroute_006_1.outputs[0], reroute_003_2.inputs[0])
    lunarterrain.links.new(reroute_003_2.outputs[0], compare_1.inputs[0])
    lunarterrain.links.new(compare_1.outputs[0], set_position_001_1.inputs[1])
    lunarterrain.links.new(integer_012.outputs[0], math_001_3.inputs[1])
    lunarterrain.links.new(group_input_4.outputs[0], math_001_3.inputs[0])
    lunarterrain.links.new(group_input_4.outputs[1], reroute_005_1.inputs[0])
    lunarterrain.links.new(math_001_3.outputs[0], float_to_integer_1.inputs[0])
    lunarterrain.links.new(position_004_1.outputs[0], attribute_statistic_001.inputs[2])
    lunarterrain.links.new(reroute_007.outputs[0], attribute_statistic_001.inputs[0])
    lunarterrain.links.new(
        vector_math_028.outputs[0], transform_geometry_001_1.inputs[3]
    )
    lunarterrain.links.new(
        attribute_statistic_001.outputs[5], vector_math_028.inputs[1]
    )
    lunarterrain.links.new(reroute_1.outputs[0], vector_math_028.inputs[0])
    lunarterrain.links.new(reroute_005_1.outputs[0], reroute_008.inputs[0])
    lunarterrain.links.new(reroute_034.outputs[0], reroute_006_1.inputs[0])
    lunarterrain.links.new(group_input_4.outputs[3], reroute_004_1.inputs[0])
    lunarterrain.links.new(group_013_1.outputs[0], noise_texture_009.inputs[1])
    lunarterrain.links.new(reroute_012_1.outputs[0], reroute_009.inputs[0])
    lunarterrain.links.new(reroute_009.outputs[0], group_013_1.inputs[2])
    lunarterrain.links.new(reroute_009.outputs[0], group_2.inputs[3])
    lunarterrain.links.new(group_2.outputs[0], noise_texture_009.inputs[2])
    lunarterrain.links.new(reroute_009.outputs[0], group_014_1.inputs[3])
    lunarterrain.links.new(group_014_1.outputs[0], noise_texture_009.inputs[3])
    lunarterrain.links.new(reroute_009.outputs[0], group_015.inputs[3])
    lunarterrain.links.new(group_015.outputs[0], noise_texture_009.inputs[4])
    lunarterrain.links.new(reroute_009.outputs[0], group_016.inputs[3])
    lunarterrain.links.new(group_016.outputs[0], noise_texture_009.inputs[5])
    lunarterrain.links.new(group_017.outputs[0], noise_texture_010.inputs[1])
    lunarterrain.links.new(reroute_010_1.outputs[0], group_017.inputs[2])
    lunarterrain.links.new(reroute_010_1.outputs[0], group_018.inputs[3])
    lunarterrain.links.new(group_018.outputs[0], noise_texture_010.inputs[2])
    lunarterrain.links.new(reroute_010_1.outputs[0], group_020.inputs[3])
    lunarterrain.links.new(group_020.outputs[0], noise_texture_010.inputs[4])
    lunarterrain.links.new(reroute_010_1.outputs[0], group_021.inputs[3])
    lunarterrain.links.new(group_021.outputs[0], noise_texture_010.inputs[5])
    lunarterrain.links.new(reroute_012_1.outputs[0], reroute_010_1.inputs[0])
    lunarterrain.links.new(group_019_1.outputs[0], noise_texture_011_1.inputs[1])
    lunarterrain.links.new(reroute_011.outputs[0], group_019_1.inputs[2])
    lunarterrain.links.new(reroute_011.outputs[0], group_022_1.inputs[3])
    lunarterrain.links.new(group_022_1.outputs[0], noise_texture_011_1.inputs[2])
    lunarterrain.links.new(reroute_011.outputs[0], group_023_1.inputs[3])
    lunarterrain.links.new(group_023_1.outputs[0], noise_texture_011_1.inputs[4])
    lunarterrain.links.new(reroute_011.outputs[0], group_024_1.inputs[3])
    lunarterrain.links.new(group_024_1.outputs[0], noise_texture_011_1.inputs[5])
    lunarterrain.links.new(reroute_012_1.outputs[0], reroute_011.inputs[0])
    lunarterrain.links.new(reroute_010_1.outputs[0], group_026.inputs[3])
    lunarterrain.links.new(group_026.outputs[0], noise_texture_010.inputs[6])
    lunarterrain.links.new(grid.outputs[0], set_position_005.inputs[0])
    lunarterrain.links.new(noise_texture_009.outputs[0], math_002_2.inputs[0])
    lunarterrain.links.new(noise_texture_010.outputs[0], math_003_2.inputs[0])
    lunarterrain.links.new(reroute_015_1.outputs[0], math_003_2.inputs[1])
    lunarterrain.links.new(math_003_2.outputs[0], math_002_2.inputs[1])
    lunarterrain.links.new(math_002_2.outputs[0], combine_xyz_002.inputs[2])
    lunarterrain.links.new(combine_xyz_002.outputs[0], set_position_005.inputs[3])
    lunarterrain.links.new(vector.outputs[0], raycast.inputs[2])
    lunarterrain.links.new(reroute_007.outputs[0], transform_geometry_001_1.inputs[0])
    lunarterrain.links.new(vector_math_012.outputs[0], transform_geometry_1.inputs[1])
    lunarterrain.links.new(reroute_001_1.outputs[0], transform_geometry_1.inputs[0])
    lunarterrain.links.new(vector_math_017.outputs[0], set_position_001_1.inputs[2])
    lunarterrain.links.new(gradient_texture_001.outputs[1], float_curve_1.inputs[1])
    lunarterrain.links.new(math_3.outputs[0], vector_math_019.inputs[1])
    lunarterrain.links.new(reroute_008.outputs[0], reroute_1.inputs[0])
    lunarterrain.links.new(float_to_integer_1.outputs[0], reroute_012_1.inputs[0])
    lunarterrain.links.new(position_005.outputs[0], attribute_statistic_002.inputs[2])
    lunarterrain.links.new(reroute_013_1.outputs[0], attribute_statistic_002.inputs[0])
    lunarterrain.links.new(
        vector_math_030.outputs[0], transform_geometry_002_1.inputs[3]
    )
    lunarterrain.links.new(
        attribute_statistic_002.outputs[5], vector_math_030.inputs[1]
    )
    lunarterrain.links.new(reroute_013_1.outputs[0], transform_geometry_002_1.inputs[0])
    lunarterrain.links.new(set_position_005.outputs[0], reroute_013_1.inputs[0])
    lunarterrain.links.new(transform_geometry_002_1.outputs[0], reroute_001_1.inputs[0])
    lunarterrain.links.new(transform_geometry_1.outputs[0], reroute_007.inputs[0])
    lunarterrain.links.new(reroute_008.outputs[0], separate_xyz_001_1.inputs[0])
    lunarterrain.links.new(separate_xyz_1.outputs[0], math_009.inputs[0])
    lunarterrain.links.new(separate_xyz_1.outputs[1], math_009.inputs[1])
    lunarterrain.links.new(separate_xyz_1.outputs[0], math_010_1.inputs[0])
    lunarterrain.links.new(math_009.outputs[0], math_010_1.inputs[1])
    lunarterrain.links.new(math_009.outputs[0], math_011.inputs[1])
    lunarterrain.links.new(separate_xyz_1.outputs[1], math_011.inputs[0])
    lunarterrain.links.new(math_010_1.outputs[0], grid.inputs[0])
    lunarterrain.links.new(math_011.outputs[0], grid.inputs[1])
    lunarterrain.links.new(separate_xyz_001_1.outputs[0], mix.inputs[2])
    lunarterrain.links.new(separate_xyz_001_1.outputs[1], mix.inputs[3])
    lunarterrain.links.new(mix.outputs[0], math_006_1.inputs[0])
    lunarterrain.links.new(reroute_035.outputs[0], math_007_1.inputs[0])
    lunarterrain.links.new(reroute_014.outputs[0], reroute_002_2.inputs[0])
    lunarterrain.links.new(float_to_integer_1.outputs[0], reroute_014.inputs[0])
    lunarterrain.links.new(noise_texture_011_1.outputs[0], reroute_015_1.inputs[0])
    lunarterrain.links.new(reroute_002_2.outputs[0], group_002_1.inputs[3])
    lunarterrain.links.new(reroute_002_2.outputs[0], group_003_1.inputs[3])
    lunarterrain.links.new(group_003_1.outputs[0], math_006_1.inputs[1])
    lunarterrain.links.new(reroute_002_2.outputs[0], group_004_1.inputs[3])
    lunarterrain.links.new(group_004_1.outputs[0], math_007_1.inputs[1])
    lunarterrain.links.new(reroute_002_2.outputs[0], group_005_1.inputs[3])
    lunarterrain.links.new(set_position_001_1.outputs[0], set_shade_smooth_1.inputs[0])
    lunarterrain.links.new(compare_001_1.outputs[0], boolean_math_1.inputs[0])
    lunarterrain.links.new(compare_002.outputs[0], boolean_math_1.inputs[1])
    lunarterrain.links.new(group_006_1.outputs[0], float_to_integer_002.inputs[0])
    lunarterrain.links.new(math_013.outputs[0], compare_001_1.inputs[1])
    lunarterrain.links.new(float_to_integer_002.outputs[0], math_005_2.inputs[1])
    lunarterrain.links.new(repeat_output.outputs[0], join_geometry.inputs[0])
    lunarterrain.links.new(separate_xyz_003_1.outputs[0], math_013.inputs[0])
    lunarterrain.links.new(delete_geometry_1.outputs[0], sample_index.inputs[0])
    lunarterrain.links.new(separate_xyz_003_1.outputs[1], math_014.inputs[0])
    lunarterrain.links.new(math_004_2.outputs[0], reroute_018_1.inputs[0])
    lunarterrain.links.new(math_014.outputs[0], compare_002.inputs[1])
    lunarterrain.links.new(math_005_2.outputs[0], reroute_019_1.inputs[0])
    lunarterrain.links.new(math_008_1.outputs[0], compare_002.inputs[0])
    lunarterrain.links.new(math_004_2.outputs[0], math_005_2.inputs[0])
    lunarterrain.links.new(repeat_input.outputs[1], math_004_2.inputs[0])
    lunarterrain.links.new(reroute_019_1.outputs[0], group_007_1.inputs[2])
    lunarterrain.links.new(reroute_019_1.outputs[0], group_001_1.inputs[0])
    lunarterrain.links.new(delete_geometry_1.outputs[0], domain_size.inputs[0])
    lunarterrain.links.new(group_007_1.outputs[0], float_curve_001.inputs[1])
    lunarterrain.links.new(
        distribute_points_on_faces.outputs[0], delete_geometry_1.inputs[0]
    )
    lunarterrain.links.new(join_geometry_001.outputs[0], repeat_output.inputs[0])
    lunarterrain.links.new(domain_size.outputs[0], repeat_input.inputs[0])
    lunarterrain.links.new(position_001_1.outputs[0], separate_xyz_002_1.inputs[0])
    lunarterrain.links.new(reroute_018_1.outputs[0], repeat_output.inputs[1])
    lunarterrain.links.new(math_012.outputs[0], compare_001_1.inputs[0])
    lunarterrain.links.new(reroute_018_1.outputs[0], sample_index.inputs[2])
    lunarterrain.links.new(position_1.outputs[0], sample_index.inputs[1])
    lunarterrain.links.new(separate_xyz_002_1.outputs[0], math_012.inputs[0])
    lunarterrain.links.new(separate_xyz_002_1.outputs[1], math_008_1.inputs[0])
    lunarterrain.links.new(repeat_input.outputs[0], join_geometry_001.inputs[0])
    lunarterrain.links.new(reroute_036.outputs[0], distribute_points_on_faces.inputs[6])
    lunarterrain.links.new(reroute_031.outputs[0], group_006_1.inputs[2])
    lunarterrain.links.new(reroute_032.outputs[0], separate_xyz_003_1.inputs[0])
    lunarterrain.links.new(switch_2.outputs[0], set_position_001_1.inputs[0])
    lunarterrain.links.new(math_018.outputs[0], compare_003.inputs[0])
    lunarterrain.links.new(math_018.outputs[0], math_019.inputs[1])
    lunarterrain.links.new(math_019.outputs[0], group_008_1.inputs[0])
    lunarterrain.links.new(position_006.outputs[0], sample_index_001.inputs[1])
    lunarterrain.links.new(math_017.outputs[0], math_018.inputs[1])
    lunarterrain.links.new(math_022.outputs[0], math_020.inputs[0])
    lunarterrain.links.new(reroute_023.outputs[0], math_020.inputs[1])
    lunarterrain.links.new(vector_math_001_1.outputs[1], math_018.inputs[0])
    lunarterrain.links.new(reroute_021_1.outputs[0], math_017.inputs[0])
    lunarterrain.links.new(math_020.outputs[0], math_021.inputs[0])
    lunarterrain.links.new(
        math_016.outputs[0], distribute_points_on_faces_001.inputs[2]
    )
    lunarterrain.links.new(group_009_1.outputs[0], math_022.inputs[1])
    lunarterrain.links.new(reroute_020_1.outputs[0], sample_index_001.inputs[0])
    lunarterrain.links.new(reroute_017_1.outputs[0], sample_nearest.inputs[0])
    lunarterrain.links.new(sample_nearest.outputs[0], sample_index_001.inputs[2])
    lunarterrain.links.new(sample_nearest_001.outputs[0], sample_index_002.inputs[2])
    lunarterrain.links.new(compare_003.outputs[0], set_position_1.inputs[1])
    lunarterrain.links.new(group_008_1.outputs[0], math_022.inputs[0])
    lunarterrain.links.new(separate_xyz_004.outputs[0], combine_xyz_003.inputs[0])
    lunarterrain.links.new(separate_xyz_004.outputs[1], combine_xyz_003.inputs[1])
    lunarterrain.links.new(combine_xyz_003.outputs[0], vector_math_001_1.inputs[0])
    lunarterrain.links.new(separate_xyz_005.outputs[0], combine_xyz_004.inputs[0])
    lunarterrain.links.new(separate_xyz_005.outputs[1], combine_xyz_004.inputs[1])
    lunarterrain.links.new(sample_index_001.outputs[0], separate_xyz_005.inputs[0])
    lunarterrain.links.new(combine_xyz_004.outputs[0], vector_math_001_1.inputs[1])
    lunarterrain.links.new(position_006.outputs[0], separate_xyz_004.inputs[0])
    lunarterrain.links.new(math_023.outputs[0], random_value_1.inputs[8])
    lunarterrain.links.new(sample_nearest_001.outputs[0], sample_index_003.inputs[2])
    lunarterrain.links.new(math_021.outputs[0], vector_math_003_1.inputs[0])
    lunarterrain.links.new(mix_001.outputs[1], vector_math_003_1.inputs[1])
    lunarterrain.links.new(reroute_017_1.outputs[0], reroute_020_1.inputs[0])
    lunarterrain.links.new(sample_index_002.outputs[0], reroute_021_1.inputs[0])
    lunarterrain.links.new(sample_index_003.outputs[0], reroute_022_1.inputs[0])
    lunarterrain.links.new(reroute_021_1.outputs[0], reroute_023.inputs[0])
    lunarterrain.links.new(
        distribute_points_on_faces_001.outputs[0], reroute_017_1.inputs[0]
    )
    lunarterrain.links.new(
        distribute_points_on_faces_001.outputs[0], sample_nearest_001.inputs[0]
    )
    lunarterrain.links.new(
        store_named_attribute_001.outputs[0], sample_index_002.inputs[0]
    )
    lunarterrain.links.new(store_named_attribute.outputs[0], sample_index_003.inputs[0])
    lunarterrain.links.new(
        distribute_points_on_faces_001.outputs[0], store_named_attribute_001.inputs[0]
    )
    lunarterrain.links.new(
        distribute_points_on_faces_001.outputs[0], store_named_attribute.inputs[0]
    )
    lunarterrain.links.new(string_001.outputs[0], store_named_attribute_001.inputs[2])
    lunarterrain.links.new(string_001.outputs[0], named_attribute.inputs[0])
    lunarterrain.links.new(named_attribute.outputs[0], sample_index_002.inputs[1])
    lunarterrain.links.new(string.outputs[0], store_named_attribute.inputs[2])
    lunarterrain.links.new(string.outputs[0], named_attribute_001.inputs[0])
    lunarterrain.links.new(named_attribute_001.outputs[0], sample_index_003.inputs[1])
    lunarterrain.links.new(
        distribute_points_on_faces_001.outputs[1], store_named_attribute.inputs[3]
    )
    lunarterrain.links.new(random_value_1.outputs[1], float_curve_002.inputs[1])
    lunarterrain.links.new(
        float_curve_002.outputs[0], store_named_attribute_001.inputs[3]
    )
    lunarterrain.links.new(reroute_025.outputs[0], group_008_1.inputs[1])
    lunarterrain.links.new(reroute_025.outputs[0], group_009_1.inputs[1])
    lunarterrain.links.new(reroute_021_1.outputs[0], reroute_025.inputs[0])
    lunarterrain.links.new(reroute_022_1.outputs[0], mix_001.inputs[5])
    lunarterrain.links.new(reroute_026.outputs[0], group_009_1.inputs[3])
    lunarterrain.links.new(reroute_026.outputs[0], group_008_1.inputs[3])
    lunarterrain.links.new(position_007.outputs[0], vector_math_004_1.inputs[0])
    lunarterrain.links.new(vector_math_003_1.outputs[0], vector_math_004_1.inputs[1])
    lunarterrain.links.new(vector_math_004_1.outputs[0], set_position_1.inputs[2])
    lunarterrain.links.new(reroute_027.outputs[0], group_008_1.inputs[2])
    lunarterrain.links.new(reroute_027.outputs[0], group_009_1.inputs[2])
    lunarterrain.links.new(math_024.outputs[0], mix_001.inputs[0])
    lunarterrain.links.new(math_025.outputs[0], math_024.inputs[1])
    lunarterrain.links.new(reroute_023.outputs[0], math_025.inputs[1])
    lunarterrain.links.new(math_006_1.outputs[0], math_016.inputs[0])
    lunarterrain.links.new(math_007_1.outputs[0], math_021.inputs[1])
    lunarterrain.links.new(
        reroute_028.outputs[0], distribute_points_on_faces_001.inputs[0]
    )
    lunarterrain.links.new(reroute_024.outputs[0], math_017.inputs[1])
    lunarterrain.links.new(reroute_028.outputs[0], set_position_1.inputs[0])
    lunarterrain.links.new(reroute_024.outputs[0], reroute_027.inputs[0])
    lunarterrain.links.new(group_005_1.outputs[0], math_024.inputs[0])
    lunarterrain.links.new(reroute_033.outputs[0], distribute_points_on_faces.inputs[0])
    lunarterrain.links.new(transform_geometry_001_1.outputs[0], reroute_028.inputs[0])
    lunarterrain.links.new(reroute_030.outputs[0], reroute_016.inputs[0])
    lunarterrain.links.new(reroute_016.outputs[0], reroute_031.inputs[0])
    lunarterrain.links.new(set_position_1.outputs[0], reroute_033.inputs[0])
    lunarterrain.links.new(reroute_008.outputs[0], reroute_032.inputs[0])
    lunarterrain.links.new(
        group_002_1.outputs[0], distribute_points_on_faces_001.inputs[5]
    )
    lunarterrain.links.new(reroute_002_2.outputs[0], reroute_029.inputs[0])
    lunarterrain.links.new(
        reroute_029.outputs[0], distribute_points_on_faces_001.inputs[6]
    )
    lunarterrain.links.new(reroute_029.outputs[0], math_023.inputs[0])
    lunarterrain.links.new(reroute_030.outputs[0], reroute_026.inputs[0])
    lunarterrain.links.new(reroute_029.outputs[0], reroute_030.inputs[0])
    lunarterrain.links.new(math_006_1.outputs[0], reroute_024.inputs[0])
    lunarterrain.links.new(reroute_024.outputs[0], reroute_035.inputs[0])
    lunarterrain.links.new(separate_xyz_002_1.outputs[0], combine_xyz_005.inputs[0])
    lunarterrain.links.new(separate_xyz_002_1.outputs[1], combine_xyz_005.inputs[1])
    lunarterrain.links.new(combine_xyz_005.outputs[0], vector_math_005.inputs[0])
    lunarterrain.links.new(vector_math_005.outputs[1], compare_004.inputs[0])
    lunarterrain.links.new(compare_004.outputs[0], boolean_math_001.inputs[0])
    lunarterrain.links.new(boolean_math_1.outputs[0], boolean_math_001.inputs[1])
    lunarterrain.links.new(boolean_math_001.outputs[0], delete_geometry_1.inputs[1])
    lunarterrain.links.new(reroute_004_1.outputs[0], reroute_034.inputs[0])
    lunarterrain.links.new(reroute_034.outputs[0], compare_004.inputs[1])
    lunarterrain.links.new(separate_xyz_003_1.outputs[0], mix_002.inputs[2])
    lunarterrain.links.new(separate_xyz_003_1.outputs[1], mix_002.inputs[3])
    lunarterrain.links.new(group_010_1.outputs[0], math_026.inputs[1])
    lunarterrain.links.new(mix_002.outputs[0], math_026.inputs[0])
    lunarterrain.links.new(math_026.outputs[0], math_027.inputs[0])
    lunarterrain.links.new(math_027.outputs[0], math_028.inputs[0])
    lunarterrain.links.new(math_028.outputs[0], distribute_points_on_faces.inputs[2])
    lunarterrain.links.new(group_011_1.outputs[0], distribute_points_on_faces.inputs[5])
    lunarterrain.links.new(reroute_031.outputs[0], reroute_036.inputs[0])
    lunarterrain.links.new(reroute_036.outputs[0], group_011_1.inputs[3])
    lunarterrain.links.new(float_curve_001.outputs[0], math_029.inputs[0])
    lunarterrain.links.new(math_027.outputs[0], math_029.inputs[1])
    lunarterrain.links.new(position_008.outputs[0], attribute_statistic_1.inputs[2])
    lunarterrain.links.new(attribute_statistic_1.outputs[5], separate_xyz_006.inputs[0])
    lunarterrain.links.new(separate_xyz_006.outputs[2], combine_xyz_006.inputs[2])
    lunarterrain.links.new(vector_math_1.outputs[0], transform_geometry_003_1.inputs[1])
    lunarterrain.links.new(sample_index.outputs[0], vector_math_1.inputs[0])
    lunarterrain.links.new(math_029.outputs[0], transform_geometry_004.inputs[3])
    lunarterrain.links.new(
        transform_geometry_004.outputs[0], transform_geometry_003_1.inputs[0]
    )
    lunarterrain.links.new(group_001_1.outputs[0], transform_geometry_004.inputs[0])
    lunarterrain.links.new(
        transform_geometry_004.outputs[0], attribute_statistic_1.inputs[0]
    )
    lunarterrain.links.new(vector_math_006.outputs[0], vector_math_1.inputs[1])
    lunarterrain.links.new(combine_xyz_006.outputs[0], vector_math_006.inputs[0])
    lunarterrain.links.new(reroute_019_1.outputs[0], group_012_1.inputs[2])
    lunarterrain.links.new(group_012_1.outputs[0], vector_math_006.inputs[3])
    lunarterrain.links.new(group_input_4.outputs[2], reroute_040.inputs[0])
    lunarterrain.links.new(reroute_038.outputs[0], math_015.inputs[1])
    lunarterrain.links.new(reroute_040.outputs[0], reroute_038.inputs[0])
    lunarterrain.links.new(math_029.outputs[0], math_015.inputs[0])
    lunarterrain.links.new(math_015.outputs[0], math_030.inputs[0])
    lunarterrain.links.new(math_030.outputs[0], float_to_integer_001.inputs[0])
    lunarterrain.links.new(float_to_integer_001.outputs[0], group_001_1.inputs[1])
    lunarterrain.links.new(join_geometry.outputs[0], switch_2.inputs[1])
    lunarterrain.links.new(reroute_033.outputs[0], reroute_037.inputs[0])
    lunarterrain.links.new(repeat_output.outputs[0], mesh_boolean_1.inputs[1])
    lunarterrain.links.new(mesh_boolean_1.outputs[0], switch_2.inputs[2])
    lunarterrain.links.new(reroute_041.outputs[0], switch_2.inputs[0])
    lunarterrain.links.new(group_input_4.outputs[4], reroute_039.inputs[0])
    lunarterrain.links.new(reroute_039.outputs[0], reroute_041.inputs[0])
    lunarterrain.links.new(value.outputs[0], math_027.inputs[1])
    lunarterrain.links.new(
        transform_geometry_003_1.outputs[0], join_geometry_001.inputs[0]
    )
    lunarterrain.links.new(reroute_037.outputs[0], join_geometry.inputs[0])
    lunarterrain.links.new(reroute_037.outputs[0], mesh_boolean_1.inputs[1])
    return lunarterrain


lunarterrain = lunarterrain_node_group()

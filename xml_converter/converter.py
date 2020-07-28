"""This module creates BNG files from XML files."""

from beamngpy import BeamNGpy, Scenario, Road, StaticObject, Vehicle
from beamngpy.beamngcommon import ENV
from os.path import join
from math import radians, sin, cos
from shapely.geometry import LineString
from numpy import asarray, clip, concatenate, arange, linspace, array, around, dot
from scipy.interpolate import splev
from re import findall

from utils.utility_functions import get_angle, multilinestrings_to_linestring

NUM_NODES = 100


def _calc_rot_matrix(rad_x, rad_y, rad_z):
    """Calculates the rotation matrix which is required for BNG.
    :param rad_x: Radians in x direction.
    :param rad_y: Radians in y direction.
    :param rad_z: Radians in z direction.
    :return: Rotation matrix.
    """
    rot_matrix_x = [[1, 0, 0], [0, cos(rad_x), sin(rad_x)], [0, -sin(rad_x), cos(rad_x)]]
    rot_matrix_y = [[cos(rad_y), 0, -sin(rad_y)], [0, 1, 0], [sin(rad_y), 0, cos(rad_y)]]
    rot_matrix_z = [[cos(rad_z), sin(rad_z), 0], [-sin(rad_z), cos(rad_z), 0], [0, 0, 1]]
    rot_matrix = dot(rot_matrix_z, rot_matrix_y)
    rot_matrix = dot(rot_matrix, rot_matrix_x)
    return rot_matrix


def _get_nodes(divider_line, road_segments, fac, width=0.2):

    nodes = list()
    i = 0
    while i < len(list(divider_line.coords)):
        index = int(round(fac * i))
        if index >= len(road_segments):
            index = len(road_segments) - 1
        z = road_segments[index].attrib.get("z")
        if z is None:
            z = 0.01
        else:
            z = (int(z))
        nodes.append((list(divider_line.coords)[i][0], list(divider_line.coords)[i][1], z, width))
        i += 1
    return nodes


def _get_offset_nodes(road_segments, line, offset, direction):
    outer_line = line.parallel_offset(offset, direction)
    outer_line = multilinestrings_to_linestring(outer_line)
    fac = len(road_segments) / len(list(outer_line.coords))
    nodes = _get_nodes(outer_line, road_segments, fac)
    if direction == "right":
        nodes = nodes[::-1]
    return nodes


def b_spline(old_coords, samples=NUM_NODES):
    """Calculate {@code samples} samples on a bspline. This is the road representation function.
    :param samples: Number of samples to return.
    :param old_coords: List of tuples.
    :return: Array with samples, representing a bspline of the given control points of the lanes.
    """
    old_coords = asarray(old_coords)
    count = len(old_coords)
    degree = clip(2, 0, count - 1)
    kv = concatenate(([0] * degree, arange(count - degree + 1), [count - degree] * degree))
    u = linspace(False, (count - degree), samples)
    splines = around(array(splev(u, (kv, old_coords.T, degree))).T, 3)
    return splines


class Converter:

    def __init__(self, dbc_root, dbe_root, index):
        self.dbc_root = dbc_root
        self.dbe_root = dbe_root
        self.index = index
        self.lines = list()
        self.lights = list()
        self.light_content = list()
        self.traffic_lights = list()
        self.light_index = 0

    def _init_prefab(self):
        self.bng = BeamNGpy('localhost', 64255)
        self.scenario = Scenario('urban', 'urban_{}'.format(self.index))

    def _finalize_prefab(self):
        self.bng.user = None
        self.scenario.make(self.bng)
        self._change_object_options()
        self._add_lights_to_prefab()
        self._add_waypoints()
        self._write_lua_file()

    def add_to_prefab(self):
        self._init_prefab()
        self._add_lanes()
        self._add_obstacles()
        self._add_participants()
        self._finalize_prefab()

    def _add_lanes(self):
        lanes = self.dbe_root.findall("lanes/lane")
        rid = 0
        for lane in lanes:
            self._add_road(lane, rid)
            if bool(lane.attrib.get("markings")):
                self._add_lane_markings(lane, rid)
            rid += 1

    def _add_road(self, lane, rid):
        road = Road(material='road_rubber_sticky', rid='road_{}'.format(rid), interpolate=False, texture_length=2.5,
                    drivability=1)
        nodes = list()
        tuples = list()
        road_segments = lane.findall("laneSegment")
        width = None
        for segment in road_segments:
            d = segment.attrib
            width = float(d.get("width"))
            tuples.append((float(d.get("x")), float(d.get("y"))))
        tuples = b_spline(tuples)
        for coords in tuples:
            nodes.append((coords[0], coords[1], 0.01, width))
        road.nodes.extend(nodes)
        self.scenario.add_road(road)

    def _add_lane_markings(self, lane, rid):
        linestring_nodes = list()
        widths = list()
        road_segments = lane.findall("laneSegment")
        for segment in road_segments:
            d = segment.attrib
            linestring_nodes.append((float(d.get("x")), float(d.get("y"))))
            widths.append(float(d.get("width")))
        linestring_nodes = b_spline(linestring_nodes)
        line = LineString(linestring_nodes)
        coords = line.coords
        if not line.is_simple:
            temp_coords = list(around(list(line.coords), 2))
            coords = list()
            for coord in temp_coords:
                coords.append(tuple(coord))
            coords = list(set(coords))
            if len(coords) > 1:
                line.coords = coords
        if len(coords) > 1:
            outer_offset = int(road_segments[0].attrib.get("width")) / 2 - 0.4
            self._add_outer_marking(road_segments, rid, line, outer_offset, "left")
            self._add_outer_marking(road_segments, rid, line, outer_offset, "right")
            left_lanes = int(lane.attrib.get("leftLanes"))
            right_lanes = int(lane.attrib.get("rightLanes"))
            if left_lanes != 0 and right_lanes != 0:
                self._add_yellow_divider_line(road_segments, rid, line, left_lanes, right_lanes)
            if left_lanes > 1:
                self._add_separator_lines(road_segments, rid, line, left_lanes, right_lanes,
                                          int(road_segments[0].attrib.get("width")), "left")
            if right_lanes > 1:
                self._add_separator_lines(road_segments, rid, line, left_lanes, right_lanes,
                                          int(road_segments[0].attrib.get("width")), "right")

    def _add_outer_marking(self, road_segments, rid, line, offset, direction):
        nodes = _get_offset_nodes(road_segments, line, offset, direction)
        road = Road(material='line_white', rid='road_{}_{}_line'.format(rid, direction), interpolate=False,
                    texture_length=16, drivability=-1)
        road.nodes.extend(nodes)
        self.scenario.add_road(road)

    def _add_yellow_divider_line(self, road_segments, rid, line, left_lanes, right_lanes):
        mid = int(road_segments[0].attrib.get("width")) / 2
        lane_width = int(road_segments[0].attrib.get("width")) / (left_lanes + right_lanes)
        nodes = []
        if left_lanes == right_lanes:
            for node in road_segments:
                d = node.attrib
                z = 0.01 if d.get("z") is None else d.get("z")
                nodes.append((d.get("x"), d.get("y"), z, 0.3))
        else:
            if left_lanes > right_lanes:
                offset = left_lanes * lane_width - mid
                direction = "right"
            elif left_lanes < right_lanes:
                offset = right_lanes * lane_width - mid
                direction = "left"
            else:
                raise TypeError("leftLanes and rightLanes must be Integers.")
            nodes = _get_offset_nodes(road_segments, line, offset, direction)
        road = Road(material='line_yellow_double', rid='road_{}_left_right_divider'.format(rid), interpolate=False,
                    texture_length=16, drivability=-1)
        road.nodes.extend(nodes)
        self.scenario.add_road(road)

    def _add_separator_lines(self, road_segments, rid, line, left_lanes, right_lanes, width, direction):
        separators = left_lanes - 1 if direction == "left" else right_lanes - 1
        width_per_lane = width / (left_lanes + right_lanes)
        iterator = 1
        mid = width / 2
        while iterator <= separators:
            position = iterator * width_per_lane if direction == "left" else width - (iterator * width_per_lane)
            offset = mid - position if direction == "left" else position - mid
            nodes = _get_offset_nodes(road_segments, line, offset, direction)
            road = Road(material='line_dashed_short', rid='road_{}_{}_separator_{}'.format(rid, direction,
                                                                                           iterator - 1),
                        interpolate=False, texture_length=16, drivability=-1)
            road.nodes.extend(nodes)
            self.scenario.add_road(road)
            iterator += 1

    def _change_object_options(self):
        prefab_path = join(ENV["BNG_HOME"], "levels", "urban", "scenarios", "urban_{}.prefab".format(self.index))
        prefab_file = open(prefab_path, "r")
        original_content = prefab_file.readlines()
        prefab_file.close()
        new_content = list()
        for line in original_content:
            if "overObjects" in line:
                line = line.replace("0", "1")
                new_content.append("        annotation = \"STREET\";\n")
            if "breakAngle" in line:
                line = "        breakAngle = \"3\";\n"
            if "renderPriority" in line:
                line = ""
            if "distanceFade" in line:
                line = ""
            if "Material" in line:
                if "road_rubber_sticky" in line:
                    new_content.append("        distanceFade = \"1000 1000\";\n")
                    new_content.append("        renderPriority = \"10\";\n")
                else:
                    new_content.append("        distanceFade = \"0 0\";\n")
                    new_content.append("        renderPriority = \"9\";\n")
            new_content.append(line)
        prefab_file = open(prefab_path, "w")
        prefab_file.writelines(new_content)
        prefab_file.close()

    def _add_obstacles(self):

        def calc_coords_after_rot(coords, ref_coords, my_rot_matrix):
            coords = dot(my_rot_matrix, coords)
            coords = (coords[0] + ref_coords[0], coords[1] + ref_coords[1], coords[2] + ref_coords[2])
            return coords

        def add_sign_to_traffic_light(my_sign):
            sign_name = my_sign + "_traffic_light_" + str(id_number)
            my_shape = '/levels/urban/art/objects/' + my_sign + '_without_pole.dae'
            sign_coords = (0, 0.12, 0.66)
            my_rot = (float(x_rot), float(y_rot), -float(z_rot) + 90)
            my_rot_matrix = _calc_rot_matrix(radians(rot[0]), radians(rot[1]), radians(rot[2]))
            sign_coords = calc_coords_after_rot(sign_coords, pole_coords, my_rot_matrix)
            my_sign = StaticObject(name=sign_name, pos=sign_coords, rot=my_rot, scale=(1.7, 1.7, 1.7), shape=my_shape)
            self.scenario.add_object(my_sign)

        obstacles = self.dbe_root.find("obstacles")
        if obstacles is None:
            obstacles = list()
        id_number = 0
        first_golf = True
        for obstacle in obstacles:
            obstacle_attr = obstacle.attrib
            z = 0 if obstacle_attr.get("z") is None else obstacle_attr.get("z")
            x_rot = 0 if obstacle_attr.get("xRot") is None else obstacle_attr.get("xRot")
            y_rot = 0 if obstacle_attr.get("yRot") is None else obstacle_attr.get("yRot")
            z_rot = 0 if obstacle_attr.get("zRot") is None else obstacle_attr.get("zRot")
            assert obstacle_attr.get("x") is not None, "x Value for a {} object is None.".format(obstacle.tag)
            assert obstacle_attr.get("y") is not None, "y Value for a {} object is None.".format(obstacle.tag)
            pos = (float(obstacle_attr.get("x")), float(obstacle_attr.get("y")), float(z))
            rot = (float(x_rot), float(y_rot), float(z_rot))
            sign = obstacle_attr.get("sign")
            mode = obstacle_attr.get("mode")
            oid = obstacle_attr.get("oid")
            if obstacle.tag == "stopsign":
                rot = (rot[0], rot[1], 90 - rot[2])
                name_sign = "stopsign_" + str(id_number)
                stopsign = StaticObject(pos=pos, rot=rot, name=name_sign,
                                        scale=(1.9, 1.9, 1.9), shape='/levels/urban/art/objects/stopsign.dae')
                self.scenario.add_object(stopsign)
            elif obstacle.tag == "prioritysign":
                rot = (rot[0], rot[1], 90 - rot[2])
                name_sign = "prioritysign_" + str(id_number)
                prioritysign = StaticObject(pos=pos, rot=rot, name=name_sign,
                                            scale=(1.2, 1.2, 1.2), shape='/levels/urban/art/objects/priority.dae')
                self.scenario.add_object(prioritysign)
            elif obstacle.tag == "golf":
                rot = (rot[0], rot[1], 90 - rot[2])
                name_car = "Golf_" + str(id_number)
                golf = StaticObject(pos=pos, rot=rot, name=name_car,
                                    scale=(1.2, 1.2, 1.2), shape='/vehicles/87Golf/87Golf.dae')
                self.scenario.add_object(golf)
                if first_golf:
                    color = findall("\d+\.\d+", obstacle_attr.get("color"))
                    materials_path = join(ENV['BNG_HOME'], "levels", "urban", "art", "objects", "materials.cs")
                    materials = open(materials_path, "r")
                    original_content = materials.readlines()
                    materials.close()
                    for idx, line in enumerate(original_content):
                        if "Player-Generated Color" in line:
                            original_content[idx + 1] = "   diffuseColor[2] = \"{} {} {} {}\";\n" \
                                .format(color[0], color[1], color[2], color[3])
                    first_golf = False
                    materials = open(materials_path, "w")
                    materials.writelines(original_content)
                    materials.close()
            elif obstacle.tag == "trafficlightsingle":
                name_light = "trafficlightsingle_" + str(id_number)
                name_pole = "polesingle_" + str(id_number)
                rot = (rot[0], rot[1], -rot[2] - 90)
                rad_x = radians(rot[0])
                rad_y = radians(rot[1])
                rad_z = radians(rot[2])
                pole_coords = (pos[0], pos[1], pos[2])
                rot_matrix = _calc_rot_matrix(rad_x, rad_y, rad_z)
                traffic_light_coords = (0, 0, 4.62)  # x y z coordinates when pole is placed at (0,0,0)
                traffic_light_coords = dot(rot_matrix, traffic_light_coords)
                traffic_light_coords = (
                    traffic_light_coords[0] + pole_coords[0], traffic_light_coords[1] + pole_coords[1],
                    traffic_light_coords[2] + pole_coords[2])
                if mode is not None and mode == "blinking":
                    self._add_blinking_traffic_lights(traffic_light_coords)
                if mode is not None and mode == "manual":
                    self._add_traffic_lights(traffic_light_coords, oid, 0)
                if mode is not None and mode == "off":
                    shape = '/levels/urban/art/objects/trafficlight1a_off.dae'
                else:
                    shape = '/levels/urban/art/objects/trafficlight1a.dae'
                traffic_light = StaticObject(name=name_light, pos=traffic_light_coords, rot=rot, scale=(1, 1, 1),
                                             shape=shape)
                self.scenario.add_object(traffic_light)
                pole = StaticObject(name=name_pole, pos=pole_coords, rot=rot, scale=(1, 1, 1.1),
                                    shape='/levels/urban/art/objects/pole_traffic1.dae')
                self.scenario.add_object(pole)
                add_sign_to_traffic_light(sign)
            elif obstacle.tag == "trafficlightdouble":
                rot = (rot[0], rot[1], -(rot[2] + 90))
                name_light1 = "trafficlightdouble" + str(id_number)
                name_light2 = "trafficlightdouble" + str(id_number) + 'a'
                name_pole = "poledouble" + str(id_number)
                rad_x = radians(rot[0])
                rad_y = radians(rot[1])
                rad_z = radians(rot[2])
                pole_coords = (pos[0], pos[1], pos[2])
                traffic_light1_coords = (5.7, 0.17, 5.9)  # x y z coordinates when pole is placed at (0,0,0)
                traffic_light2_coords = (2.1, 0.17, 5.5)
                rot_matrix = _calc_rot_matrix(rad_x, rad_y, rad_z)
                traffic_light1_coords = calc_coords_after_rot(traffic_light1_coords, pole_coords, rot_matrix)
                traffic_light2_coords = calc_coords_after_rot(traffic_light2_coords, pole_coords, rot_matrix)
                if mode is not None and (mode == "blinking" or mode == "manual"):
                    rot_matrix = _calc_rot_matrix(rad_x, rad_y, radians(-float(z_rot)))
                    light_pos_1 = (-0.25, 2.09, 5.48)
                    light_pos_2 = (-0.25, 5.72, 5.89)
                    light_pos_1 = calc_coords_after_rot(light_pos_1, pole_coords, rot_matrix)
                    light_pos_2 = calc_coords_after_rot(light_pos_2, pole_coords, rot_matrix)
                    if mode == "blinking":
                        self._add_blinking_traffic_lights(light_pos_1)
                        self._add_blinking_traffic_lights(light_pos_2)
                    else:
                        self._add_traffic_lights(light_pos_1, oid, 0)
                        self._add_traffic_lights(light_pos_2, oid, 1)
                if mode is not None and mode == "off":
                    shape = '/levels/urban/art/objects/trafficlight2a_off.dae'
                else:
                    shape = '/levels/urban/art/objects/trafficlight2a.dae'
                pole = StaticObject(name=name_pole, pos=pole_coords, rot=rot, scale=(0.75, 0.75, 0.75),
                                    shape='/levels/urban/art/objects/pole_light_signal1.dae')
                self.scenario.add_object(pole)
                traffic_light1 = StaticObject(name=name_light1, pos=traffic_light1_coords, rot=rot, scale=(1, 1, 1),
                                              shape=shape)
                self.scenario.add_object(traffic_light1)
                traffic_lights2 = StaticObject(name=name_light2, pos=traffic_light2_coords, rot=rot, scale=(1, 1, 1),
                                               shape=shape)
                self.scenario.add_object(traffic_lights2)
                add_sign_to_traffic_light(sign)
            else:
                raise NotImplementedError("Error. Object type \"{}\" is not supported.".format(obstacle.tag))
            id_number += 1

    def _add_blinking_traffic_lights(self, pos):
        light = "   new PointLight(traffic_blinking_" + str(self.light_index) + "){\n" \
                "       radius = \"0.300000012\";\n" \
                "       isEnabled = \"1\";\n" \
                "       color = \"1 0.662744999 0 2\";\n" \
                "       brightness = \"10\";\n" \
                "       castShadows = \"0\";\n" \
                "       priority = \"1\";\n" \
                "       animate = \"0\";\n" \
                "       animationPeriod = \"1\";\n" \
                "       animationPhase = \"1\";\n" \
                "       flareType = \"BNG_Sunflare_2\";\n" \
                "       flareScale = \"0.400000006\";\n" \
                "       attenuationRatio = \"0 1 1\";\n" \
                "       shadowType = \"DualParaboloidSinglePass\";\n" \
                "       texSize = \"512\";\n" \
                "       overDarkFactor = \"2000 1000 500 100\";\n" \
                "       shadowDistance = \"400\";\n" \
                "       shadowSoftness = \"0.150000006\";\n" \
                "       numSplits = \"1\";\n" \
                "       logWeight = \"0.910000026\";\n" \
                "       fadeStartDistance = \"0\";\n" \
                "       lastSplitTerrainOnly = \"0\";\n" \
                "       representedInLightmap = \"0\";\n" \
                "       shadowDarkenColor = \"0 0 0 -1\";\n" \
                "       includeLightmappedGeometryInShadow = \"0\";\n" \
                "       position = \"" + str(pos[0]) + " " + str(pos[1]) + " " + str(pos[2]) + "\";\n" \
                "       rotationMatrix = \"1 0 0 0 0.999999762 -0.000690533896 0 0.000690533896 0.999999762\";\n" \
                "       mode = \"Ignore\";\n" \
                "       canSave = \"1\";\n" \
                "       canSaveDynamicFields = \"1\";\n" \
                "   };\n"
        self.light_content.append(light)
        self.lights.append({"id": "traffic_blinking_{}".format(self.light_index),
                            "position": (pos[0], pos[1], pos[2])})
        self.light_index += 1

    def _add_traffic_lights(self, pos, oid, index):
        green = "   new PointLight(" + oid + "_green_" + str(index) + "){\n" \
                "       radius = \"0.300000012\";\n" \
                "       isEnabled = \"1\";\n" \
                "       color = \"0 0.852744999 0 2\";\n" \
                "       brightness = \"10\";\n" \
                "       castShadows = \"0\";\n" \
                "       priority = \"1\";\n" \
                "       animate = \"0\";\n" \
                "       animationPeriod = \"1\";\n" \
                "       animationPhase = \"1\";\n" \
                "       flareType = \"BNG_Sunflare_2\";\n" \
                "       flareScale = \"0.400000006\";\n" \
                "       attenuationRatio = \"0 1 1\";\n" \
                "       shadowType = \"DualParaboloidSinglePass\";\n" \
                "       texSize = \"512\";\n" \
                "       overDarkFactor = \"2000 1000 500 100\";\n" \
                "       shadowDistance = \"400\";\n" \
                "       shadowSoftness = \"0.150000006\";\n" \
                "       numSplits = \"1\";\n" \
                "       logWeight = \"0.910000026\";\n" \
                "       fadeStartDistance = \"0\";\n" \
                "       lastSplitTerrainOnly = \"0\";\n" \
                "       representedInLightmap = \"0\";\n" \
                "       shadowDarkenColor = \"0 0 0 -1\";\n" \
                "       includeLightmappedGeometryInShadow = \"0\";\n" \
                "       position = \"" + str(pos[0]) + " " + str(pos[1]) + " " + str(-33) + "\";\n" \
                "       rotationMatrix = \"1 0 0 0 0.999999762 -0.000690533896 0 0.000690533896 0.999999762\";\n" \
                "       mode = \"Ignore\";\n" \
                "       canSave = \"1\";\n" \
                "       canSaveDynamicFields = \"1\";\n" \
                "   };\n"
        yellow = "   new PointLight(" + oid + "_yellow_" + str(index) + "){\n" \
                 "       radius = \"0.300000012\";\n" \
                 "       isEnabled = \"1\";\n" \
                 "       color = \"1 0.662744999 0 2\";\n" \
                 "       brightness = \"10\";\n" \
                 "       castShadows = \"0\";\n" \
                 "       priority = \"1\";\n" \
                 "       animate = \"0\";\n" \
                 "       animationPeriod = \"1\";\n" \
                 "       animationPhase = \"1\";\n" \
                 "       flareType = \"BNG_Sunflare_2\";\n" \
                 "       flareScale = \"0.400000006\";\n" \
                 "       attenuationRatio = \"0 1 1\";\n" \
                 "       shadowType = \"DualParaboloidSinglePass\";\n" \
                 "       texSize = \"512\";\n" \
                 "       overDarkFactor = \"2000 1000 500 100\";\n" \
                 "       shadowDistance = \"400\";\n" \
                 "       shadowSoftness = \"0.150000006\";\n" \
                 "       numSplits = \"1\";\n" \
                 "       logWeight = \"0.910000026\";\n" \
                 "       fadeStartDistance = \"0\";\n" \
                 "       lastSplitTerrainOnly = \"0\";\n" \
                 "       representedInLightmap = \"0\";\n" \
                 "       shadowDarkenColor = \"0 0 0 -1\";\n" \
                 "       includeLightmappedGeometryInShadow = \"0\";\n" \
                 "       position = \"" + str(pos[0]) + " " + str(pos[1]) + " " + str(-33) + "\";\n" \
                 "       rotationMatrix = \"1 0 0 0 0.999999762 -0.000690533896 0 0.000690533896 0.999999762\";\n" \
                 "       mode = \"Ignore\";\n" \
                 "       canSave = \"1\";\n" \
                 "       canSaveDynamicFields = \"1\";\n" \
                 "   };\n"
        red = "   new PointLight(" + oid + "_red_" + str(index) + "){\n" \
              "       radius = \"0.300000012\";\n" \
              "       isEnabled = \"1\";\n" \
              "       color = \"0.88 0 0 2\";\n" \
              "       brightness = \"10\";\n" \
              "       castShadows = \"0\";\n" \
              "       priority = \"1\";\n" \
              "       animate = \"0\";\n" \
              "       animationPeriod = \"1\";\n" \
              "       animationPhase = \"1\";\n" \
              "       flareType = \"BNG_Sunflare_2\";\n" \
              "       flareScale = \"0.400000006\";\n" \
              "       attenuationRatio = \"0 1 1\";\n" \
              "       shadowType = \"DualParaboloidSinglePass\";\n" \
              "       texSize = \"512\";\n" \
              "       overDarkFactor = \"2000 1000 500 100\";\n" \
              "       shadowDistance = \"400\";\n" \
              "       shadowSoftness = \"0.150000006\";\n" \
              "       numSplits = \"1\";\n" \
              "       logWeight = \"0.910000026\";\n" \
              "       fadeStartDistance = \"0\";\n" \
              "       lastSplitTerrainOnly = \"0\";\n" \
              "       representedInLightmap = \"0\";\n" \
              "       shadowDarkenColor = \"0 0 0 -1\";\n" \
              "       includeLightmappedGeometryInShadow = \"0\";\n" \
              "       position = \"" + str(pos[0]) + " " + str(pos[1]) + " " + str(-33) + "\";\n" \
              "       rotationMatrix = \"1 0 0 0 0.999999762 -0.000690533896 0 0.000690533896 0.999999762\";\n" \
              "       mode = \"Ignore\";\n" \
              "       canSave = \"1\";\n" \
              "       canSaveDynamicFields = \"1\";\n" \
              "   };\n"
        self.light_content.append(green)
        self.light_content.append(yellow)
        self.light_content.append(red)
        self.traffic_lights.append([{"id": oid + "_green_" + str(index), "position": (pos[0], pos[1], pos[2] - 0.5)},
                            {"id": oid + "_yellow_" + str(index), "position": (pos[0], pos[1], pos[2])},
                            {"id": oid + "_red_" + str(index), "position": (pos[0], pos[1], pos[2] + 0.5)}])

    def _add_lights_to_prefab(self):
        prefab_path = join(ENV["BNG_HOME"], "levels", "urban", "scenarios", "urban_{}.prefab".format(self.index))
        prefab_file = open(prefab_path, "r")
        original_content = prefab_file.readlines()
        prefab_file.close()
        original_content[-1] = ""
        for light in self.light_content:
            original_content.append(light)
        prefab_file = open(prefab_path, "w")
        prefab_file.writelines(original_content)
        prefab_file.close()

    def _add_participants(self):
        participants = self.dbc_root.findall("participants/participant")
        for participant in participants:
            attr = participant.attrib
            assert attr.get("id") is not None, "Vehicles needs a ID."
            model = "ETK800" if attr.get("model") is None else attr.get("model")
            color = "White" if attr.get("color") is None else attr.get("color")
            vehicle = Vehicle(attr.get("id"), color=color, model=model)
            init_state = participant.find("initialState")
            x = init_state.get("x")
            y = init_state.get("y")
            assert x is not None and y is not None, "x and y coordinates must be set."
            z = 0 if init_state.get("z") is None else init_state.get("z")
            x_rot = 0 if init_state.get("x_rot") is None else init_state.get("x_rot")
            y_rot = 0 if init_state.get("y_rot") is None else init_state.get("y_rot")
            z_rot = -90 if init_state.get("orientation") is None else -float(init_state.get("orientation")) - 90
            pos = (x, y, z)
            rot = (x_rot, y_rot, z_rot)
            self.scenario.add_vehicle(vehicle=vehicle, pos=pos, rot=rot)

    def _add_waypoints(self):
        prefab_path = join(ENV["BNG_HOME"], "levels", "urban", "scenarios", "urban_{}.prefab".format(self.index))
        prefab_file = open(prefab_path, "r")
        original_content = prefab_file.readlines()
        prefab_file.close()
        participants = self.dbc_root.findall("participants/participant")
        for participant in participants:
            waypoints = participant.findall("movement/waypoint")
            if len(waypoints) == 0:
                continue
            lines = list()
            line = list()
            vid = participant.get("id")
            index = 0
            i = 0
            current_index = int(waypoints[0].attrib.get("lane"))
            while i < len(waypoints):
                attr = waypoints[i].attrib
                z = 0 if attr.get("z") is None else attr.get("z")
                original_content.extend([
                    "    new BeamNGWaypoint(wp_{}_{}){{\n".format(vid, index),
                    "        drawDebug = \"0\";\n",
                    "        directionalWaypoint = \"0\";\n",
                    "        position = \"" + attr.get("x") + " " + attr.get("y") + " " + str(z) + "\";\n",
                    "        scale = \"" + attr.get("tolerance") + " " + attr.get("tolerance") + " "
                    + attr.get("tolerance") + "\";\n",
                    "        rotationMatrix = \"1 0 0 0 1 0 0 0 1\";\n",
                    "        mode = \"Ignore\";\n",
                    "        canSave = \"1\";\n",
                    "        canSaveDynamicFields = \"1\";\n",
                    "    };\n"
                ])
                if 1 < i < len(waypoints) - 1:
                    attr_prev = waypoints[i - 1].attrib
                    attr_next = waypoints[i + 1].attrib
                    p1 = (float(attr_prev.get("x")), float(attr_prev.get("y")))
                    p2 = (float(attr.get("x")), float(attr.get("y")))
                    p3 = (float(attr_next.get("x")), float(attr_next.get("y")))
                    angle = get_angle(p1, p2, p3)
                    if int(attr.get("lane")) != current_index:
                        current_index = int(attr.get("lane"))
                        line[-1]["speed"] = 0
                        line[-2]["speed"] = 0
                        line[-3]["speed"] = 0
                        lines.append(line)
                        line = list()
                    else:
                        if 170 <= angle <= 190:
                            speed = 13.33
                        elif 150 <= angle < 170 or 190 > angle >= 210:
                            speed = 10
                        elif 130 <= angle < 150 or 210 > angle >= 230:
                            speed = 8.5
                        elif 110 <= angle < 130 or 230 > angle >= 250:
                            speed = 6
                        elif 90 <= angle < 110 or 250 > angle >= 270:
                            speed = 3
                        elif angle < 90 or angle > 270:
                            speed = 2
                        else:
                            speed = 1
                        if len(line) > 0:
                            line[-1]["speed"] = 0 if line[-1].get("speed") == 0 else \
                                (speed + float(line[-1].get("speed"))) / 2
                        line.append({"pos": (float(attr.get("x")), float(attr.get("y")), float(z)), 'speed': speed})
                else:
                    speed = 0 if i == len(waypoints) - 1 else 13.33
                    line.append({"pos": (float(attr.get("x")), float(attr.get("y")), float(z)), 'speed': speed})
                    if i == len(waypoints) - 1:
                        line[-1]["speed"] = 0
                        line[-2]["speed"] = 0
                        line[-3]["speed"] = 0
                        lines.append(line)
                index += 1
                i += 1
            if vid == "ego":
                self.success_point = "wp_{}_{}".format(vid, index - 1)
            self.lines.append({"vid": vid, "lines": lines})
        original_content.append("};")
        prefab_file = open(prefab_path, "w")
        prefab_file.writelines(original_content)
        prefab_file.close()

    def _get_on_race_start_line_content(self):
        ego_lines = None
        for entry in self.lines:
            if entry.get("vid") == "ego":
                ego_lines = entry.get("lines")
                break
        assert ego_lines is not None, "Missing line of vehicle \"ego\"."
        content = 'local vehicleName = \'ego\'\n'\
                  '  local arg = {line = {\n                 '
        i = 0
        while i < len(ego_lines[0]):
            pos = ego_lines[0][i].get("pos")
            speed = ego_lines[0][i].get("speed")
            content += "{pos = {" + str(pos[0]) + ", " + str(pos[1]) + ", " + str(pos[2]) + "}, speed = " \
                       + str(speed) + "}"
            if i + 1 == len(ego_lines[0]):
                content += "\n                }"
            else:
                content += ", \n                 "
            i += 1
        content += '}\n'\
                   '  sh.setAiLine(vehicleName, arg)\n'
        return content

    def _get_ego_lines_content(self, idx):
        ego_lines = None
        for entry in self.lines:
            if entry.get("vid") == "ego":
                ego_lines = entry.get("lines")
                break
        assert ego_lines is not None, "Missing line of vehicle \"ego\"."
        content = '    if ego_time_' + str(idx) + ' == 7 then\n' \
                  '      print(ego_time_' + str(idx) + ')\n' \
                  '      local egoName = \"ego\"\n'\
                  '      local arg_ego = {line = {\n                 '
        i = 0
        #print(ego_lines)
        #print(len(ego_lines))
        #print(idx)
        while i < len(ego_lines[idx+1]):
            pos = ego_lines[idx+1][i].get("pos")
            speed = ego_lines[idx+1][i].get("speed")
            content += "{pos = {" + str(pos[0]) + ", " + str(pos[1]) + ", " + str(pos[2]) + "}, speed = " \
                       + str(speed) + "}"
            if i + 1 == len(ego_lines[idx+1]):
                content += "\n                  }"
            else:
                content += ", \n                   "
            i += 1
        content += '}\n' \
                   '      sh.setAiLine(egoName, arg_ego)\n' \
                   '    end\n'
        return content

    def _write_lua_file(self):
        """
        :return:
        """
        triggers = self.dbc_root.find("triggerPoints")
        trigger_content = ""
        trigger_list = list()
        trigger_points = list()
        spawn_points = list()
        teleport_triggers = list()
        traffic_triggers = list()
        for trigger in triggers:
            if trigger.attrib.get("action") == "spawnAndStart":
                teleport_triggers.append(trigger)
            elif trigger.attrib.get("action") == "switchLights":
                traffic_triggers.append(trigger.attrib)
        for idx, trigger in enumerate(teleport_triggers):
            spawn_point = trigger.find("spawnPoint").attrib
            spawn_points.append(spawn_point)
            trigger_point = trigger.attrib
            trigger_points.append(trigger_point)
            z = 0 if trigger_point.get("z") is None else trigger_point.get("z")
            trigger_content += "local trigger_" + str(idx) + " = Point3F(" + str(trigger_point.get("x")) + ", " \
                               + str(trigger_point.get("y")) + ", " + str(z) + ")\n" \
                               "local triggered_" + str(idx) + " = 0\n" \
                               "local ego_time_" + str(idx) + " = 0\n"
            trigger_list.append("trigger_" + str(idx))

        for idx, trigger in enumerate(traffic_triggers):
            z = 0 if trigger.get("z") is None else trigger.get("z")
            trigger_content += "local trigger_traffic_" + str(idx) + " = Point3F(" + str(trigger.get("x")) + ", " \
                               + str(trigger.get("y")) + ", " + str(z) + ")\n" \
                               "local triggered_traffic_" + str(idx) + " = 0\n" \
                               "local traffic_time_" + str(idx) + " = 0\n" \
                               "local yellow_triggered_" + str(idx) + " = 0\n"

        content = "local M = {}\n" \
                  "local points_1 = {}\n" \
                  "local points_2 = {}\n" \
                  "local time = 0\n" \
                  "local sh = require(\'ge/extensions/scenario/scenariohelper\')\n" \
                  "local ego = nil\n" \

        for idx, trigger in enumerate(self.traffic_lights):
            for idx_1, light in enumerate(trigger):
                pos = light.get("position")
                content += "local " + light.get("id") + " = nil\n" \
                           "points_1[" + str(3 * idx + idx_1 + 1) + "] = Point3F(" + str(pos[0]) + ", " \
                           + str(pos[1]) + ", -33)\n"\
                           "points_2[" + str(3 * idx + idx_1 + 1) + "] = Point3F(" + str(pos[0]) + ", " \
                           + str(pos[1]) + ", " + str(pos[2]) + ")\n"

        content += "\nlocal function onRaceStart()\n" \
                   "  ego = scenetree.findObject(\"ego\")\n" \
                   "  " + self._get_on_race_start_line_content() + ""

        for idx, trigger in enumerate(self.traffic_lights):
            for idx_1, light in enumerate(trigger):
                content += "  " + light.get("id") + " = scenetree.findObject(\"" + light.get("id") + "\")\n"

        for idx, trigger in enumerate(self.traffic_lights):
            oid = trigger[0].get("id")
            oid = oid[:22]
            i = int(oid[-1])
            init_state = traffic_triggers[i].get("initState")
            i = 0 if init_state == "green" else 2
            content += "  " + trigger[i].get("id") + ":setPosition(points_2[" + str(3 * idx + i + 1) + "])\n"

        content += "end\n\n"
        content += trigger_content
        content += "local function onRaceTick(raceTickTime)\n" \
                   "  time = time + raceTickTime\n" \
                   "  local pos = ego:getPosition()\n"
        for idx, trigger_point in enumerate(trigger_list):
            spawn_point = spawn_points[idx]
            z = 0 if spawn_point.get("z") is None else spawn_point.get("z")
            z_rot_spawn = -90 if spawn_point.get("orientation") is None \
                else -float(spawn_point.get("orientation")) - 90
            vehicle_name = trigger_points[idx].get("triggers")

            lines = None
            for entry in self.lines:
                if entry.get("vid") == vehicle_name:
                    lines = entry.get("lines")
                    break
            assert lines is not None, "Missing line of vehicle \"" + vehicle_name + "\"."
            #print(len(lines))
            #print(idx)
            #print(lines[idx])
            lines = lines[idx]
            line_content = '    local arg = {line = {\n                 '
            i = 0
            while i < len(lines):
                pos = lines[i].get("pos")
                speed = lines[i].get("speed")
                line_content += "{pos = {" + str(pos[0]) + ", " + str(pos[1]) + ", " + str(pos[2]) + "}, speed = " \
                                + str(speed) + "}"
                if i + 1 == len(lines):
                    line_content += "\n                }"
                else:
                    line_content += ", \n                 "
                i += 1
            line_content += '}\n' \
                            '    sh.setAiLine(vehicleName, arg)\n'

            content += "  if triggered_" + str(idx) + " == 1 then\n" \
                       "    ego_time_" + str(idx) + " = ego_time_" + str(idx) + " + raceTickTime\n" \
                       "  end\n" \
                       "  if math.sqrt((" + trigger_point + ".x - pos.x) ^ 2 + (" \
                       + trigger_point + ".y - pos.y) ^ 2) <= " + str(float(trigger_points[idx].get("tolerance"))*4) \
                       + " and triggered_" + str(idx) + " == 0 then\n" \
                       "    triggered_" + str(idx) + " = 1\n" \
                       "    local vehicleName = \"" + vehicle_name + "\"\n" \
                       "" \
                       "    TorqueScript.eval(vehicleName..\'.position = \"" + spawn_point.get("x") + " " \
                       + spawn_point.get("y") + " " + str(z) + "\";\')\n" \
                       "    TorqueScript.eval(vehicleName..\'.rotation = \"0 0 01 " + str(z_rot_spawn) + "\";\')\n"
            content += line_content
            content += "  end\n\n"
            content += self._get_ego_lines_content(idx)
        for idx, light in enumerate(self.lights):
            pos = light.get("position")
            content += "  local light_" + str(idx) + " = scenetree.findObject(\"" + light.get("id") + "\")\n" \
                       "  local p1_" + str(idx) + " = Point3F(" + str(pos[0]) + ", " + str(pos[1]) + ", -33)\n" \
                       "  local p2_" + str(idx) + " = Point3F(" + str(pos[0]) + ", " + str(pos[1]) \
                       + ", " + str(pos[2]) + ")\n"
        content += "  if time == math.floor(time) then\n" \
                   "    if time % 2 == 0 then\n"
        for idx, light in enumerate(self.lights):
            content += "      light_" + str(idx) + ":setPosition(p1_" + str(idx) + ")\n"
        content += "    else\n"
        for idx, light in enumerate(self.lights):
            content += "      light_" + str(idx) + ":setPosition(p2_" + str(idx) + ")\n"
        content += "    end\n" \
                   "  end\n"

        for idx, trigger in enumerate(traffic_triggers):
            init_state = trigger.get("initState")
            content += "  if math.sqrt((trigger_traffic_" + str(idx) + ".x - pos.x) ^ 2 + (" \
                       + "trigger_traffic_" + str(idx) + ".y - pos.y) ^ 2) <= " \
                       + str(float(trigger.get("tolerance"))*5) \
                       + " and triggered_traffic_" + str(idx) + " == 0 then\n" \
                         "    triggered_traffic_" + str(idx) + " = 1\n"

            for idx_1, traffic_light in enumerate(self.traffic_lights):
                oid = traffic_light[0].get("id")
                oid = oid[:22]
                temp_idx = int(oid[-1])
                old = 0 if init_state == "green" else 2
                new = 0 if old == 2 else 2
                if idx == temp_idx:
                    content += "    " + traffic_light[1].get("id") + ":setPosition(points_1[" \
                               + str(3 * idx_1 + 2) + "])\n"
                    content += "    " + traffic_light[new].get("id") + ":setPosition(points_2[" \
                               + str(3 * idx_1 + new + 1) + "])\n"

            content += "  end\n" \
                       "  if triggered_traffic_" + str(idx) + " == 1 then\n" \
                       "    traffic_time_" + str(idx) + " = traffic_time_" + str(idx) \
                       + " + raceTickTime\n" \
                       "  end\n" \
                       "  if traffic_time_" + str(idx) + " == 7 then\n"

            for idx_1, traffic_light in enumerate(self.traffic_lights):
                oid = traffic_light[0].get("id")
                oid = oid[:22]
                temp_idx = int(oid[-1])
                old = 0 if init_state == "green" else 2
                new = 0 if old == 2 else 2
                if idx == temp_idx:
                    content += "    " + traffic_light[new].get("id") + ":setPosition(points_1[" \
                               + str(3 * idx_1 + new + 1) + "])\n"
                    content += "    " + traffic_light[old].get("id") + ":setPosition(points_2[" \
                               + str(3 * idx_1 + old + 1) + "])\n"
            content += "  end\n"

            if init_state != "red":
                content += "  if traffic_time_" + str(idx) + " == 7 then\n"
                for idx_1, traffic_light in enumerate(self.traffic_lights):
                    oid = traffic_light[0].get("id")
                    oid = oid[:22]
                    temp_idx = int(oid[-1])
                    if idx == temp_idx:
                        content += "    " + traffic_light[1].get("id") + ":setPosition(points_2[" \
                                   + str(3 * idx_1 + 2) + "])\n"
                content += "  end\n"
                content += "  if traffic_time_" + str(idx) + " == 8 then\n"
                for idx_1, traffic_light in enumerate(self.traffic_lights):
                    oid = traffic_light[0].get("id")
                    oid = oid[:22]
                    temp_idx = int(oid[-1])
                    if idx == temp_idx:
                        content += "    " + traffic_light[1].get("id") + ":setPosition(points_1[" \
                                   + str(3 * idx_1 + 2) + "])\n"
                content += "  end\n"

            content += "  if math.sqrt((trigger_traffic_" + str(idx) + ".x - pos.x) ^ 2 + (" \
                       + "trigger_traffic_" + str(idx) + ".y - pos.y) ^ 2) <= " \
                       + str(float(trigger.get("tolerance")) * 15) \
                       + " and yellow_triggered_" + str(idx) + " == 0 then\n" \
                       "    yellow_triggered_" + str(idx) + " = 1\n"
            for idx_1, traffic_light in enumerate(self.traffic_lights):
                oid = traffic_light[0].get("id")
                oid = oid[:22]
                temp_idx = int(oid[-1])
                if idx == temp_idx and init_state == "green":
                    content += "    " + traffic_light[0].get("id") + ":setPosition(points_1[" \
                               + str(3 * idx_1  + 1) + "])\n"
                    content += "    " + traffic_light[1].get("id") + ":setPosition(points_2[" \
                               + str(3 * idx_1 + 2) + "])\n"
            content += "  end\n"

        content += "end\n\n" \
                   "M.onRaceTick = onRaceTick\n" \
                   "M.onRaceStart = onRaceStart\n" \
                   "return M"
        with open("urban_{}.lua".format(self.index), "w") as lua_file:
            print(content, file=lua_file)

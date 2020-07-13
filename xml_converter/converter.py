from beamngpy import BeamNGpy, Scenario, Road, StaticObject, Vehicle
from beamngpy.beamngcommon import ENV
from os.path import join
from math import radians, sin, cos
from numpy import dot
from shapely.geometry import LineString, MultiLineString
from numpy import asarray, clip, concatenate, arange, linspace, array, around
from scipy.interpolate import splev

from utils.utility_functions import get_angle

NUM_NODES = 100


def _calc_rot_matrix(rad_x, rad_y, rad_z):
    rot_matrix_x = [[1, 0, 0], [0, cos(rad_x), sin(rad_x)], [0, -sin(rad_x), cos(rad_x)]]
    rot_matrix_y = [[cos(rad_y), 0, -sin(rad_y)], [0, 1, 0], [sin(rad_y), 0, cos(rad_y)]]
    rot_matrix_z = [[cos(rad_z), sin(rad_z), 0], [-sin(rad_z), cos(rad_z), 0], [0, 0, 1]]
    rot_matrix = dot(rot_matrix_z, rot_matrix_y)
    rot_matrix = dot(rot_matrix, rot_matrix_x)
    return rot_matrix


def _get_nodes(divider_line, road_segments, fac, width=0.2):
    nodes = list()
    it = 0
    while it < len(list(divider_line.coords)):
        index = int(round(fac * it))
        if index >= len(road_segments):
            index = len(road_segments) - 1
        z = road_segments[index].attrib.get("z")
        if z is None:
            z = 0.01
        else:
            z = (int(z))
        nodes.append((list(divider_line.coords)[it][0], list(divider_line.coords)[it][1], z, width))
        it += 1
    return nodes


def _get_offset_nodes(road_segments, line, offset, direction):
    outer_line = line.parallel_offset(offset, direction)
    if isinstance(outer_line, MultiLineString):
        temp_list = list()
        for line in outer_line:
            for coord in list(line.coords):
                temp_list.append(coord)
        outer_line = LineString(temp_list)
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
    return array(splev(u, (kv, old_coords.T, degree))).T


class Converter:

    def __init__(self, dbc_root, dbe_root, index):
        self.dbc_root = dbc_root
        self.dbe_root = dbe_root
        self.index = index
        self.lines = list()

    def _init_prefab(self):
        self.bng = BeamNGpy('localhost', 64255)
        self.scenario = Scenario('urban', 'urban_{}'.format(self.index))

    def _finalize_prefab(self):
        self.bng.user = None
        self.scenario.make(self.bng)
        self._change_object_options()
        self._add_trigger_points()
        self._add_waypoints()

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
        obstacles = self.dbe_root.find("obstacles")
        if obstacles is None:
            obstacles = list()
        id_number = 0
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
                traffic_light = StaticObject(name=name_light, pos=traffic_light_coords, rot=rot,
                                             scale=(1, 1, 1),
                                             shape='/levels/urban/art/objects/trafficlight1a.dae')
                self.scenario.add_object(traffic_light)
                pole = StaticObject(name=name_pole, pos=pole_coords, rot=rot, scale=(1, 1, 1.1),
                                    shape='/levels/urban/art/objects/pole_traffic1.dae')
                self.scenario.add_object(pole)
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
                traffic_light1_coords = dot(rot_matrix, traffic_light1_coords)
                traffic_light1_coords = (
                    traffic_light1_coords[0] + pole_coords[0], traffic_light1_coords[1] + pole_coords[1],
                    traffic_light1_coords[2] + pole_coords[2])
                traffic_light2_coords = dot(rot_matrix, traffic_light2_coords)
                traffic_light2_coords = (
                    traffic_light2_coords[0] + pole_coords[0], traffic_light2_coords[1] + pole_coords[1],
                    traffic_light2_coords[2] + pole_coords[2])

                pole = StaticObject(name=name_pole, pos=pole_coords, rot=rot, scale=(0.75, 0.75, 0.75),
                                    shape='/levels/urban/art/objects/pole_light_signal1.dae')
                self.scenario.add_object(pole)
                traffic_light1 = StaticObject(name=name_light1, pos=traffic_light1_coords, rot=rot, scale=(1, 1, 1),
                                              shape='/levels/urban/art/objects/trafficlight2a.dae')
                self.scenario.add_object(traffic_light1)
                traffic_lights2 = StaticObject(name=name_light2, pos=traffic_light2_coords, rot=rot, scale=(1, 1, 1),
                                               shape='/levels/urban/art/objects/trafficlight2a.dae')
                self.scenario.add_object(traffic_lights2)
            else:
                raise NotImplementedError("Error. Object type \"{}\" is not supported.".format(obstacle.tag))
            id_number += 1

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
            lines = list()
            line = list()
            waypoints = participant.findall("movement/waypoint")
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
            self.lines.append(lines)
        original_content.append("};")
        prefab_file = open(prefab_path, "w")
        prefab_file.writelines(original_content)
        prefab_file.close()

    def _add_trigger_points(self):
        prefab_path = join(ENV["BNG_HOME"], "levels", "urban", "scenarios", "urban_{}.prefab".format(self.index))
        prefab_file = open(prefab_path, "r")
        original_content = prefab_file.readlines()
        prefab_file.close()
        original_content[-1] = ""
        participants = self.dbc_root.findall("participants/participant")
        for participant in participants:
            trigger_points = participant.findall("triggerPoints/triggerPoint")
            vid = participant.get("id")
            index = 0
            for trigger_point in trigger_points:
                attr = trigger_point.attrib
                spawn_point = trigger_point.find("spawnPoint")
                z_spawn = 0 if attr.get("z") is None else spawn_point.get("z")
                z_rot_spawn = -90 if spawn_point.get("orientation") is None \
                    else -float(spawn_point.get("orientation")) - 90
                z = 0 if attr.get("z") is None else attr.get("z")
                original_content.extend([
                    "    new BeamNGTrigger(trigger_{}_{}){{\n".format(vid, index),
                    "        TriggerType = \"Sphere\";\n",
                    "        TriggerMode = \"Contains\";\n",
                    "        TriggerTestType = \"Race corners\";\n",
                    "        luaFunction = \""
                    + "local function teleportPlayer(data)\\n    local egoName = data.subjectName\\n    "
                      "local vehicleName = \\\"{}\\\"\\n\\n    if "
                      "data.event == \\'enter\\' and egoName == \\'{}\\' then\\n        "
                      "TorqueScript.eval(vehicleName..\\\'.position = "
                      "\\\"{} {} {}\\\";\\\')\\n        TorqueScript.eval(vehicleName..\\\'.rotation = \\\"0 0 "
                      "01 {}\\\";\\\')\\n    end\\nend\\n\\nreturn teleportPlayer ".format(vid, attr.get("vid"),
                                                                                           spawn_point.get("x"),
                                                                                           spawn_point.get("y"),
                                                                                           z_spawn, z_rot_spawn)
                    + "\";\n",
                    "        tickPeriod = \"2000\";\n",
                    "        debug = \"0\";\n",
                    "        ticking = \"0\";\n",
                    "        triggerColor = \"9 255 0 45\";\n",
                    "        defaultOnLeave = \"0\";\n",
                    "        position = \"" + attr.get("x") + " " + attr.get("y") + " " + str(z) + "\";\n",
                    "        scale = \"" + attr.get("tolerance") + " " + attr.get("tolerance") + " "
                    + attr.get("tolerance") + "\";\n",
                    "        rotationMatrix = \"1 0 0 0 1 0 0 0 1\";\n",
                    "        canSave = \"1\";\n",
                    "        canSaveDynamicFields = \"1\";\n",
                    "    };\n"
                ])
                index += 1
        prefab_file = open(prefab_path, "w")
        prefab_file.writelines(original_content)
        prefab_file.close()

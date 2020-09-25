from copy import deepcopy
from random import randint, random, choice, uniform, sample

from numpy import asarray, clip, concatenate, arange, linspace, array, around
from scipy.interpolate import splev
from shapely import affinity
from shapely.geometry import LineString, shape, Point, MultiPoint
from termcolor import colored
from os import path
from glob import glob
from pathlib import Path
from scipy.spatial.distance import euclidean

from utils.utility_functions import convert_points_to_lines, get_angle, calc_width, \
    calc_min_max_angles, get_lanes_of_intersection, get_intersection_lines, get_width_lines, \
    get_resize_factor_intersection, multilinestrings_to_linestring, calc_speed_waypoints
from utils.validity_checks import intersection_check_width, intersection_check_last
from utils.xml_creator import build_all_xml
from xml_converter.prefab_creator import b_spline

MIN_DEGREES = 90
MAX_DEGREES = 270
OID_INDEX = 0
INTERSECTION_ID = 0
COLORS = ["White", "Red", "Green", "Yellow", "Black", "Blue", "Orange", "Gray", "Purple"]
PARTICIPANTS_SAMPLES = 45


def _add_parked_cars(individual):
    car_positions = list()
    for idx, lane in enumerate(individual.get("lanes")):
        control_points = lane.get("control_points")
        if lane.get("type") == "intersection" or control_points[0] == control_points[-1]:
            continue
        width = lane.get("width")
        rotations = [0, 45, 90]
        rotation = choice(rotations)
        noise = [x / 10 for x in range(-10, 10)]
        if rotation == 45:
            offset = 3.5
            max_distance = 4
        elif rotation == 90:
            offset = 3
            max_distance = 4.5
        else:
            offset = 2
            max_distance = 5.5
        right = True if random() >= 0.3 else False
        left = True if random() >= 0.3 else False
        line = LineString(control_points)
        line = multilinestrings_to_linestring(line)
        prev_lane = LineString(individual.get("lanes")[idx - 1].get("control_points")) if idx != 0 else None
        prev_width = int(individual.get("lanes")[idx - 1].get("width")) / 2 + offset if idx != 0 else 0
        if left:
            left_lines = [line.parallel_offset(width / 2 + offset + x, "left") for x in noise]
            left_lines = [multilinestrings_to_linestring(x) for x in left_lines]
            iterator = 1
            while iterator < len(left_lines[0].coords):
                left_line = choice(left_lines)
                coords = b_spline(left_line.coords)
                point = coords[iterator]
                if abs(euclidean(point, coords[-1])) < 12:
                    break
                if len(car_positions) == 0 or (abs(euclidean(point, car_positions[-1][0])) > max_distance and
                                               (prev_lane is None or Point(point).distance(prev_lane) > prev_width)):
                    angle = get_angle((coords[iterator - 1][0] + 5, coords[iterator - 1][1]),
                                      coords[iterator - 1], point) - rotation + randint(-8, 8)
                    car_positions.append((point, angle, idx))
                iterator += 1
        if right:
            right_lines = [line.parallel_offset(width / 2 + offset + x, "right") for x in noise]
            right_lines = [multilinestrings_to_linestring(x) for x in right_lines]
            iterator = 1
            while iterator < len(right_lines[0].coords):
                right_line = choice(right_lines)
                coords = right_line.coords[::-1]
                coords = b_spline(coords)
                point = coords[iterator]
                if abs(euclidean(point, coords[-1])) < 12:
                    break
                if len(car_positions) == 0 or (abs(euclidean(point, car_positions[-1][0])) > max_distance and
                                               (prev_lane is None or Point(point).distance(prev_lane) > prev_width)):
                    angle = get_angle((coords[iterator - 1][0] + 5, coords[iterator - 1][1]),
                                      coords[iterator - 1], point) + 180 - rotation + randint(-8, 8)
                    car_positions.append((point, angle, idx))
                iterator += 1
        lane["parked_left"] = left
        lane["parked_right"] = right
        lane["parked_rotation"] = rotation
        lane["parked_offset"] = offset
        lane["parked_max_distance"] = max_distance
    parked_cars = list()
    color = (round(uniform(0, 1), 2), round(uniform(0, 1), 2), round(uniform(0, 1), 2), round(uniform(1, 1.3), 2))
    individual["parked_color"] = color
    for position in car_positions:
        if random() <= 0.4:
            continue
        parked_cars.append({"name": "parkedCar", "position": (position[0][0], position[0][1]), "zRot": position[1],
                            "color": color, "lane": position[2]})
    individual["obstacles"].extend(parked_cars)


def _add_ego_car(individual, add_ego_car=True):
    """Adds the ego car to the criteria xml file. Movement mode can be assigned manually. Each control point is one
    waypoint.
    :param individual: Individual of the population.
    :return: Void.
    """
    samples = 100
    lanes = individual.get("lanes")
    waypoints = list()
    if add_ego_car:
        ego_lanes = individual.get("ego_lanes")
        directions = individual.get("directions")
        lines = list()
        ego_index = 0
        for idx, lane in enumerate(ego_lanes):
            temp_points = lanes[lane].get("control_points")
            temp_points = LineString(temp_points)
            if temp_points.coords[0] == temp_points.coords[-1]:
                continue
            left_lanes = lanes[lane].get("left_lanes")
            right_lanes = lanes[lane].get("right_lanes")
            width = lanes[lane].get("width")
            width_per_lane = width / (left_lanes + right_lanes)
            left = False
            if idx + 1 < len(ego_lanes) and ego_lanes[idx + 1] - ego_lanes[idx] != 1:
                if directions[ego_index] == "left" and right_lanes > 1:
                    left = True
                ego_index += 1
            if left:
                offset = right_lanes - left_lanes - 1
                offset = offset / 2 * width_per_lane
                temp_points = temp_points.parallel_offset(offset, "left")
                temp_points = multilinestrings_to_linestring(temp_points)
                if offset < 0:
                    temp_points.coords = temp_points.coords[::-1]
            else:
                offset = (left_lanes + right_lanes - 1) * width_per_lane / 2
                temp_points = temp_points.parallel_offset(offset, "right")
                temp_points = multilinestrings_to_linestring(temp_points)
                temp_points.coords = temp_points.coords[::-1]
            temp_points.coords = b_spline(temp_points, samples).tolist()
            lines.append(temp_points)

        ego_index = 0
        same_lane = 0
        action_index = 0
        actions = individual.get("actions")
        for idx, lane in enumerate(lines):
            control_points = list(lines[idx].coords)
            opposite_dir = False
            deleted_points = list()
            lane_change = False
            stop_flag = False
            if idx != 0 and ego_lanes[idx] - ego_lanes[idx - 1] != 1:
                if actions[action_index] == "stop":
                    same_lane += 1
                    stop_flag = True
                action_index += 1
                opposite_dir = True
            if idx + 1 < len(ego_lanes) and ego_lanes[idx + 1] - ego_lanes[idx] != 1:
                intersec_point = lines[idx].intersection(lines[idx + 1])
                if isinstance(intersec_point, MultiPoint):
                    intersec_point = Point((intersec_point[0]))
                lane_change = True
                index = len(control_points) // 2
                deleted_points = control_points[index:]
                control_points = control_points[:index]
                if directions[ego_index] == "right":
                    control_points.append((intersec_point.x, intersec_point.y))
                ego_index += 1
            iterator = 0
            while iterator < len(control_points):
                if len(waypoints) == 0 or (euclidean(control_points[iterator], waypoints[-1].get("position")) >= 1.5
                                           and (not opposite_dir
                                            or euclidean(control_points[0], control_points[iterator]) > 4)):
                    waypoint = {"position": control_points[iterator],
                                "tolerance": 2,
                                "movementMode": "_BEAMNG",
                                "lane": same_lane}
                    waypoints.append(waypoint)
                iterator += 1
            del waypoints[-1]
            if lane_change:
                iterator = 0
                while iterator < len(deleted_points):
                    if len(waypoints) == 0 \
                            or euclidean(deleted_points[iterator], waypoints[-1].get("position")) >= 1.5:
                        waypoint = {"position": deleted_points[iterator],
                                    "tolerance": 2,
                                    "lane": same_lane + 1 if stop_flag else same_lane}
                        waypoints.append(waypoint)
                    iterator += 1
                del waypoints[-1]

    init_state = {"position": waypoints[0].get("position") if add_ego_car else lanes[0].get("control_points")[0],
                  "orientation": 0}
    model = "ETK800"
    ego = {"id": "ego",
           "init_state": init_state,
           "waypoints": waypoints,
           "model": model,
           "color": "White"}
    individual.setdefault("participants", []).extend([ego])


def _add_other_0(individual, lanes_start=None, lanes_end=None):
    global PARTICIPANTS_SAMPLES
    ego_lanes = individual.get("ego_lanes")
    lanes = individual.get("lanes")

    # Drive from one opposite lane to another at an intersection.
    # Get lanes where a car can spawn, be teleported to or drive to.
    spawn_lanes = list()
    triggers = list()
    for idx, lane in enumerate(lanes):
        if idx not in ego_lanes:
            spawn_lanes.append(idx)
    i = 0
    j = 0
    action_index = 0
    waypoints = list()
    spawns = list()
    ends = list()
    while i < len(spawn_lanes):
        if individual.get("actions")[action_index] == "go":
            action_index += 1
            if i < len(spawn_lanes) - 1 and spawn_lanes[i + 1] - spawn_lanes[i] == 1:
                i += 2
            else:
                i += 1
            continue
        lines = list()
        three_way = False
        if len(spawn_lanes) > 1 and i < len(spawn_lanes) - 1 and spawn_lanes[i + 1] - spawn_lanes[i] == 1:
            spawn_indices = [spawn_lanes[i], spawn_lanes[i] + 1, spawn_lanes[i] + 2]
            spawn_index = choice(spawn_indices) if lanes_start is None else lanes_start[j]
            end_indices = [spawn_lanes[i] - 1, spawn_lanes[i], spawn_lanes[i] + 1]
            end_index = choice(end_indices) if lanes_end is None else lanes_end[j]
            while end_index == spawn_index:
                end_index = choice(end_indices)
        else:
            spawn_indices = [spawn_lanes[i], spawn_lanes[i] + 1]
            spawn_index = choice(spawn_indices) if lanes_start is None else lanes_start[j]
            end_index = spawn_lanes[i] - 1 if spawn_index == spawn_lanes[i] else spawn_lanes[i]
            end_index = end_index if lanes_end is None else lanes_end[j]
            three_way = True
        j += 1
        spawns.append(spawn_index)
        ends.append(end_index)
        spawn_point = lanes[spawn_index].get("control_points")[0] if three_way and spawn_index == spawn_lanes[i] + 1 \
            else lanes[spawn_index].get("control_points")[0]
        end_point = lanes[end_index].get("control_points")[-1] if end_index != spawn_lanes[i] - 1 \
            else lanes[end_index].get("control_points")[0]
        middle_point = lanes[spawn_index].get("control_points")[-1]
        orientation = get_angle((spawn_point[0] + 1, spawn_point[1]), spawn_point,
                                 lanes[spawn_index].get("control_points")[1]) + 180
        if (three_way and spawn_index == spawn_lanes[i] + 1) or (not three_way and spawn_index != spawn_lanes[i] + 2):
            orientation += 0
        temp_lane = lanes[spawn_lanes[i] - 1] if spawn_index != spawn_lanes[i] + 2 else lanes[spawn_lanes[i] - 2]
        temp_points = temp_lane.get("control_points")
        temp_line = LineString(temp_points)
        temp_width_per_lane = temp_lane.get("width") / (temp_lane.get("left_lanes") + temp_lane.get("right_lanes"))
        temp_offset = temp_lane.get("left_lanes") + temp_lane.get("right_lanes") - 1
        temp_offset = temp_offset * temp_width_per_lane / 2
        temp_line = temp_line.parallel_offset(temp_offset, "right")
        temp_line = multilinestrings_to_linestring(temp_line)

        # Reversed because car spawns from the opposite direction.
        left_lanes = lanes[spawn_index].get("right_lanes")
        right_lanes = lanes[spawn_index].get("left_lanes")
        width = lanes[spawn_index].get("width")
        points = lanes[spawn_index].get("control_points")
        points = points[::-1]
        line = LineString(points)
        width_per_lane = width / (left_lanes + right_lanes)
        angle = get_angle(spawn_point, middle_point, end_point)
        left = True if (240 <= angle <= 300) and right_lanes > 1 else False
        if left:
            offset = right_lanes - left_lanes - 1
            offset = offset / 2 * width_per_lane
            line = line.parallel_offset(offset, "left")
            line = multilinestrings_to_linestring(line)
            if offset < 0:
                line.coords = line.coords[::-1]
        else:
            offset = (left_lanes + right_lanes - 1) * width_per_lane / 2
            line = line.parallel_offset(offset, "right")
            line = multilinestrings_to_linestring(line)
            line.coords = line.coords[::-1]
        line.coords = b_spline(list(line.coords), PARTICIPANTS_SAMPLES).tolist()
        if left:
            line.coords = line.coords[:-4]
        else:
            line.coords = line.coords[:-9]
        lines.append(line)
        left_lanes = lanes[end_index].get("left_lanes")
        right_lanes = lanes[end_index].get("right_lanes")
        width = lanes[end_index].get("width")
        points = lanes[end_index].get("control_points")
        line = LineString(points)
        width_per_lane = width / (left_lanes + right_lanes)
        offset = (left_lanes + right_lanes - 1) * width_per_lane / 2
        if end_index != spawn_lanes[i] - 1:
            line = line.parallel_offset(offset, "right")
        else:
            line = line.parallel_offset(offset, "left")
        line = multilinestrings_to_linestring(line)
        line.coords = line.coords[::-1]
        line.coords = b_spline(list(line.coords), PARTICIPANTS_SAMPLES).tolist()
        line.coords = line.coords[PARTICIPANTS_SAMPLES // 10:]
        lines.append(line)
        for line in lines:
            for point in list(line.coords):
                if len(waypoints) == 0 or euclidean(point, waypoints[-1].get("position")) >= 1.5:
                    waypoint = {"position": point,
                                "tolerance": 2,
                                "lane": spawn_index}
                    waypoints.append(waypoint)
        trigger_point = {"position": temp_line.coords[-1],
                         "action": "spawnAndStart",
                         "tolerance": 2,
                         "triggeredBy": "ego",
                         "triggers": "other_0"}
        spawn_point = {"position": list(lines[0].coords)[0], "orientation": orientation}
        triggers.append({"triggerPoint": trigger_point, "spawnPoint": spawn_point})

        if i < len(spawn_lanes) - 1 and spawn_lanes[i + 1] - spawn_lanes[i] == 1:
            i += 2
        else:
            i += 1
        action_index += 1
    if len(waypoints) != 0:
        init_state = {"position": waypoints[0].get("position"),
                      "orientation": triggers[0].get("spawnPoint").get("orientation")}
        other = {"id": "other_{}".format(0),
                 "init_state": init_state,
                 "waypoints": waypoints,
                 "model": "ETK800",
                 "color": choice(COLORS),
                 "spawn_lanes": spawns,
                 "end_lanes": ends}
        individual["participants"].append(other)
    return spawn_lanes, triggers


def _add_other_1(individual, spawn_lanes=None, end_lane=None):
    if spawn_lanes is None:
        spawn_lanes = list()
    global COLORS
    global PARTICIPANTS_SAMPLES
    ego_lanes = individual.get("ego_lanes")
    lanes = individual.get("lanes")
    spawn_lanes.append(ego_lanes[-1])
    end_index = choice(spawn_lanes) if end_lane is None else end_lane
    temp_val = 0
    temp_index = 0
    i = 1
    while i < len(spawn_lanes):
        if spawn_lanes[i - 1] - spawn_lanes[i] != 1:
            temp_val += 1
        if end_index == spawn_lanes[i]:
            temp_index = temp_val
            break
        i += 1
    waypoints = list()
    ego_waypoints = None
    for participant in individual.get("participants"):
        if participant.get("id") == "ego":
            ego_waypoints = participant.get("waypoints")
    for waypoint in ego_waypoints[4:]:
        if waypoint.get("lane") > temp_index:
            break
        waypoints.append(waypoint)
    left_lanes = lanes[end_index].get("left_lanes")
    right_lanes = lanes[end_index].get("right_lanes")
    width = lanes[end_index].get("width")
    points = lanes[end_index].get("control_points")
    line = LineString(points)
    width_per_lane = width / (left_lanes + right_lanes)
    offset = (left_lanes + right_lanes - 1) * width_per_lane / 2
    angle = get_angle(waypoints[-2].get("position"), points[0], points[-1])
    left = True if (240 <= angle <= 300) and right_lanes > 1 else False
    if not left:
        line = line.parallel_offset(offset, "right")
    else:
        line = line.parallel_offset(offset, "left")
    line = multilinestrings_to_linestring(line)
    line.coords = line.coords[::-1]
    line.coords = b_spline(list(line.coords), PARTICIPANTS_SAMPLES).tolist()
    line.coords = line.coords[PARTICIPANTS_SAMPLES // 10:]
    for point in list(line.coords):
        if len(waypoints) == 0 or euclidean(point, waypoints[-1].get("position")) >= 1.5:
            waypoint = {"position": point,
                        "tolerance": 2,
                        "lane": temp_index}
            waypoints.append(waypoint)
    init_state = {"position": ego_waypoints[4].get("position"),
                  "orientation": 0}
    other = {"id": "other_{}".format(1),
             "init_state": init_state,
             "waypoints": waypoints,
             "model": "ETK800",
             "color": choice(COLORS),
             "end_lane": end_index}
    individual.get("participants").append(other)


def _add_other_2(individual, triggers=None):
    if triggers is None:
        triggers = list()
    global COLORS
    global PARTICIPANTS_SAMPLES
    ego_lanes = individual.get("ego_lanes")
    lanes = individual.get("lanes")
    spawn_lanes = [0]
    i = 1
    while i < len(ego_lanes):
        if i == len(ego_lanes) - 1 or (ego_lanes[i + 1] - ego_lanes[i] == 1 and ego_lanes[i] - ego_lanes[i - 1] == 1):
            spawn_lanes.append(ego_lanes[i])
        i += 1
    waypoints = list()
    for idx in spawn_lanes:
        left_lanes = lanes[idx].get("left_lanes")
        right_lanes = lanes[idx].get("right_lanes")
        width = lanes[idx].get("width")
        points = lanes[idx].get("control_points")
        points = points[::-1]
        line = LineString(points)
        width_per_lane = width / (left_lanes + right_lanes)
        offset = (left_lanes + right_lanes - 1) * width_per_lane / 2
        line = line.parallel_offset(offset, "right")
        line = multilinestrings_to_linestring(line)
        line.coords = line.coords[::-1]
        line.coords = b_spline(list(line.coords), PARTICIPANTS_SAMPLES).tolist()
        line.coords = line.coords[PARTICIPANTS_SAMPLES // 10:]
        for point in list(line.coords):
            if len(waypoints) == 0 or euclidean(point, waypoints[-1].get("position")) >= 1.5:
                waypoint = {"position": point,
                            "tolerance": 2,
                            "lane": idx}
                waypoints.append(waypoint)
        trigger_point = {"position": line.coords[-1],
                         "action": "spawnAndStart",
                         "tolerance": 2,
                         "triggeredBy": "ego",
                         "triggers": "other_2"}
        orientation = get_angle((waypoints[-1].get("position")[0] + 1, waypoints[-1].get("position")[1]),
                                waypoints[-1].get("position"), waypoints[-2].get("position")) + 180
        spawn_point = {"position": line.coords[0], "orientation": orientation}
        triggers.append({"triggerPoint": trigger_point, "spawnPoint": spawn_point})
    init_state = {"position": triggers[-len(spawn_lanes)].get("spawnPoint").get("position"),
                  "orientation": triggers[-len(spawn_lanes)].get("spawnPoint").get("orientation")}
    other = {"id": "other_{}".format(2),
             "init_state": init_state,
             "waypoints": waypoints,
             "model": "ETK800",
             "color": choice(COLORS)}
    individual["participants"].append(other)


def _add_other_participants(individual):
    spawn_lanes = list()
    triggers = list()
    sl, t = _add_other_0(individual)
    spawn_lanes.extend(sl)
    triggers.extend(t)
    # _add_other_1(individual, spawn_lanes)
    _add_other_2(individual, triggers)
    individual.setdefault("triggers", []).extend(triggers)


def _merge_lanes(population):
    """Merge lanes for each individual which will be driven by the ego car.
    :param population: Population list.
    :return: Population with merged lanes.
    """
    for individual in population:
        iterator = 1
        lanes = individual.get("lanes")
        ego_lanes = individual.get("ego_lanes")
        new_lane_list = [lanes[0]]
        while iterator < len(lanes):
            if iterator in ego_lanes and (iterator - 1) in ego_lanes:
                new_lane_list[-1].get("control_points").extend(lanes[iterator].get("control_points"))
            else:
                new_lane_list.append(lanes[iterator])
            iterator += 1
        individual["lanes"] = new_lane_list
    return population


def _add_stop_sign_triggers(last_point):
    trigger_point = {"position": last_point, "action": "stop", "tolerance": 2, "triggeredBy": "ego", "duration": 4}
    return {"triggerPoint": trigger_point}


def _handle_manual_mode(last_point, oid):
    triggers = list()
    init_state = choice(["green", "red"])
    trigger_point = {"position": last_point,
                     "action": "switchLights",
                     "tolerance": 2,
                     "triggeredBy": "ego",
                     "triggers": oid,
                     "initState": init_state,
                     "switchTo": "green" if init_state == "red" else "red"}
    triggers.append({"triggerPoint": trigger_point})
    if init_state == "green":
        triggers.append(_add_stop_sign_triggers(last_point))
    return triggers


def _add_traffic_signs(last_point, current_left_lanes, current_right_lanes, width, intersection, lane_id):
    global OID_INDEX
    global INTERSECTION_ID
    intersection_point = intersection.get("intersection_point")
    new_left_lanes = intersection.get("new_left_lanes")
    new_right_lanes = intersection.get("new_right_lanes")
    left_point = intersection.get("left_point")
    straight_point = intersection.get("straight_point")
    right_point = intersection.get("right_point")
    layout = intersection.get("layout")
    number_of_ways = intersection.get("number_of_ways")
    direction = intersection.get("direction")

    def opposite_direction(my_point, my_right_point, num_lanes):
        line = LineString([intersection_point, my_point])
        my_z_rot = int(round(get_angle(temp_point, line.coords[0], line.coords[1]))) + 180
        angle = int(round(get_angle(my_right_point, line.coords[0], line.coords[1])))
        offset = 0.1 if angle <= 270 else ((angle - 270) / 10 + 0.2) * 1.3
        fac = (old_width * (current_left_lanes + current_right_lanes) / 2 + offset) / line.length
        vector = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        vector = affinity.rotate(vector, -90, vector.coords[1])
        fac = (new_width * (new_left_lanes + new_right_lanes) / 2 + 0.2) / vector.length
        vector = affinity.scale(vector, xfact=fac, yfact=fac, origin=vector.coords[1])
        my_position = vector.coords[0]
        my_mode = "off" if mode is not None and mode == "manual" else mode
        if sign_on_my_lane == "stopsign":
            obstacles.append({"name": "prioritysign", "position": my_position, "zRot": my_z_rot,
                              "intersection_id": INTERSECTION_ID})
        elif sign_on_my_lane == "prioritysign":
            obstacles.append({"name": "stopsign", "position": my_position, "zRot": my_z_rot,
                              "intersection_id": INTERSECTION_ID})
        else:
            if num_lanes == 1:
                obstacles.append({"name": "trafficlightsingle", "position": my_position, "zRot": my_z_rot,
                                  "mode": my_mode, "sign": "priority" if pole_sign == "yield" else "yield",
                                  "intersection_id": INTERSECTION_ID})
            else:
                obstacles.append({"name": "trafficlightdouble", "position": my_position, "zRot": my_z_rot,
                                  "mode": my_mode, "sign": "priority" if pole_sign == "yield" else "yield",
                                  "intersection_id": INTERSECTION_ID})

    def my_direction(my_point, my_right_point):
        line = LineString([intersection_point, my_point])
        my_z_rot = int(round(get_angle(temp_point, line.coords[0], line.coords[1]))) + 180
        angle = int(round(get_angle(my_right_point, line.coords[0], line.coords[1])))
        offset = 0.1 if angle <= 270 else ((angle - 270) / 10 + 0.2) * 1.3
        fac = (new_width * (new_left_lanes + new_right_lanes) / 2 + offset) / line.length
        vector = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        vector = affinity.rotate(vector, -90, vector.coords[1])
        fac = (old_width * (current_left_lanes + current_right_lanes) / 2 + 0.2) / vector.length
        vector = affinity.scale(vector, xfact=fac, yfact=fac, origin=vector.coords[1])
        my_position = vector.coords[0]
        return my_position, my_z_rot

    modes = ["off", "flashing", "manual"]
    mode = choice(modes)
    oid = None
    if mode == "manual":
        oid = "traffic_light_manual_" + str(OID_INDEX)
        OID_INDEX += 1

    # Calculate traffic sign position.
    old_width = width / (current_left_lanes + current_right_lanes)
    new_width = intersection.get("new_width") / (new_left_lanes + new_right_lanes)
    obstacles = list()
    temp_point = (intersection_point[0] + 5, intersection_point[1])

    # Bottom direction.
    position, z_rot = my_direction(last_point, right_point)
    pole_sign = "yield" if random() < 0.5 else "priority"
    if current_right_lanes == 1:
        if current_left_lanes == 1 and new_left_lanes == 1 and new_right_lanes == 1 and random() < 0.5:
            if random() < 0.5:
                obstacles.append({"name": "stopsign", "position": position, "zRot": z_rot,
                                  "intersection_id": INTERSECTION_ID, "facingEgo": True, "lane_id": lane_id})
            else:
                obstacles.append({"name": "prioritysign", "position": position, "zRot": z_rot,
                                  "intersection_id": INTERSECTION_ID, "facingEgo": True, "lane_id": lane_id})
        else:
            obstacles.append({"name": "trafficlightsingle", "position": position, "zRot": z_rot, "mode": mode,
                              "sign": pole_sign, "oid": oid, "intersection_id": INTERSECTION_ID, "facingEgo": True,
                              "lane_id": lane_id})
    else:
        obstacles.append({"name": "trafficlightdouble", "position": position, "zRot": z_rot, "mode": mode,
                          "sign": pole_sign, "oid": oid, "intersection_id": INTERSECTION_ID, "facingEgo": True,
                          "lane_id": lane_id})
    sign_on_my_lane = obstacles[0].get("name")

    triggers = list()
    if sign_on_my_lane.startswith("trafficlight") and mode == "manual":
        triggers = _handle_manual_mode(last_point, oid)

    if sign_on_my_lane.startswith("trafficlight") and mode != "manual" and pole_sign == "yield":
        triggers.append(_add_stop_sign_triggers(last_point))

    if sign_on_my_lane.startswith("stop"):
        triggers.append(_add_stop_sign_triggers(last_point))

    if sign_on_my_lane.startswith("priority") or obstacles[-1].get("sign") == "priority" \
            or (len(triggers) != 0 and triggers[0].get("triggerPoint").get("switchTo") == "green"):
        action = "go"
    else:
        action = "stop"

    # Left direction.
    if number_of_ways == 4 or direction == "left" or layout == "left":
        if (direction == "left" or (direction == "straight" and number_of_ways == 4)) and layout != "left":
            opposite_direction(left_point, last_point, new_left_lanes)
        else:
            opposite_direction(left_point, last_point, new_right_lanes)

    # Top direction.
    if number_of_ways == 4 or direction == "straight" or layout == "straight":
        if sign_on_my_lane.startswith("trafficlight"):
            if current_left_lanes == 1:
                this_sign = "trafficlightsingle"
            else:
                this_sign = "trafficlightdouble"
        else:
            this_sign = sign_on_my_lane
        position, z_rot = my_direction(straight_point, left_point)
        obstacles.append({"name": this_sign, "position": position,
                          "zRot": z_rot, "intersection_id": INTERSECTION_ID})
        if sign_on_my_lane.startswith("trafficlight"):
            mode = "off" if mode == "manual" else mode
            obstacles[-1]["mode"] = mode
            obstacles[-1]["sign"] = pole_sign

    # Right direction.
    if number_of_ways == 4 or direction == "right" or layout == "right":
        if direction == "right" and layout != "right":
            opposite_direction(right_point, straight_point, new_left_lanes)
        else:
            opposite_direction(right_point, straight_point, new_right_lanes)

    INTERSECTION_ID += 1
    return obstacles, triggers, action


def _preparation(population, traffic=True, add_ego_car=True, add_parked_cars=True):
    for individual in population:
        if add_parked_cars:
            _add_parked_cars(individual)
        _add_ego_car(individual, add_ego_car)
        if traffic:
            _add_other_participants(individual)
        if traffic or add_ego_car:
            calc_speed_waypoints(individual["participants"])


class FuelConsumptionTestGenerator:

    def __init__(self, files_name="urban", traffic=True, spline_degree=2, max_tries=20, population_size=1,
                 add_ego=True):
        self.FILES_NAME = files_name                  # File name for XML file series.
        self.TRAFFIC = traffic                        # Enable traffic or not.
        self.SPLINE_DEGREE = spline_degree            # Sharpness of curves.
        self.MAX_TRIES = max_tries                    # Maximum number of invalid generated points/segments.
        self.POPULATION_SIZE = population_size        # Minimum number of generated roads for each generation.
        self.NUMBER_ELITES = 2                        # Number of best kept test cases.
        self.MIN_SEGMENT_LENGTH = 10                  # Minimum length of a road segment.
        self.MAX_SEGMENT_LENGTH = 30                  # Maximum length of a road segment.
        self.MIN_NODES = 6                            # Minimum number of control points for each road.
        self.MAX_NODES = 16                           # Maximum number of control points for each road.
        self.population_list = list()
        self.intersection_length = 50
        self.opposite_lane = 30
        self.intersecting_length = 20
        self.MAX_LEFT_LANES = 2
        self.MAX_RIGHT_LANES = 2
        self.MAX_WIDTH = 5
        self.mutation_probability = 0.5  # I have eight mutatable properties.
        self.ADD_EGO_CAR = add_ego
        print(colored("##################", attrs=["bold"]))
        print(colored("##################", "red", attrs=["bold"]))
        print(colored("##################", "yellow", attrs=["bold"]))

    def _bspline(self, lanes):
        """Calculate {@code samples} samples on a bspline. This is the road representation function.
        :param lanes: List of lanes.
        :return: List of arrays with samples, representing a bspline of the given control points of the lanes.
        """
        splined_list = list()
        for lane in lanes:
            samples = lane.get("samples")
            # Calculate splines for each lane.
            point_list = asarray(lane.get("control_points"))
            count = len(point_list)
            degree = clip(self.SPLINE_DEGREE, 1, count - 1)

            # Calculate knot vector.
            kv = concatenate(([0] * degree, arange(count - degree + 1), [count - degree] * degree))

            # Calculate query range.
            u = linspace(False, (count - degree), samples)

            # Calculate result.
            splined_list.append({"control_points": around(array(splev(u, (kv, point_list.T, degree))).T, 3),
                                 "width": lane.get("width")})
        return splined_list

    def _add_segment(self, last_point, penultimate_point=None):
        """Generates a new random point within a given range.
        :param last_point: Last point of the control point list as tuple.
        :param penultimate_point: Point before the last point as tuple.
        :return: A new random point as tuple.
        """
        x_min = int(round(last_point[0] - self.MAX_SEGMENT_LENGTH))
        x_max = int(round(last_point[0] + self.MAX_SEGMENT_LENGTH))
        y_min = int(round(last_point[1] - self.MAX_SEGMENT_LENGTH))
        y_max = int(round(last_point[1] + self.MAX_SEGMENT_LENGTH))
        while True:
            x_pos = randint(x_min, x_max)
            y_pos = randint(y_min, y_max)
            point = (x_pos, y_pos)
            dist = Point(last_point).distance(Point(point))
            deg = None
            if penultimate_point is not None:
                deg = get_angle((penultimate_point[0], penultimate_point[1]),
                                (last_point[0], last_point[1]),
                                point)
            if self.MAX_SEGMENT_LENGTH >= dist >= self.MIN_SEGMENT_LENGTH:
                if penultimate_point is not None:
                    if MIN_DEGREES <= deg <= MAX_DEGREES:
                        return point
                else:
                    return point

    def _create_urban_environment(self):
        global MIN_DEGREES, MAX_DEGREES
        print(colored("Creating urban scenario...", "grey", attrs=['bold']))
        p0 = (1, 0)
        p1 = (30, 0)
        p2 = (45, 0)
        left_lanes = randint(1, self.MAX_LEFT_LANES)
        right_lanes = randint(1, self.MAX_RIGHT_LANES)
        MIN_DEGREES, MAX_DEGREES = calc_min_max_angles(left_lanes + right_lanes)
        lanes = [{"control_points": [p0, p1, p2], "width": calc_width(left_lanes, right_lanes),
                  "left_lanes": left_lanes, "right_lanes": right_lanes, "samples": 100, "type": "normal"}]
        ego_lanes = [0]
        intersection_lanes = list()
        obstacles = list()
        directions = list()
        triggers = list()
        actions = list()
        tries = 0
        lane_index = 0
        number_of_pieces = 3
        one_intersection = False
        intersection_possible = True
        intersection_probability = 0.25
        lines_of_roads = convert_points_to_lines(lanes)
        last_point = p2
        while (number_of_pieces <= self.MAX_NODES and tries <= self.MAX_TRIES) \
                or len(lanes[lane_index].get("control_points")) == 1:
            control_points = lanes[lane_index].get("control_points")
            if intersection_possible and ((number_of_pieces == self.MAX_NODES - 1 and not one_intersection)
                                          or random() <= intersection_probability) and len(control_points) > 1:
                # Add intersection, if possible.
                intersection = self._create_intersection(control_points[-1], control_points[-2])
                intersection_items = get_lanes_of_intersection(intersection, control_points[-1],
                                                               lanes[lane_index].get("width"),
                                                               lanes[lane_index].get("left_lanes"),
                                                               lanes[lane_index].get("right_lanes"), lane_index)
                new_line, new_lane_line = get_intersection_lines(control_points[-1], intersection)
                temp_list = deepcopy(lanes)
                temp_list.extend(intersection_items.get("lanes"))
                temp_list = self._bspline(temp_list)
                control_points_lines = convert_points_to_lines(temp_list)
                width_lines = get_width_lines(temp_list)
                intersection_lanes_temp = deepcopy(intersection_lanes)
                intersection_lanes_temp.extend(intersection_items.get("intersection_lanes"))
                if not intersection_check_last(lines_of_roads, new_line) \
                        and not intersection_check_last(lines_of_roads, new_lane_line) \
                        and not intersection_check_width(width_lines, control_points_lines, intersection_lanes_temp):
                    left_lanes = intersection_items.get("left_lanes")
                    right_lanes = intersection_items.get("right_lanes")
                    obs, trs, action = _add_traffic_signs(control_points[-1], lanes[lane_index].get("left_lanes"),
                                                          lanes[lane_index].get("right_lanes"),
                                                          lanes[lane_index].get("width"), intersection, lane_index + 1)
                    actions.append(action)
                    obstacles.extend(obs)
                    triggers.extend(trs)
                    directions.append(intersection.get("direction"))
                    lanes.extend(intersection_items.get("lanes"))
                    ego_lanes.extend(intersection_items.get("ego_lanes"))
                    last_point = intersection_items.get("last_point")
                    intersection_lanes.extend(intersection_items.get("intersection_lanes"))
                    lane_index = intersection_items.get("lane_index")
                    MIN_DEGREES, MAX_DEGREES = calc_min_max_angles(left_lanes + right_lanes)
                    lines_of_roads = convert_points_to_lines(lanes)
                    number_of_pieces += 1
                    one_intersection = True
                else:
                    intersection_possible = False
            # Add segment, if possible.
            control_points = lanes[lane_index].get("control_points")
            if len(control_points) == 1:
                new_point = self._add_segment(control_points[0])
            else:
                new_point = self._add_segment(control_points[-1], control_points[-2])
            new_line = LineString([(control_points[-1][0], control_points[-1][1]),
                                   (new_point[0], new_point[1])])
            temp_list = deepcopy(lanes)
            temp_list[lane_index].get("control_points").append(new_point)
            temp_list = self._bspline(temp_list)
            control_points_lines = convert_points_to_lines(temp_list)
            width_lines = get_width_lines(temp_list)
            if not intersection_check_last(lines_of_roads, new_line, max_intersections=0) \
                    and not intersection_check_width(width_lines, control_points_lines, intersection_lanes):
                lanes[lane_index].get("control_points").append(new_point)
                intersection_possible = True
                tries = 0
                number_of_pieces += 1
                lines_of_roads = convert_points_to_lines(lanes)
                last_point = new_point
            else:
                tries += 1
        if number_of_pieces >= self.MIN_NODES and one_intersection:
            print(colored("Finished creating urban scenario!", "grey", attrs=['bold']))
            return {"lanes": lanes, "success_point": {"position": last_point, "tolerance": 10}, "ego_lanes": ego_lanes,
                    "obstacles": obstacles, "directions": directions, "triggers": triggers, "tod": random(),
                    "intersection_lanes": intersection_lanes, "actions": actions}
        else:
            print(colored("Couldn't create a valid road network. Restarting...", "grey", attrs=['bold']))

    def _create_intersection(self, last_point, penultimate_point):
        layout = None
        random_number = random()
        if random_number <= 0.33:
            direction = "straight"
        elif 0.33 < random_number <= 0.66:
            direction = "left"
        else:
            direction = "right"
        if random() <= 0.5:
            number_of_ways = 4
        else:
            number_of_ways = 3
            if direction == "straight":
                if random() <= 0.5:
                    layout = "left"
                else:
                    layout = "right"
            elif direction == "left":
                if random() <= 0.5:
                    layout = "straight"
                else:
                    layout = "right"
            else:
                if random() <= 0.5:
                    layout = "straight"
                else:
                    layout = "left"
        line = LineString([(penultimate_point[0], penultimate_point[1]),
                           (last_point[0], last_point[1])])
        fac = get_resize_factor_intersection(line.length, self.intersection_length)
        angle = randint(-10, 10)
        line_rot = affinity.rotate(line, angle, line.coords[0])
        line_intersection = affinity.scale(line_rot, xfact=fac, yfact=fac, origin=line_rot.coords[0])
        straight_point = list(shape(line_intersection).coords)[1]
        fac = get_resize_factor_intersection(line.length, self.intersecting_length)
        line_intersection = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        intersection_point = list(shape(line_intersection).coords)[1]
        line = LineString([(intersection_point[0], intersection_point[1]),
                           (line.coords[1][0], line.coords[1][1])])
        fac = get_resize_factor_intersection(line.length, self.opposite_lane)

        # Right turn.
        angle = randint(-110, -70)

        line_rot1 = affinity.rotate(line, angle, line.coords[0])
        line_rot1 = affinity.scale(line_rot1, xfact=fac, yfact=fac,
                                   origin=line_rot1.coords[0])

        # Left turn.
        angle = randint(70, 110)
        line_rot2 = affinity.rotate(line, angle, line.coords[0])
        line_rot2 = affinity.scale(line_rot2, xfact=fac, yfact=fac,
                                   origin=line_rot2.coords[0])
        p1 = (list(shape(line_rot1).coords)[1][0], list(shape(line_rot1).coords)[1][1])
        p2 = (list(shape(line_rot2).coords)[1][0], list(shape(line_rot2).coords)[1][1])
        left_lanes = randint(1, self.MAX_LEFT_LANES)
        right_lanes = randint(1, self.MAX_RIGHT_LANES)
        return {"intersection_point": intersection_point,
                "straight_point": straight_point,
                "left_point": p1,
                "right_point": p2,
                "direction": direction,
                "number_of_ways": number_of_ways,
                "layout": layout,
                "new_left_lanes": left_lanes,
                "new_right_lanes": right_lanes,
                "new_width": calc_width(left_lanes, right_lanes)}

    def _spline_population(self, population):
        """Converts the control points list of every individual to a bspline
         list and adds the width parameter as well as the ego car.
        :param population: List of individuals.
        :return: List of individuals with bsplined control points.
        """
        for individual in population:
            splined_list = self._bspline(individual.get("lanes"))
            iterator = 0
            while iterator < len(splined_list):
                lane = splined_list[iterator]
                individual.get("lanes")[iterator]["control_points"] = lane.get("control_points").tolist()
                iterator += 1
        return population

    def _create_start_population(self):
        """Creates and returns an initial population."""
        startpop = list()
        i = 0
        while len(startpop) < self.POPULATION_SIZE: 
            urban = self._create_urban_environment()
            if urban is not None:
                startpop.append({"lanes": urban.get("lanes"),
                                 "file_name": self.FILES_NAME,
                                 "obstacles": urban.get("obstacles"),
                                 "success_point": urban.get("success_point"),
                                 "ego_lanes": urban.get("ego_lanes"),
                                 "directions": urban.get("directions"),
                                 "fitness": 0,
                                 "triggers": urban.get("triggers"),
                                 "tod": urban.get("tod"),
                                 "intersection_lanes": urban.get("intersection_lanes"),
                                 "actions": urban.get("actions")})
                i += 1
        return startpop

    def create_test_cases(self):
        self.population_list = self._create_start_population()
        temp_list = deepcopy(self.population_list)
        temp_list = self._spline_population(temp_list)
        _preparation(population=temp_list, traffic=self.TRAFFIC, add_ego_car=self.ADD_EGO_CAR)
        temp_list = _merge_lanes(temp_list)
        build_all_xml(temp_list)
        print(colored("Population finished.", "grey", attrs=['bold']))

    def get_test(self):
        """Returns the two first test files starting with "files_name".
        :return: Tuple of the path to the dbe and dbc file.
        """
        destination_path = path.dirname(path.realpath(__file__)) + "\\scenario"
        xml_names = destination_path + "\\" + self.FILES_NAME + "*"
        i = 0
        # self.genetic_algorithm()
        self.create_test_cases()
        matches = glob(xml_names)
        while i < self.POPULATION_SIZE * 2 - 1:
            yield Path(matches[i + 1]), Path(matches[i])
            i += 2

    def on_test_finished(self, score):
        """This method is called after a test was finished. Also updates fitness value of an individual.
        :return: Void.
        """
        i = 0
        while i < self.POPULATION_SIZE:
            self.population_list[i]["fitness"] = score
            i += 1
            yield

# TODO Desired features:
#       TODO Buggy traffic
#       TODO Right turns are buggy
#       TODO Comments
#       TODO Refactor
#       TODO Rename files
#       TODO XML failure criteria is actually read
#       TODO Check if other participants fail (damage etc.)
#       TODO Test setup.py
#       TODO lanes -> roads
#       TODO control_point_lines -> polylines
#       TODO Test without ego car
#       TODO Retest experiments
#       TODO ReadMe
#       TODO Remove TODOS

# TODO Future Work:
#       TODO Make traffic participant 2 spawn earlier
#       TODO Make all objects collidable
#       TODO Fix Shapely errors
#       TODO Traffic for standing at the traffic light or stop sign
#       TODO Improve speed of car
#       TODO Teleporting cars shouldnt be visible to ego(line triggered by ego, teleport by other)
#       TODO Test oracle: Out of map
#       TODO Roundabouts
#       TODO Add weather presets
#       TODO Converter:
#           TODO Add input checking
#           TODO Implement Sensor deployment
#       TODO Manual traffic lights for all directions
#       TODO Fix lane markings
#       TODO Fix BNG errors and warnings
#       TODO Parallel offset instead of width lines
#       TODO Improve performance
#       TODO Crossover
#       TODO Traffic light color cant be determined correctly
#       TODO Ego shouldn't stop at yield sign when other car spawns on opposite lane
#       TODO Find more/other ways to save fuel by looking at the driving style.
#        (engine brake before intersection&smooth braking, constant dist to car in front of ego)

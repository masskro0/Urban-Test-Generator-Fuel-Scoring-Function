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
    get_resize_factor_intersection, multilinestrings_to_linestring
from utils.validity_checks import intersection_check_width, intersection_check_last, intersection_check_all
from utils.xml_creator import build_all_xml
from xml_converter.converter import b_spline
from utils.plotter import plotter

MIN_DEGREES = 90
MAX_DEGREES = 270
OID_INDEX = 0
INTERSECTION_ID = 0


def _add_ego_car(individual):
    """Adds the ego car to the criteria xml file. Movement mode can be assigned manually. Each control point is one
    waypoint.
    :param individual: Individual of the population.
    :return: Void.
    """
    samples = 57
    lanes = individual.get("lanes")
    ego_lanes = individual.get("ego_lanes")
    directions = individual.get("directions")
    waypoints = list()
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
    for idx, lane in enumerate(lines):
        control_points = list(lines[idx].coords)
        opposite_dir = False
        deleted_points = list()
        lane_change = False
        if idx != 0 and ego_lanes[idx] - ego_lanes[idx - 1] != 1:
            same_lane += 1
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
            if len(waypoints) == 0 or (euclidean(control_points[iterator], waypoints[-1].get("position")) >= 1.5 and
                                       (not opposite_dir
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
                if len(waypoints) == 0 or euclidean(deleted_points[iterator], waypoints[-1].get("position")) >= 1.5:
                    waypoint = {"position": deleted_points[iterator],
                                "tolerance": 2,
                                "lane": same_lane + 1}
                    waypoints.append(waypoint)
                iterator += 1
            del waypoints[-1]

    init_state = {"position": waypoints[0].get("position"),
                  "orientation": 0}
    model = "ETK800"
    ego = {"id": "ego",
           "init_state": init_state,
           "waypoints": waypoints,
           "model": model,
           "color": "White"}
    individual["participants"] = [ego]


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
        parked_cars.append({"name": "golf", "position": (position[0][0], position[0][1]), "zRot": position[1],
                            "color": color, "lane": position[2]})
    individual["obstacles"].extend(parked_cars)


def _add_other_participants(individual):
    colors = ["White", "Red", "Green", "Yellow", "Black", "Blue", "Orange", "Gray", "Purple"]
    ego_lanes = individual.get("ego_lanes")
    lanes = individual.get("lanes")

    # Drive from one opposite lane to another at an intersection.
    # Get lanes where a car can spawn, be teleported to or drive to.
    spawn_lanes = list()
    triggers = list()
    for idx, lane in enumerate(lanes):
        if idx not in ego_lanes:
            spawn_lanes.append(idx)
    samples = 45
    i = 0
    waypoints = list()
    while i < len(spawn_lanes):
        lines = list()
        three_way = False
        if len(spawn_lanes) > 1 and i < len(spawn_lanes) - 1 and spawn_lanes[i + 1] - spawn_lanes[i] == 1:
            spawn_indices = [spawn_lanes[i], spawn_lanes[i] + 1, spawn_lanes[i] + 2]
            spawn_index = choice(spawn_indices)
            end_indices = [spawn_lanes[i] - 1, spawn_lanes[i], spawn_lanes[i] + 1]
            end_index = choice(end_indices)
            while end_index == spawn_index:
                end_index = choice(end_indices)
        else:
            spawn_indices = [spawn_lanes[i] - 1, spawn_lanes[i], spawn_lanes[i] + 1]
            spawn_index = choice(spawn_indices)
            end_index = spawn_lanes[i] - 1 if spawn_index == spawn_lanes[i] else spawn_lanes[i]
            three_way = True
        spawn_point = lanes[spawn_index].get("control_points")[-1] if three_way and spawn_index == spawn_lanes[i] + 1 \
            else lanes[spawn_index].get("control_points")[0]
        end_point = lanes[end_index].get("control_points")[-1] if end_index != spawn_lanes[i] - 1 \
            else lanes[end_index].get("control_points")[0]
        middle_point = lanes[spawn_index].get("control_points")[0]
        orientation = get_angle((spawn_point[0] + 1, spawn_point[1]), spawn_point, middle_point)
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
            if offset < 0:
                line.coords = line.coords[::-1]
        else:
            offset = (left_lanes + right_lanes - 1) * width_per_lane / 2
            line = line.parallel_offset(offset, "right")
            line = multilinestrings_to_linestring(line)
            line.coords = line.coords[::-1]
        line.coords = b_spline(list(line.coords), samples).tolist()
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
        line.coords = b_spline(list(line.coords), samples).tolist()
        line.coords = line.coords[samples // 10:]
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
    init_state = {"position": waypoints[0].get("position"),
                  "orientation": triggers[0].get("spawnPoint").get("orientation")}
    other = {"id": "other_{}".format(0),
             "init_state": init_state,
             "waypoints": waypoints,
             "model": "ETK800",
             "color": choice(colors)}
    individual["participants"].append(other)

    spawn_lanes.append(ego_lanes[-1])
    end_index = choice(spawn_lanes)
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
    for waypoint in ego_waypoints[3:]:
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
    line.coords = b_spline(list(line.coords), samples).tolist()
    line.coords = line.coords[samples // 10:]
    for point in list(line.coords):
        if len(waypoints) == 0 or euclidean(point, waypoints[-1].get("position")) >= 1.5:
            waypoint = {"position": point,
                        "tolerance": 2,
                        "lane": temp_index}
            waypoints.append(waypoint)
    init_state = {"position": ego_waypoints[2].get("position"),
                  "orientation": 0}
    other = {"id": "other_{}".format(1),
             "init_state": init_state,
             "waypoints": waypoints,
             "model": "ETK800",
             "color": choice(colors)}
    individual.get("participants").append(other)

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
        line.coords = b_spline(list(line.coords), samples).tolist()
        line.coords = line.coords[samples // 10:]
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
             "color": choice(colors)}
    individual.get("participants").append(other)
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


def _handle_manual_mode(last_point, oid):
    triggers = list()
    trigger_point = {"position": last_point,
                     "action": "switchLights",
                     "tolerance": 2,
                     "triggeredBy": "ego",
                     "triggers": oid,
                     "initState": "green",
                     "switchTo": "red"}
    triggers.append({"triggerPoint": trigger_point})
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

    def opposite_direction(my_point, my_right_point):
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
        else:
            if new_right_lanes == 1:
                obstacles.append({"name": "trafficlightsingle", "position": my_position, "zRot": my_z_rot,
                                  "mode": my_mode, "sign": "priority", "intersection_id": INTERSECTION_ID})
            else:
                obstacles.append({"name": "trafficlightdouble", "position": my_position, "zRot": my_z_rot,
                                  "mode": my_mode, "sign": "priority", "intersection_id": INTERSECTION_ID})

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

    modes = ["off", "blinking", "manual"]
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
    if current_right_lanes == 1:
        if current_left_lanes == 1:
            obstacles.append({"name": "stopsign", "position": position, "zRot": z_rot,
                              "intersection_id": INTERSECTION_ID, "facingEgo": True, "lane_id": lane_id})
        else:
            obstacles.append({"name": "trafficlightsingle", "position": position, "zRot": z_rot, "mode": mode,
                              "sign": "yield", "oid": oid, "intersection_id": INTERSECTION_ID, "facingEgo": True,
                              "lane_id": lane_id})
    else:
        obstacles.append({"name": "trafficlightdouble", "position": position, "zRot": z_rot, "mode": mode,
                          "sign": "yield", "oid": oid, "intersection_id": INTERSECTION_ID, "facingEgo": True,
                          "lane_id": lane_id})
    sign_on_my_lane = obstacles[0].get("name")

    triggers = list()
    if sign_on_my_lane.startswith("trafficlight") and mode == "manual":
        triggers = _handle_manual_mode(last_point, oid)

    # Left direction.
    if number_of_ways == 4 or direction == "left" or layout == "left":
        opposite_direction(left_point, last_point)

    # Top direction.
    if number_of_ways == 4 or direction == "straight" or layout == "straight":
        position, z_rot = my_direction(straight_point, left_point)
        obstacles.append({"name": sign_on_my_lane, "position": position,
                          "zRot": z_rot, "intersection_id": INTERSECTION_ID})
        if sign_on_my_lane.startswith("trafficlight"):
            mode = "off" if mode == "manual" else mode
            obstacles[-1]["mode"] = mode
            obstacles[-1]["sign"] = "yield"

    # Right direction.
    if number_of_ways == 4 or direction == "right" or layout == "right":
        opposite_direction(right_point, straight_point)
    INTERSECTION_ID += 1
    return obstacles, triggers


def _preparation(population):
    for individual in population:
        _add_parked_cars(individual)
        _add_ego_car(individual)
        _add_other_participants(individual)


def _get_connected_lanes(lanes, ego_lanes, directions):
    connected_lanes = list()
    i = 0
    j = 0
    while i < len(ego_lanes):
        if i != 0 and (ego_lanes[i] - ego_lanes[i - 1] == 1 or (j != len(directions) and directions[j] == "straight")):
            connected_lanes[-1].append(ego_lanes[i])
        else:
            connected_lanes.append([ego_lanes[i]])
        if i != 0 and ego_lanes[i] - ego_lanes[i - 1] != 1:
            if directions[j] == "straight":
                connected_lanes.insert(-2, [ego_lanes[i] - 1])
            else:
                penultimate_point = lanes[ego_lanes[i - 1]].get("control_points")[0]
                last_point = lanes[ego_lanes[i - 1]].get("control_points")[1]
                next_point = lanes[ego_lanes[i] - 1].get("control_points")[1]
                if 135 <= get_angle(penultimate_point, last_point, next_point) <= 225:
                    connected_lanes[-2].append(ego_lanes[i] - 1)
                else:
                    connected_lanes[-1].append(ego_lanes[i] - 1)
            if ego_lanes[i] - ego_lanes[i - 1] == 3:
                if directions[j] == "straight":
                    connected_lanes.insert(-2, [ego_lanes[i] - 2])
                else:
                    penultimate_point = lanes[ego_lanes[i - 1]].get("control_points")[0]
                    last_point = lanes[ego_lanes[i - 1]].get("control_points")[1]
                    next_point = lanes[ego_lanes[i] - 2].get("control_points")[1]
                    if 135 <= get_angle(penultimate_point, last_point, next_point) <= 225:
                        connected_lanes[-2].append(ego_lanes[i] - 2)
                    else:
                        connected_lanes[-1].append(ego_lanes[i] - 2)
            j += 1
        i += 1
        return connected_lanes


def _create_intersections_manual(individual):
    """Creates intersection "type" manually.
    :param individual: Individual of the population list.
    :return: List of intersections.
    """
    intersections = list()
    for idx, intersection_list in enumerate(individual.get("intersection_lanes")):
        intersection = dict()
        intersection["intersection_point"] = individual.get("lanes")[intersection_list[0]].get("control_points")[1]
        intersection["last_point"] = individual.get("lanes")[intersection_list[0]].get("control_points")[0]
        i = 1
        while i < len(intersection_list):
            point = individual.get("lanes")[intersection_list[i]].get("control_points")[1]
            deg = get_angle(intersection.get("last_point"), intersection.get("intersection_point"), point)
            if 135 <= deg <= 225:
                intersection["straight_point"] = point
            elif 45 <= deg < 135:
                intersection["right_point"] = point
            else:
                intersection["left_point"] = point
            i += 1
        intersection["direction"] = individual.get("directions")[idx]
        intersection["number_of_ways"] = len(intersection_list)
        if len(intersection_list) == 3:
            if intersection["direction"] == "left":
                if intersection.get("straight_point") is not None:
                    intersection["layout"] = "straight"
                    intersection["right_point"] = (intersection["intersection_point"][0] + 1,
                                                   intersection["intersection_point"][1])
                else:
                    intersection["layout"] = "right"
                    intersection["straight_point"] = (intersection["intersection_point"][0],
                                                      intersection["intersection_point"][1] + 1)
            elif intersection["direction"] == "straight":
                if intersection.get("left_point") is not None:
                    intersection["layout"] = "left"
                    intersection["right_point"] = (intersection["intersection_point"][0] + 1,
                                                   intersection["intersection_point"][1])
                else:
                    intersection["layout"] = "right"
                    intersection["left_point"] = (intersection["intersection_point"][0] - 1,
                                                  intersection["intersection_point"][1])
            else:
                if intersection.get("straight_point") is not None:
                    intersection["layout"] = "straight"
                    intersection["left_point"] = (intersection["intersection_point"][0] - 1,
                                                  intersection["intersection_point"][1])
                else:
                    intersection["layout"] = "left"
                    intersection["straight_point"] = (intersection["intersection_point"][0],
                                                      intersection["intersection_point"][1] + 1)
        if intersection["direction"] == "straight":
            lane = individual.get("lanes")[intersection_list[1]]
        else:
            lane = individual.get("lanes")[intersection_list[-1]]
        intersection["new_left_lanes"] = lane.get("left_lanes")
        intersection["new_right_lanes"] = lane.get("right_lanes")
        intersection["new_width"] = lane.get("width")
        lane = individual.get("lanes")[intersection_list[0]]
        intersection["current_left_lanes"] = lane.get("left_lanes")
        intersection["current_right_lanes"] = lane.get("right_lanes")
        intersection["current_width"] = lane.get("width")
        intersections.append(intersection)
    return intersections


class FuelConsumptionTestGenerator:

    def __init__(self):
        self.files_name = "urban"
        self.SPLINE_DEGREE = 3  # Sharpness of curves
        self.MAX_TRIES = 20  # Maximum number of invalid generated points/segments
        self.POPULATION_SIZE = 3  # Minimum number of generated roads for each generation
        self.NUMBER_ELITES = 2  # Number of best kept test cases.
        self.MIN_SEGMENT_LENGTH = 15  # Minimum length of a road segment
        self.MAX_SEGMENT_LENGTH = 30  # Maximum length of a road segment
        self.MIN_NODES = 6  # Minimum number of control points for each road
        self.MAX_NODES = 12  # Maximum number of control points for each road
        self.population_list = list()
        self.intersection_length = 50
        self.opposite_lane = 30
        self.intersecting_length = 20
        self.MAX_LEFT_LANES = 2
        self.MAX_RIGHT_LANES = 2
        self.MAX_WIDTH = 5
        self.mutation_probability = 0.5  # I have eight mutatable properties.
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
        lanes = [{"control_points": [p0, p1, p2], "width": calc_width(left_lanes, right_lanes),
                  "left_lanes": left_lanes, "right_lanes": right_lanes, "samples": 100, "type": "normal"}]
        ego_lanes = [0]
        intersection_lanes = list()
        obstacles = list()
        directions = list()
        triggers = list()
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
                    obs, trs = _add_traffic_signs(control_points[-1], lanes[lane_index].get("left_lanes"),
                                                  lanes[lane_index].get("right_lanes"),
                                                  lanes[lane_index].get("width"), intersection, lane_index + 1)
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
            return {"lanes": lanes, "success_point": {"position": last_point, "tolerance": 3}, "ego_lanes": ego_lanes,
                    "obstacles": obstacles, "directions": directions, "triggers": triggers, "tod": random(),
                    "intersection_lanes": intersection_lanes}
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
        line_intersection = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        straight_point = list(shape(line_intersection).coords)[1]
        fac = get_resize_factor_intersection(line.length, self.intersecting_length)
        line_intersection = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        intersection_point = list(shape(line_intersection).coords)[1]
        line = LineString([(intersection_point[0], intersection_point[1]),
                           (line.coords[1][0], line.coords[1][1])])
        fac = get_resize_factor_intersection(line.length, self.opposite_lane)

        # Right turn.
        right_turn_angle = randint(-110, -70)

        line_rot1 = affinity.rotate(line, right_turn_angle, line.coords[0])
        line_rot1 = affinity.scale(line_rot1, xfact=fac, yfact=fac,
                                   origin=line_rot1.coords[0])

        # Left turn.
        left_turn_angle = randint(70, 110)
        line_rot2 = affinity.rotate(line, left_turn_angle, line.coords[0])
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
                                 "file_name": self.files_name,
                                 "obstacles": urban.get("obstacles"),
                                 "success_point": urban.get("success_point"),
                                 "ego_lanes": urban.get("ego_lanes"),
                                 "directions": urban.get("directions"),
                                 "fitness": 0,
                                 "triggers": urban.get("triggers"),
                                 "tod": urban.get("tod"),
                                 "intersection_lanes": urban.get("intersection_lanes")})
                i += 1
        return startpop

    def _mutation(self, individual):
        """Mutates several properties of an individual.
        :param individual: Individual of a population list.
        :return: Mutated child.
        """
        #child = deepcopy(individual)
        print(colored("Mutating individual...", "grey", attrs=['bold']))
        #if random() <= self.mutation_probability:
        #    self._mutate_points(child)
        #if random() <= self.mutation_probability * 2:
        #    self._mutate_num_lanes_and_width(child)
        #child = self._update(child)
        #if random() <= self.mutation_probability:
        #    self._mutate_traffic_signs(child)
        #if random() <= self.mutation_probability:
        #    child = self._mutate_traffic_light_mode(child)
        #if random() <= self.mutation_probability:
        #    self._mutate_parked_cars(child)
        child = {'lanes': [{'control_points': [(1, 0), (30, 0), (45, 0)], 'width': 10.0, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal', 'parked_left': False, 'parked_right': True, 'parked_rotation': 45, 'parked_offset': 3.5, 'parked_max_distance': 4}, {'control_points': [(45, 0), (65.0, 0.0)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.0, 0.0), (69.3577871373829, 49.809734904587266)], 'width': 15, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.0, 0.0), (71.09346717025738, -49.62730758206611)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.0, 0.0), (95.0, 0.0)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(95.0, 0.0), (101, -16), (117, -39), (106, -56), (118, -81), (110, -104), (91, -111)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal', 'parked_left': False, 'parked_right': False, 'parked_rotation': 45, 'parked_offset': 3.5, 'parked_max_distance': 4}, {'control_points': [(91, -111), (72.2331376636578, -117.91410717654713)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(72.2331376636578, -117.91410717654713), (81.96613509823635, -166.9576466179301)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(72.2331376636578, -117.91410717654713), (44.082844159144486, -128.2852679413678)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(44.082844159144486, -128.2852679413678), (44, -154), (34, -167)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal', 'parked_left': True, 'parked_right': True, 'parked_rotation': 0, 'parked_offset': 2, 'parked_max_distance': 5.5}], 'file_name': 'urban', 'obstacles': [{'name': 'stopsign', 'position': (57.4, -5.2), 'zRot': 360, 'intersection_id': 0, 'facingEgo': True, 'lane_id': 1}, {'name': 'prioritysign', 'position': (57.77379511270662, 5.751692179424881), 'zRot': 265, 'intersection_id': 0}, {'name': 'stopsign', 'position': (73.41, 5.2), 'zRot': 180, 'intersection_id': 0}, {'name': 'prioritysign', 'position': (73.39453921644794, -5.185615811407327), 'zRot': 457, 'intersection_id': 0}, {'name': 'stopsign', 'position': (77.56687748556557, -110.40736224201027), 'zRot': 200, 'intersection_id': 1, 'facingEgo': True, 'lane_id': 6}, {'name': 'prioritysign', 'position': (80.77860847595778, -121.41766659464311), 'zRot': 461, 'intersection_id': 1}, {'name': 'stopsign', 'position': (65.65140149638324, -125.8806402383244), 'zRot': 380, 'intersection_id': 1}, {'name': 'golf', 'position': (50.551, -133.814), 'zRot': 268.54437571430185, 'color': (0.23, 0.9, 0.06, 1.23), 'lane': 9}, {'name': 'golf', 'position': (48.293, -150.495), 'zRot': 263.375150604797, 'color': (0.23, 0.9, 0.06, 1.23), 'lane': 9}, {'name': 'golf', 'position': (36.592, -134.566), 'zRot': 454.693253519757, 'color': (0.23, 0.9, 0.06, 1.23), 'lane': 9}, {'name': 'golf', 'position': (33.654, -151.082), 'zRot': 441.12191027006367, 'color': (0.23, 0.9, 0.06, 1.23), 'lane': 9}, {'name': 'golf', 'position': (1.444, -9.3), 'zRot': 130.0, 'color': (0.23, 0.9, 0.06, 1.23), 'lane': 0}, {'name': 'golf', 'position': (48.588, -150.097), 'zRot': 254.5548968032686, 'color': (0.23, 0.9, 0.06, 1.23), 'lane': 9}, {'name': 'golf', 'position': (47.849, -156.22), 'zRot': 255.63818955327997, 'color': (0.23, 0.9, 0.06, 1.23), 'lane': 9, 'init_position': (48.239, -156.04), 'init_zRot': 255.63818955327997}, {'name': 'golf', 'position': (36.773, -129.473), 'zRot': 454.40812250593603, 'color': (0.23, 0.9, 0.06, 1.23), 'lane': 9}, {'name': 'golf', 'position': (34.036, -151.2), 'zRot': 428.996191448273, 'color': (0.23, 0.9, 0.06, 1.23), 'lane': 9, 'init_position': (34.036, -151.2), 'init_zRot': 428.15619144827303}], 'success_point': {'position': (34, -167), 'tolerance': 3}, 'ego_lanes': [0, 1, 4, 5, 6, 8, 9], 'directions': ['straight', 'straight'], 'fitness': 0, 'triggers': [{'triggerPoint': {'position': (45.0, -2.5), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_0'}, 'spawnPoint': {'position': (76.056, -49.018), 'orientation': 0.0}}, {'triggerPoint': {'position': (90.13573660293162, -108.65414220795722), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_0'}, 'spawnPoint': {'position': (73.097, -120.26), 'orientation': 0.0}}, {'triggerPoint': {'position': (1.0, 2.5), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_2'}, 'spawnPoint': {'position': (41.0, 2.5), 'orientation': 180.0}}, {'triggerPoint': {'position': (97.341, 0.878), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_2'}, 'spawnPoint': {'position': (111.425, -106.051), 'orientation': 475.42528439344665}}, {'triggerPoint': {'position': (46.583, -128.293), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_2'}, 'spawnPoint': {'position': (45.273, -156.441), 'orientation': 449.81137260077315}}], 'tod': 0.9621780035427762, 'intersection_lanes': [[1, 2, 3, 4], [6, 7, 8]], 'participants': [{'id': 'ego', 'init_state': {'position': (1.0, -2.5), 'orientation': 0}, 'waypoints': [{'position': (1.0, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (2.571, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (4.143, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (5.714, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (7.286, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (8.857, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (10.429, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (12.0, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (13.571, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (15.143, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (16.714, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (18.286, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (19.857, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (21.429, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (23.0, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (24.571, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (26.143, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (27.714, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (29.286, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (30.857, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (32.429, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (34.0, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (35.571, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (37.143, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (38.714, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (40.286, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (41.857, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (43.429, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (45.0, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (46.786, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (48.571, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (50.357, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (52.143, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (55.0, -2.5), 'tolerance': 2, 'lane': 1}, {'position': (56.786, -2.5), 'tolerance': 2, 'lane': 1}, {'position': (58.571, -2.5), 'tolerance': 2, 'lane': 1}, {'position': (60.357, -2.5), 'tolerance': 2, 'lane': 1}, {'position': (62.143, -2.5), 'tolerance': 2, 'lane': 1}, {'position': (69.286, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (70.893, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (72.5, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (74.107, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (75.714, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (77.321, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (78.929, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (80.536, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (82.143, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (83.75, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (85.357, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (86.964, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (88.571, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (90.179, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (91.786, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (93.393, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (92.659, -0.878), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (95.901, -9.52), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (97.911, -14.876), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (98.689, -16.946), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (101.094, -20.512), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (104.472, -25.369), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (108.754, -32.004), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (110.848, -38.297), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (110.346, -43.966), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (107.606, -48.917), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (105.335, -52.428), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (104.093, -54.352), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (103.512, -55.787), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (104.791, -59.254), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (106.993, -63.844), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (110.3, -70.828), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (112.609, -78.33), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (112.973, -85.504), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (111.386, -92.349), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (107.248, -98.583), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (100.164, -104.018), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (90.136, -108.654), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (88.46, -109.271), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (86.785, -109.889), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (85.109, -110.506), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (83.433, -111.123), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (80.752, -112.111), 'tolerance': 2, 'lane': 2}, {'position': (79.077, -112.729), 'tolerance': 2, 'lane': 2}, {'position': (77.401, -113.346), 'tolerance': 2, 'lane': 2}, {'position': (75.725, -113.963), 'tolerance': 2, 'lane': 2}, {'position': (74.05, -114.581), 'tolerance': 2, 'lane': 2}, {'position': (67.347, -117.05), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (65.839, -117.605), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (64.331, -118.161), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (62.823, -118.717), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (61.315, -119.272), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (59.807, -119.828), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (58.299, -120.383), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (56.791, -120.939), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (55.283, -121.495), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (53.775, -122.05), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (52.267, -122.606), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (50.759, -123.161), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (49.251, -123.717), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (47.743, -124.273), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (46.235, -124.828), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (44.727, -125.384), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (41.583, -128.277), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (41.565, -130.038), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (41.523, -131.766), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (41.458, -133.462), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (41.368, -135.127), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (41.254, -136.759), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (41.117, -138.36), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (40.955, -139.928), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (40.769, -141.464), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (40.56, -142.969), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (40.2, -145.166), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (39.787, -147.29), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (39.319, -149.343), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (38.798, -151.324), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (38.223, -153.233), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (37.593, -155.069), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (36.91, -156.834), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (36.173, -158.527), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (35.382, -160.148), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (34.536, -161.697), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (33.637, -163.174), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}], 'model': 'ETK800', 'color': 'White'}, {'id': 'other_0', 'init_state': {'position': (76.056, -49.018), 'orientation': 0.0}, 'waypoints': [{'position': (76.056, -49.018), 'tolerance': 2, 'lane': 3}, {'position': (75.779, -46.762), 'tolerance': 2, 'lane': 3}, {'position': (75.502, -44.506), 'tolerance': 2, 'lane': 3}, {'position': (75.225, -42.251), 'tolerance': 2, 'lane': 3}, {'position': (74.948, -39.995), 'tolerance': 2, 'lane': 3}, {'position': (74.671, -37.739), 'tolerance': 2, 'lane': 3}, {'position': (74.394, -35.483), 'tolerance': 2, 'lane': 3}, {'position': (74.117, -33.227), 'tolerance': 2, 'lane': 3}, {'position': (73.84, -30.972), 'tolerance': 2, 'lane': 3}, {'position': (73.563, -28.716), 'tolerance': 2, 'lane': 3}, {'position': (73.286, -26.46), 'tolerance': 2, 'lane': 3}, {'position': (73.009, -24.204), 'tolerance': 2, 'lane': 3}, {'position': (72.732, -21.949), 'tolerance': 2, 'lane': 3}, {'position': (72.456, -19.693), 'tolerance': 2, 'lane': 3}, {'position': (72.179, -17.437), 'tolerance': 2, 'lane': 3}, {'position': (71.902, -15.181), 'tolerance': 2, 'lane': 3}, {'position': (71.625, -12.925), 'tolerance': 2, 'lane': 3}, {'position': (71.348, -10.67), 'tolerance': 2, 'lane': 3}, {'position': (63.182, 2.5), 'tolerance': 2, 'lane': 3}, {'position': (61.364, 2.5), 'tolerance': 2, 'lane': 3}, {'position': (59.545, 2.5), 'tolerance': 2, 'lane': 3}, {'position': (57.727, 2.5), 'tolerance': 2, 'lane': 3}, {'position': (55.909, 2.5), 'tolerance': 2, 'lane': 3}, {'position': (54.091, 2.5), 'tolerance': 2, 'lane': 3}, {'position': (52.273, 2.5), 'tolerance': 2, 'lane': 3}, {'position': (50.455, 2.5), 'tolerance': 2, 'lane': 3}, {'position': (48.636, 2.5), 'tolerance': 2, 'lane': 3}, {'position': (46.818, 2.5), 'tolerance': 2, 'lane': 3}, {'position': (45.0, 2.5), 'tolerance': 2, 'lane': 3}, {'position': (73.097, -120.26), 'tolerance': 2, 'lane': 6}, {'position': (74.803, -119.631), 'tolerance': 2, 'lane': 6}, {'position': (76.51, -119.003), 'tolerance': 2, 'lane': 6}, {'position': (78.216, -118.374), 'tolerance': 2, 'lane': 6}, {'position': (79.922, -117.746), 'tolerance': 2, 'lane': 6}, {'position': (81.628, -117.117), 'tolerance': 2, 'lane': 6}, {'position': (83.334, -116.489), 'tolerance': 2, 'lane': 6}, {'position': (85.04, -115.86), 'tolerance': 2, 'lane': 6}, {'position': (86.746, -115.232), 'tolerance': 2, 'lane': 6}, {'position': (68.214, -123.346), 'tolerance': 2, 'lane': 6}, {'position': (68.656, -125.575), 'tolerance': 2, 'lane': 6}, {'position': (69.098, -127.804), 'tolerance': 2, 'lane': 6}, {'position': (69.541, -130.034), 'tolerance': 2, 'lane': 6}, {'position': (69.983, -132.263), 'tolerance': 2, 'lane': 6}, {'position': (70.426, -134.492), 'tolerance': 2, 'lane': 6}, {'position': (70.868, -136.721), 'tolerance': 2, 'lane': 6}, {'position': (71.31, -138.951), 'tolerance': 2, 'lane': 6}, {'position': (71.753, -141.18), 'tolerance': 2, 'lane': 6}, {'position': (72.195, -143.409), 'tolerance': 2, 'lane': 6}, {'position': (72.638, -145.638), 'tolerance': 2, 'lane': 6}, {'position': (73.08, -147.868), 'tolerance': 2, 'lane': 6}, {'position': (73.523, -150.097), 'tolerance': 2, 'lane': 6}, {'position': (73.965, -152.326), 'tolerance': 2, 'lane': 6}, {'position': (74.407, -154.555), 'tolerance': 2, 'lane': 6}, {'position': (74.85, -156.785), 'tolerance': 2, 'lane': 6}, {'position': (75.292, -159.014), 'tolerance': 2, 'lane': 6}, {'position': (75.735, -161.243), 'tolerance': 2, 'lane': 6}, {'position': (76.177, -163.472), 'tolerance': 2, 'lane': 6}, {'position': (76.619, -165.702), 'tolerance': 2, 'lane': 6}, {'position': (77.062, -167.931), 'tolerance': 2, 'lane': 6}], 'model': 'ETK800', 'color': 'Orange'}, {'id': 'other_1', 'init_state': {'position': (4.143, -2.5), 'orientation': 0}, 'waypoints': [{'position': (5.714, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (7.286, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (8.857, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (10.429, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (12.0, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (13.571, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (15.143, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (16.714, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (18.286, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (19.857, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (21.429, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (23.0, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (24.571, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (26.143, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (27.714, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (29.286, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (30.857, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (32.429, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (34.0, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (35.571, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (37.143, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (38.714, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (40.286, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (41.857, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (43.429, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (45.0, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (46.786, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (48.571, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (50.357, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (52.143, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (55.0, -2.5), 'tolerance': 2, 'lane': 1}, {'position': (56.786, -2.5), 'tolerance': 2, 'lane': 1}, {'position': (58.571, -2.5), 'tolerance': 2, 'lane': 1}, {'position': (60.357, -2.5), 'tolerance': 2, 'lane': 1}, {'position': (62.143, -2.5), 'tolerance': 2, 'lane': 1}, {'position': (69.286, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (70.893, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (72.5, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (74.107, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (75.714, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (77.321, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (78.929, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (80.536, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (82.143, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (83.75, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (85.357, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (86.964, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (88.571, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (90.179, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (91.786, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (93.393, -2.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (92.659, -0.878), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (95.901, -9.52), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (97.911, -14.876), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (98.689, -16.946), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (101.094, -20.512), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (104.472, -25.369), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (108.754, -32.004), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (110.848, -38.297), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (110.346, -43.966), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (107.606, -48.917), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (105.335, -52.428), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (104.093, -54.352), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (103.512, -55.787), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (104.791, -59.254), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (106.993, -63.844), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (110.3, -70.828), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (112.609, -78.33), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (112.973, -85.504), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (111.386, -92.349), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (107.248, -98.583), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (100.164, -104.018), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (90.136, -108.654), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (88.46, -109.271), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (86.785, -109.889), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (85.109, -110.506), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (83.433, -111.123), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (60.591, -5.121), 'tolerance': 2, 'lane': 1}, {'position': (60.868, -7.377), 'tolerance': 2, 'lane': 1}, {'position': (61.145, -9.632), 'tolerance': 2, 'lane': 1}, {'position': (61.422, -11.888), 'tolerance': 2, 'lane': 1}, {'position': (61.699, -14.144), 'tolerance': 2, 'lane': 1}, {'position': (61.976, -16.4), 'tolerance': 2, 'lane': 1}, {'position': (62.253, -18.656), 'tolerance': 2, 'lane': 1}, {'position': (62.53, -20.911), 'tolerance': 2, 'lane': 1}, {'position': (62.807, -23.167), 'tolerance': 2, 'lane': 1}, {'position': (63.084, -25.423), 'tolerance': 2, 'lane': 1}, {'position': (63.361, -27.679), 'tolerance': 2, 'lane': 1}, {'position': (63.638, -29.935), 'tolerance': 2, 'lane': 1}, {'position': (63.915, -32.19), 'tolerance': 2, 'lane': 1}, {'position': (64.192, -34.446), 'tolerance': 2, 'lane': 1}, {'position': (64.469, -36.702), 'tolerance': 2, 'lane': 1}, {'position': (64.746, -38.958), 'tolerance': 2, 'lane': 1}, {'position': (65.023, -41.214), 'tolerance': 2, 'lane': 1}, {'position': (65.3, -43.469), 'tolerance': 2, 'lane': 1}, {'position': (65.577, -45.725), 'tolerance': 2, 'lane': 1}, {'position': (65.854, -47.981), 'tolerance': 2, 'lane': 1}, {'position': (66.131, -50.237), 'tolerance': 2, 'lane': 1}], 'model': 'ETK800', 'color': 'Green'}, {'id': 'other_2', 'init_state': {'position': (41.0, 2.5), 'orientation': 180.0}, 'waypoints': [{'position': (41.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (39.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (37.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (35.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (33.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (31.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (29.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (27.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (25.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (23.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (21.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (19.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (17.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (15.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (13.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (11.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (9.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (7.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (5.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (3.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (1.0, 2.5), 'tolerance': 2, 'lane': 0}, {'position': (111.425, -106.051), 'tolerance': 2, 'lane': 5}, {'position': (112.634, -104.026), 'tolerance': 2, 'lane': 5}, {'position': (116.717, -92.299), 'tolerance': 2, 'lane': 5}, {'position': (120.252, -82.123), 'tolerance': 2, 'lane': 5}, {'position': (120.469, -80.628), 'tolerance': 2, 'lane': 5}, {'position': (118.569, -76.41), 'tolerance': 2, 'lane': 5}, {'position': (112.027, -60.898), 'tolerance': 2, 'lane': 5}, {'position': (114.869, -46.896), 'tolerance': 2, 'lane': 5}, {'position': (119.05, -40.426), 'tolerance': 2, 'lane': 5}, {'position': (119.491, -38.824), 'tolerance': 2, 'lane': 5}, {'position': (117.437, -35.254), 'tolerance': 2, 'lane': 5}, {'position': (107.65, -20.808), 'tolerance': 2, 'lane': 5}, {'position': (97.341, 0.878), 'tolerance': 2, 'lane': 5}, {'position': (45.273, -156.441), 'tolerance': 2, 'lane': 9}, {'position': (46.172, -155.232), 'tolerance': 2, 'lane': 9}, {'position': (46.503, -152.157), 'tolerance': 2, 'lane': 9}, {'position': (46.515, -148.73), 'tolerance': 2, 'lane': 9}, {'position': (46.533, -143.61), 'tolerance': 2, 'lane': 9}, {'position': (46.555, -136.798), 'tolerance': 2, 'lane': 9}, {'position': (46.583, -128.293), 'tolerance': 2, 'lane': 9}], 'model': 'ETK800', 'color': 'Blue'}], 'parked_color': (0.23, 0.9, 0.06, 1.23)}
        if random() <= self.mutation_probability:
            self._mutate_traffic(child)  # TODO Traffic
        if random() <= self.mutation_probability:
            child["tod"] = random()
        child["fitness"] = 0
        return child

    def _mutate_points(self, individual):
        """Mutates the control points of each lane.
        :param individual: Individual with lanes.
        :return: Void.
        """
        lanes = individual.get("lanes")
        i = 0
        penul_point = None
        temp_set = set()
        for lane in lanes:
            for point in lane.get("control_points"):
                temp_set.add(point)
        probability = 1 / (len(temp_set) - 2 - len(individual.get("directions")))
        while i < len(lanes):
            control_points = lanes[i].get("control_points")
            mutated = False
            if lanes[i].get("type") == "normal":
                penul_point = control_points[-1]
                j = 2 if i == 0 else 1
                while j < len(control_points):
                    if random() <= probability:
                        valid = False
                        tries = 0
                        while not valid and tries < self.MAX_TRIES:
                            penultimate_point = lanes[i - 1].get("control_points")[-2] if j == 1 \
                                else control_points[j - 2]
                            last_point = control_points[j - 1]
                            new_point = self._add_segment(last_point, penultimate_point)
                            temp_list = deepcopy(lanes)
                            temp_list[i]["control_points"][j] = new_point
                            if j == len(control_points) - 1 and i != len(lanes) - 1:
                                temp_list[i + 1]["control_points"][0] = new_point
                            temp_list = self._bspline(temp_list)
                            control_points_lines = convert_points_to_lines(temp_list)
                            width_lines = get_width_lines(temp_list)
                            if i == len(lanes) - 1 and j == len(control_points) - 1:
                                next_point = None
                            elif j == len(control_points) - 1:
                                next_point = lanes[i + 1].get("control_points")[1]
                            else:
                                next_point = lanes[i].get("control_points")[j + 1]
                            deg = 180 if next_point is None else get_angle(last_point, new_point, next_point)
                            dist = self.MAX_SEGMENT_LENGTH if next_point is None \
                                else Point(next_point).distance(Point(new_point))
                            if not intersection_check_all(control_points_lines) and MIN_DEGREES <= deg <= MAX_DEGREES \
                                    and self.MAX_SEGMENT_LENGTH >= dist >= self.MIN_SEGMENT_LENGTH \
                                    and not intersection_check_width(width_lines, control_points_lines,
                                                                     individual.get("intersection_lanes")):
                                valid = True
                                mutated = True
                                lanes[i]["control_points"][j] = new_point
                                if j == len(control_points) - 1 and i != len(lanes) - 1:
                                    lanes[i + 1]["control_points"][0] = new_point
                            tries += 1
                    j += 1
            else:
                if lanes[i - 1].get("type") == "normal":
                    lanes[i]["mutated"] = mutated
                    i += 1
                    continue
                if random() <= probability:
                    valid = False
                    tries = 0
                    while not valid and tries < self.MAX_TRIES:
                        deg = get_angle(penul_point, control_points[0], control_points[1])
                        if 45 <= deg <= 135:
                            new_point = self._create_intersection(control_points[0], penul_point).get("right_point")
                        elif 225 <= deg <= 315:
                            new_point = self._create_intersection(control_points[0], penul_point).get("left_point")
                        else:
                            break
                        temp_list = deepcopy(lanes)
                        temp_list[i]["control_points"][1] = new_point
                        if lanes[i + 1].get("type") == "normal":
                            temp_list[i + 1]["control_points"][0] = new_point
                        temp_list = self._bspline(temp_list)
                        control_points_lines = convert_points_to_lines(temp_list)
                        width_lines = get_width_lines(temp_list)
                        if not intersection_check_all(control_points_lines) \
                                and not intersection_check_width(width_lines, control_points_lines,
                                                                 individual.get("intersection_lanes")):
                            valid = True
                            mutated = True
                            lanes[i]["control_points"][1] = new_point
                            if lanes[i + 1].get("type") == "normal":
                                lanes[i + 1]["control_points"][0] = new_point
                        tries += 1
            lanes[i]["mutated"] = mutated
            i += 1

    def _mutate_num_lanes_and_width(self, individual):
        """Mutates the width and number of lanes for each lane.
        :param individual: Individual with lanes.
        :return: Void.
        """
        lanes = individual.get("lanes")
        temp_lanes = deepcopy(lanes)
        connected_lanes = _get_connected_lanes(lanes, individual.get("ego_lanes"), individual.get("directions"))
        probability = 1 / len(connected_lanes)
        for connection in connected_lanes:
            if random() <= probability:
                old_width = individual.get("lanes")[connection[0]].get("width")
                old_left_lanes = individual.get("lanes")[connection[0]].get("left_lanes")
                old_right_lanes = individual.get("lanes")[connection[0]].get("right_lanes")
                if random() <= 0.5:
                    left_lanes = randint(1, 2)
                    right_lanes = randint(1, 2)
                else:
                    left_lanes = old_left_lanes
                    right_lanes = old_right_lanes
                width_per_lane = randint(4, 5) if random() <= 0.5 else old_width / (old_left_lanes + old_right_lanes)
                width = width_per_lane * (left_lanes + right_lanes)
                for idx, lane in enumerate(connection):
                    temp_lanes[lane]["width"] = width
                    if idx != 0 and connection[idx] - connection[idx - 1] < 0:
                        temp_lanes[lane]["right_lanes"] = left_lanes
                        temp_lanes[lane]["left_lanes"] = right_lanes
                    else:
                        temp_lanes[lane]["left_lanes"] = left_lanes
                        temp_lanes[lane]["right_lanes"] = right_lanes
                temp_list = self._bspline(temp_lanes)
                control_points_lines = convert_points_to_lines(temp_list)
                width_lines = get_width_lines(temp_list)
                if not intersection_check_width(width_lines, control_points_lines, individual["intersection_lanes"]):
                    individual["lanes"] = temp_lanes

    def _mutate_traffic_signs(self, individual):
        """Mutates the traffic signs and lights of an individual. Mutates position, rotation and the kind of obstacle.
        :param individual: Individual of the population list.
        :return: Void.
        """
        temp_list = list()
        for obstacle in individual.get("obstacles"):
            if obstacle.get("name").startswith("traffic") or obstacle.get("name").endswith("sign"):
                temp_list.append(obstacle)
        probability = 1 / len(temp_list)
        probability_attr = 1 / 3  # Because I mutate kind of obstacle, zRot and position.
        last_id = -1
        for obstacle in individual.get("obstacles"):
            if random() <= probability:
                old_position = obstacle.get("position")
                old_rotation = obstacle.get("zRot")
                if obstacle.get("init_position") is None:
                    obstacle["init_position"] = old_position
                if obstacle.get("init_zRot") is None:
                    obstacle["init_zRot"] = old_rotation
                if random() <= probability_attr or last_id == obstacle.get("intersection_id"):
                    if last_id != obstacle.get("intersection_id"):
                        last_id = obstacle.get("intersection_id")
                    if obstacle.get("name") == "trafficlightsingle":
                        obstacle["name"] = "stopsign"
                        mode = obstacle.get("mode")
                        del obstacle["mode"]
                        del obstacle["sign"]
                        if mode == "manual":
                            oid = obstacle.get("oid")
                            del obstacle["oid"]
                            for idx, trigger in enumerate(individual.get("triggers")):
                                if trigger.get("triggerPoint").get("triggers") == oid:
                                    del individual["triggers"][idx]
                                    break
                    elif obstacle.get("name").endswith("sign"):
                        if obstacle.get("name") == "priority":
                            obstacle["sign"] = "priority"
                        else:
                            obstacle["sign"] = "yield"
                        obstacle["name"] = "trafficlightsingle"
                        if obstacle.get("facingEgo") is not None:
                            obstacle["mode"] = choice(["off", "blinking", "manual"])
                        else:
                            obstacle["mode"] = choice(["off", "blinking"])
                        if obstacle["mode"] == "manual":
                            global OID_INDEX
                            oid = "traffic_light_manual_" + str(OID_INDEX)
                            OID_INDEX += 1
                            last_point = individual.get("lanes")[obstacle.get("lane_id")].get("control_points")[0]
                            triggers = _handle_manual_mode(last_point, oid)
                            individual["triggers"].extend(triggers)
                if random() <= probability_attr:
                    tries = 0
                    while tries < self.MAX_TRIES:
                        new_position = (old_position[0] + round(uniform(-1, 1), 2),
                                        old_position[1] + round(uniform(-1, 1), 2))
                        if euclidean(new_position, obstacle.get("init_position")) <= 0.8:
                            obstacle["position"] = new_position
                            break
                        tries += 1
                if random() <= probability_attr:
                    tries = 0
                    while tries < self.MAX_TRIES:
                        new_rotation = old_rotation + round(uniform(-1, 1), 2)
                        if abs(new_rotation - obstacle.get("init_zRot")) < 3:
                            obstacle["zRot"] = new_rotation
                            break
                        tries += 1

    @staticmethod
    def _mutate_traffic_light_mode(individual):
        """Mutates the traffic light mode of traffic light for each intersection.
        :param individual: Individual of the population list.
        :return: Unmutated individual if something failed, else Void.
        """
        intersecs_light = set()
        for obstacle in individual.get("obstacles"):
            if obstacle.get("name").startswith("trafficlight"):
                intersecs_light.add(obstacle.get("intersection_id"))
        if len(intersecs_light) == 0:
            return individual
        probability = 1 / len(intersecs_light)
        mutation_list = list()
        for intersection_id in intersecs_light:
            if random() <= probability:
                mutation_list.append(intersection_id)
        if len(mutation_list) == 0:
            return individual
        mode = None
        intersec_id = mutation_list[0]
        for obstacle in individual.get("obstacles"):
            if not obstacle.get("name").startswith("trafficlight"):
                continue
            if obstacle.get("intersection_id") == intersec_id:
                obstacle["mode"] = mode
            else:
                intersec_id = obstacle.get("intersection_id")
                mode = choice(["off", "blinking", "manual"])
                obstacle["mode"] = mode
                if mode == "manual":
                    mode = "off"
                    global OID_INDEX
                    oid = "traffic_light_manual_" + str(OID_INDEX)
                    OID_INDEX += 1
                    last_point = individual.get("lanes")[obstacle.get("lane_id")].get("control_points")[0]
                    triggers = _handle_manual_mode(last_point, oid)
                    individual["triggers"].extend(triggers)

    def _mutate_parked_cars(self, individual):
        """Mutates parked cars either car by car (position and rotation) or for a whole lane (rotation angle). Also
           mutates color.
        :param individual: Individual to mutate of the population list.
        :return: Void.
        """
        num_cars = 0
        num_lanes = set()
        for obstacle in individual.get("obstacles"):
            if obstacle.get("name") == "golf":
                num_cars += 1
                num_lanes.add(obstacle.get("lane"))
        if num_cars == 0 or len(num_lanes) == 0:
            return individual
        probability = 1 / num_cars
        probability_attr = 1 / 2  # Position and rotation are mutatible.
        probability_lane = 1 / len(num_lanes)  # Probability to mutate the whole lane because the angle changes.
        for lane_index in num_lanes:
            if random() <= probability_lane:
                for obstacle in individual.get("obstacles"):
                    if obstacle.get("name") == "golf" and obstacle.get("lane") == lane_index:
                        individual["obstacles"].remove(obstacle)
                car_positions = list()
                lane = individual.get("lanes")[lane_index]
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
                prev_lane = LineString(individual.get("lanes")[lane_index - 1].get("control_points")) \
                    if lane_index != 0 else None
                prev_width = int(individual.get("lanes")[lane_index - 1].get("width")) / 2 + offset \
                    if lane_index != 0 else 0
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
                                                       (prev_lane is None or Point(point).distance(
                                                           prev_lane) > prev_width)):
                            angle = get_angle((coords[iterator - 1][0] + 5, coords[iterator - 1][1]),
                                              coords[iterator - 1], point) - rotation + randint(-8, 8)
                            car_positions.append((point, angle, lane_index))
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
                                                       (prev_lane is None or Point(point).distance(
                                                           prev_lane) > prev_width)):
                            angle = get_angle((coords[iterator - 1][0] + 5, coords[iterator - 1][1]),
                                              coords[iterator - 1], point) + 180 - rotation + randint(-8, 8)
                            car_positions.append((point, angle, lane_index))
                        iterator += 1
                lane["parked_left"] = left
                lane["parked_right"] = right
                lane["parked_rotation"] = rotation
                lane["parked_offset"] = offset
                lane["parked_max_distance"] = max_distance
                individual["lanes"][lane_index] = lane
                parked_cars = list()
                for position in car_positions:
                    if random() <= 0.4:
                        continue
                    parked_cars.append(
                        {"name": "golf", "position": (position[0][0], position[0][1]), "zRot": position[1],
                         "color": None, "lane": position[2]})
                individual["obstacles"].extend(parked_cars)
            else:
                for idx, obstacle in enumerate(individual.get("obstacles")):
                    if obstacle.get("name") == "golf" and obstacle.get("lane") == lane_index and random() <= probability:
                        if random() <= probability_attr:
                            old_position = obstacle.get("position")
                            max_distance = individual.get("lanes")[obstacle.get("lane")].get("parked_max_distance")
                            tries = 0
                            while tries < self.MAX_TRIES:
                                new_position = (old_position[0] + round(uniform(-1, 1), 2),
                                                old_position[1] + round(uniform(-1, 1), 2))
                                if idx == 0:
                                    dist1 = 10
                                else:
                                    dist1 = euclidean(new_position,
                                                      individual.get("obstacles")[idx - 1].get("position"))
                                if idx == len(individual.get("obstacles")) - 1:
                                    dist2 = 10
                                else:
                                    dist2 = euclidean(new_position,
                                                      individual.get("obstacles")[idx + 1].get("position"))
                                if dist1 >= max_distance and dist2 >= max_distance:
                                    obstacle["position"] = new_position
                                    break
                                tries += 1
                        if random() <= probability_attr:
                            old_rotation = obstacle.get("zRot")
                            max_rotation_diff = 5
                            tries = 0
                            while tries < self.MAX_TRIES:
                                new_rotation = old_rotation + round(uniform(-1, 1), 2)
                                if abs(new_rotation - old_rotation) < max_rotation_diff:
                                    obstacle["zRot"] = new_rotation
                                    break
                                tries += 1
        if random() <= 0.5:
            color = (round(uniform(0, 1), 2), round(uniform(0, 1), 2), round(uniform(0, 1), 2),
                     round(uniform(1, 1.3), 2))
            individual["parked_color"] = color
            for obstacle in individual.get("obstacles"):
                if obstacle.get("name") == "golf":
                    obstacle["color"] = color

    def _mutate_traffic(self, individual):
        pass

    def _update(self, individual):
        """Updates obstacle positions, waypoints, trigger and spawn points, success point and initial state of vehicles
           after mutating the control points.
        :param individual: Individual to update.
        :return: Void.
        """
        # Update traffic signs and lights position.
        intersections = _create_intersections_manual(individual)
        lane_ids = list()
        ego_lanes = individual.get("ego_lanes")
        for idx, lane in enumerate(ego_lanes):
            if idx != len(ego_lanes) - 1 and ego_lanes[idx + 1] - ego_lanes[idx] != 1:
                lane_ids.append(lane)
        i = 0
        for idx, intersection in enumerate(intersections):
            obs, tgr = _add_traffic_signs(intersection.get("last_point"), intersection.get("current_left_lanes"),
                                          intersection.get("current_right_lanes"), intersection.get("current_width"),
                                          intersection, lane_ids[idx])
            for obstacle in obs:
                individual["obstacles"][i]["position"] = obstacle.get("position")
                i += 1
            individual["triggers"] = tgr

        # Update success point.
        individual["success_point"] = {"position": individual.get("lanes")[-1].get("control_points")[-1],
                                       "tolerance": 3}

        # Update initial states of vehicles.
        _add_ego_car(individual)
        _add_other_participants(individual)

        # Update parked cars.
        self._add_parked_cars_mutated(individual)
        return individual

    def _add_parked_cars_mutated(self, individual):
        car_positions = list()
        lane_indices = list()
        temp = self._spline_population([deepcopy(individual)])
        lanes = temp[0].get("lanes")
        for idx, lane in enumerate(lanes):
            control_points = lane.get("control_points")
            if lane.get("parked_left") is None or lane.get("mutated") or lane.get("type") == "intersection" \
                    or control_points[0] == control_points[-1]:
                continue
            lane_indices.append(idx)
            width = lane.get("width")
            noise = [x / 10 for x in range(-10, 10)]
            line = LineString(control_points)
            prev_lane = LineString(lanes[idx - 1].get("control_points")) if idx != 0 else None
            prev_width = int(lanes[idx - 1].get("width")) / 2 + lane.get("parked_offset") if idx != 0 else 0
            if lane.get("parked_left"):
                left_lines = [line.parallel_offset(width / 2 + lane.get("parked_offset") + x, "left") for x in noise]
                left_lines = [multilinestrings_to_linestring(x) for x in left_lines]
                i = 1
                while i < len(left_lines[0].coords):
                    left_line = choice(left_lines)
                    coords = b_spline(left_line.coords)
                    point = coords[i]
                    if abs(euclidean(point, coords[-1])) < 12:
                        break
                    if len(car_positions) == 0 or (abs(euclidean(point, car_positions[-1][0]))
                                                   > lane.get("parked_max_distance") and
                                                   (prev_lane is None or Point(point).distance(
                                                       prev_lane) > prev_width)):
                        angle = get_angle((coords[i - 1][0] + 5, coords[i - 1][1]),
                                          coords[i - 1], point) - lane.get("parked_rotation") + randint(-8, 8)
                        car_positions.append((point, angle, idx))
                    i += 1
            if lane.get("parked_right"):
                right_lines = [line.parallel_offset(width / 2 + lane.get("parked_offset") + x, "right") for x in noise]
                right_lines = [multilinestrings_to_linestring(x) for x in right_lines]
                i = 1
                while i < len(right_lines[0].coords):
                    right_line = choice(right_lines)
                    coords = right_line.coords[::-1]
                    coords = b_spline(coords)
                    point = coords[i]
                    if abs(euclidean(point, coords[-1])) < 12:
                        break
                    if len(car_positions) == 0 or (abs(euclidean(point, car_positions[-1][0]))
                                                   > lane.get("parked_max_distance") and
                                                   (prev_lane is None
                                                    or Point(point).distance(prev_lane) > prev_width)):
                        angle = get_angle((coords[i - 1][0] + 5, coords[i - 1][1]),
                                          coords[i - 1], point) + 180 - lane.get("parked_rotation") + randint(-8, 8)
                        car_positions.append((point, angle, idx))
                    i += 1
        parked_cars = list()
        color = individual.get("parked_color")
        for idx, obstacle in enumerate(individual["obstacles"]):
            if obstacle.get("lane") is not None and obstacle.get("lane") in lane_indices:
                del individual["obstacles"][idx]
        for position in car_positions:
            if random() <= 0.4:
                continue
            parked_cars.append({"name": "golf", "position": (position[0][0], position[0][1]), "zRot": position[1],
                                "color": color, "lane": position[2]})
        individual["obstacles"].extend(parked_cars)

    def _choose_elite(self, population):
        """Chooses the test cases with the best fitness values.
        :param population: List of individuals.
        :return: List of best x individuals according to their fitness value.
        """
        population = sorted(population, key=lambda k: k['fitness'])
        elite = list()
        i = 0
        while i < self.NUMBER_ELITES:
            elite.append(population[i])
            i += 1
        return elite

    def genetic_algorithm(self):

        if len(self.population_list) == 0:
            #self.population_list = self._create_start_population()
            self.population_list = [{'lanes': [{'control_points': [(1, 0), (30, 0), (45, 0)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {'control_points': [(45, 0), (65.0, 0.0)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.0, 0.0), (69.3577871373829, 49.809734904587266)], 'width': 15, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.0, 0.0), (71.09346717025738, -49.62730758206611)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.0, 0.0), (95.0, 0.0)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(95.0, 0.0), (101, -16), (117, -39), (106, -56), (118, -81), (110, -104), (91, -111)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {'control_points': [(91, -111), (72.2331376636578, -117.91410717654713)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(72.2331376636578, -117.91410717654713), (81.96613509823635, -166.9576466179301)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(72.2331376636578, -117.91410717654713), (44.082844159144486, -128.2852679413678)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(44.082844159144486, -128.2852679413678), (44, -154), (34, -167)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}], 'file_name': 'urban', 'obstacles': [{'name': 'stopsign', 'position': (57.4, -5.2), 'zRot': 360, 'intersection_id': 0, 'facingEgo': True, 'lane_id': 1}, {'name': 'prioritysign', 'position': (57.77379511270662, 5.751692179424881), 'zRot': 265, 'intersection_id': 0}, {'name': 'stopsign', 'position': (73.41, 5.2), 'zRot': 180, 'intersection_id': 0}, {'name': 'prioritysign', 'position': (73.39453921644794, -5.185615811407327), 'zRot': 457, 'intersection_id': 0}, {'name': 'stopsign', 'position': (78.20495080500122, -110.17228259800765), 'zRot': 200, 'intersection_id': 1, 'facingEgo': True, 'lane_id': 6}, {'name': 'prioritysign', 'position': (80.77860847595778, -121.41766659464311), 'zRot': 461, 'intersection_id': 1}, {'name': 'stopsign', 'position': (65.65140149638324, -125.8806402383244), 'zRot': 380, 'intersection_id': 1}], 'success_point': {'position': (34, -167), 'tolerance': 3}, 'ego_lanes': [0, 1, 4, 5, 6, 8, 9], 'directions': ['straight', 'straight'], 'fitness': 0, 'triggers': [], 'tod': 0.9621780035427762, 'intersection_lanes': [[1, 2, 3, 4], [6, 7, 8]]}, {'lanes': [{'control_points': [(1, 0), (30, 0), (45, 0), (60, -19), (53, -46)], 'width': 15, 'left_lanes': 1, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}, {'control_points': [(53, -46), (47.98075682432548, -65.35993796331601)], 'width': 15, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(47.98075682432548, -65.35993796331601), (40.45189206081369, -94.39984490829002)], 'width': 15, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(47.98075682432548, -65.35993796331601), (91.48372769496474, -90.00728113511266)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(47.98075682432548, -65.35993796331601), (-0.419088083964553, -52.811830024129705)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(-0.419088083964553, -52.811830024129705), (-9, -76), (-26, -86)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}, {'control_points': [(-26, -86), (-43.23868430315539, -96.14040253126788)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(-43.23868430315539, -96.14040253126788), (-18.64368036829653, -139.6729855173977)], 'width': 15, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(-43.23868430315539, -96.14040253126788), (-69.09671075788847, -111.35100632816972)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(-69.09671075788847, -111.35100632816972), (-87, -119)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}, {'control_points': [(-87, -119), (-105.39175788375081, -126.85768680627385)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(-105.39175788375081, -126.85768680627385), (-132.979394709377, -138.64421701568466)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(-105.39175788375081, -126.85768680627385), (-79.5396220957177, -169.65566660902965)], 'width': 8, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(-105.39175788375081, -126.85768680627385), (-137.6206900563201, -88.63078618621407)], 'width': 8, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(-137.6206900563201, -88.63078618621407), (-155, -77), (-182, -77)], 'width': 8, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}], 'file_name': 'urban', 'obstacles': [{'name': 'trafficlightdouble', 'position': (42.05804987702955, -57.522748261869936), 'zRot': 255, 'mode': 'blinking', 'sign': 'yield', 'oid': None, 'intersection_id': 2, 'facingEgo': True, 'lane_id': 1}, {'name': 'trafficlightdouble', 'position': (57.64947894996543, -63.71196573746983), 'zRot': 510, 'mode': 'blinking', 'sign': 'priority', 'intersection_id': 2}, {'name': 'trafficlightdouble', 'position': (53.373933616587756, -75.23960111989193), 'zRot': 435, 'intersection_id': 2, 'mode': 'blinking', 'sign': 'yield'}, {'name': 'trafficlightdouble', 'position': (39.06801501380629, -69.45420632518766), 'zRot': 345, 'mode': 'blinking', 'sign': 'priority', 'intersection_id': 2}, {'name': 'trafficlightdouble', 'position': (-37.90077641069598, -85.80733235190593), 'zRot': 210, 'mode': 'manual', 'sign': 'yield', 'oid': 'traffic_light_manual_2', 'intersection_id': 3, 'facingEgo': True, 'lane_id': 6}, {'name': 'trafficlightsingle', 'position': (-33.53407604323862, -97.66374704960742), 'zRot': 479, 'mode': 'off', 'sign': 'priority', 'intersection_id': 3}, {'name': 'trafficlightdouble', 'position': (-46.89582047605714, -105.48478346383123), 'zRot': 390, 'intersection_id': 3, 'mode': 'off', 'sign': 'yield'}, {'name': 'trafficlightdouble', 'position': (-104.0573304275268, -119.54541606702496), 'zRot': 203, 'mode': 'manual', 'sign': 'yield', 'oid': 'traffic_light_manual_3', 'intersection_id': 4, 'facingEgo': True, 'lane_id': 10}, {'name': 'trafficlightsingle', 'position': (-98.02231575526648, -130.9346124512814), 'zRot': 481, 'mode': 'off', 'sign': 'priority', 'intersection_id': 4}, {'name': 'trafficlightdouble', 'position': (-106.7261853399748, -134.16995754552275), 'zRot': 383, 'intersection_id': 4, 'mode': 'off', 'sign': 'yield'}, {'name': 'trafficlightsingle', 'position': (-114.06239864586908, -123.08928014373154), 'zRot': 310, 'mode': 'off', 'sign': 'priority', 'intersection_id': 4}], 'success_point': {'position': (-182, -77), 'tolerance': 3}, 'ego_lanes': [0, 1, 4, 5, 6, 8, 9, 10, 13, 14], 'directions': ['right', 'straight', 'right'], 'fitness': 0, 'triggers': [{'triggerPoint': {'position': (-26, -86), 'action': 'switchLights', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'traffic_light_manual_2', 'initState': 'green', 'switchTo': 'red'}}, {'triggerPoint': {'position': (-87, -119), 'action': 'switchLights', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'traffic_light_manual_3', 'initState': 'green', 'switchTo': 'red'}}], 'tod': 0.2883351637164827, 'intersection_lanes': [[1, 2, 3, 4], [6, 7, 8], [10, 11, 12, 13]]}, {'lanes': [{'control_points': [(1, 0), (30, 0), (45, 0)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {'control_points': [(45, 0), (65.0, 0.0)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.0, 0.0), (64.12737967813584, -49.992384757819565)], 'width': 15, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.0, 0.0), (95.0, 0.0)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(95.0, 0.0), (98, 17), (111, 29), (117, 55), (117, 83), (103, 107), (85, 123)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {'control_points': [(85, 123), (70.0518136263268, 136.28727677659842)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(70.0518136263268, 136.28727677659842), (47.62953406581704, 156.218191941496)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(70.0518136263268, 136.28727677659842), (33.702976218694744, 101.95417293914247)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(70.0518136263268, 136.28727677659842), (108.70706334931185, 168.00118017642708)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(108.70706334931185, 168.00118017642708), (98, 188), (92, 213)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}], 'file_name': 'urban', 'obstacles': [{'name': 'trafficlightsingle', 'position': (57.11, -6.2), 'zRot': 360, 'mode': 'blinking', 'sign': 'yield', 'oid': None, 'intersection_id': 5, 'facingEgo': True, 'lane_id': 1}, {'name': 'trafficlightsingle', 'position': (73.80000000000001, 6.2), 'zRot': 180, 'intersection_id': 5, 'mode': 'blinking', 'sign': 'yield'}, {'name': 'trafficlightsingle', 'position': (72.59236757343677, -6.233454470021069), 'zRot': 449, 'mode': 'blinking', 'sign': 'priority', 'intersection_id': 5}, {'name': 'trafficlightsingle', 'position': (81.21893930225923, 134.65626355227096), 'zRot': 498, 'mode': 'manual', 'sign': 'yield', 'oid': 'traffic_light_manual_4', 'intersection_id': 6, 'facingEgo': True, 'lane_id': 5}, {'name': 'trafficlightdouble', 'position': (71.24788449193846, 126.13742877357714), 'zRot': 403, 'mode': 'off', 'sign': 'priority', 'intersection_id': 6}, {'name': 'trafficlightsingle', 'position': (59.27334079610989, 137.57282080473436), 'zRot': 318, 'intersection_id': 6, 'mode': 'off', 'sign': 'yield'}, {'name': 'trafficlightdouble', 'position': (69.5666739349591, 146.49583394594706), 'zRot': 219, 'mode': 'off', 'sign': 'priority', 'intersection_id': 6}], 'success_point': {'position': (92, 213), 'tolerance': 3}, 'ego_lanes': [0, 1, 3, 4, 5, 8, 9], 'directions': ['straight', 'right'], 'fitness': 0, 'triggers': [{'triggerPoint': {'position': (85, 123), 'action': 'switchLights', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'traffic_light_manual_4', 'initState': 'green', 'switchTo': 'red'}}], 'tod': 0.7187562641594675, 'intersection_lanes': [[1, 2, 3], [5, 6, 7, 8]]}]

        while len(self.population_list) < self.POPULATION_SIZE:
            selected_indices = sample(range(0, len(self.population_list)), 2)
            parent1 = self.population_list[selected_indices[0]]
            parent2 = self.population_list[selected_indices[1]]
            child1 = self._mutation(parent1)
            child2 = self._mutation(parent2)
            self.population_list.append(child1)
            self.population_list.append(child2)

        print(colored("Population finished.", "grey", attrs=['bold']))
        temp_list = deepcopy(self.population_list)
        temp_list = self._spline_population(temp_list)
        _preparation(temp_list)
        i = 0
        while i < len(self.population_list):
            self.population_list[i]["obstacles"] = list()
            self.population_list[i]["obstacles"] = temp_list[i].get("obstacles")
            self.population_list[i]["participants"] = list()
            self.population_list[i]["participants"] = temp_list[i].get("participants")
            self.population_list[i]["parked_color"] = temp_list[i].get("parked_color")
            for idx, lane in enumerate(temp_list[i].get("lanes")):
                if lane.get("parked_left") is None:
                    continue
                self.population_list[i].get("lanes")[idx]["parked_left"] = lane.get("parked_left")
                self.population_list[i].get("lanes")[idx]["parked_right"] = lane.get("parked_right")
                self.population_list[i].get("lanes")[idx]["parked_rotation"] = lane.get("parked_rotation")
                self.population_list[i].get("lanes")[idx]["parked_offset"] = lane.get("parked_offset")
                self.population_list[i].get("lanes")[idx]["parked_max_distance"] = lane.get("parked_max_distance")
            i += 1
        temp_list = _merge_lanes(temp_list)
        build_all_xml(temp_list)
        self.population_list.pop()

    def get_test(self):
        """Returns the two first test files starting with "files_name".
        :return: Tuple of the path to the dbe and dbc file.
        """
        destination_path = path.dirname(path.realpath(__file__)) + "\\scenario"
        xml_names = destination_path + "\\" + self.files_name + "*"
        matches = glob(xml_names)
        i = 0
        self.genetic_algorithm()
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
#       TODO Mutation
#       TODO Crossover

# TODO May-have/Improvements:
#       TODO Green light/priority sign for ego -> Also change mutation and trigger points
#       TODO Remove redundant XML information
#       TODO Make all objects collidable
#       TODO Improve speed of car
#       TODO Improve traffic sign positioning
#       TODO Test generator should calc speed of waypoints, not converter
#       TODO Teleporting cars shouldnt be visible to ego(line triggered by ego, teleport by other)
#       TODO Fix traffic orientation
#       TODO Find more/other ways to save fuel by looking at the driving style.
#        (engine brake before intersection&smooth braking, constant dist to car in front of ego)

# TODO Future Work:
#       TODO Roundabouts
#       TODO Add weather presets
#       TODO Converter:
#           TODO Add input checking
#           TODO Implement Sensor deployment
#       TODO Parked cars on the road and adjust waypoints
#       TODO Manual traffic lights for all directions
#       TODO Fix lane markings
#       TODO Fix BNG errors and warnings
#       TODO Parallel offset instead of width lines
#       TODO Improve performance
#       TODO Comments
#       TODO Refactor

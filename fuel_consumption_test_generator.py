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
        parked_cars.append({"name": "golf", "position": position[0], "zRot": position[1], "color": color,
                            "lane": position[2]})
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
        if i == len(ego_lanes) - 1 or (ego_lanes[i+1] - ego_lanes[i] == 1 and ego_lanes[i] - ego_lanes[i-1] == 1):
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


def _add_traffic_signs(last_point, current_left_lanes, current_right_lanes, width, intersection):
    global OID_INDEX
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
            obstacles.append({"name": "prioritysign", "position": my_position, "zRot": my_z_rot})
        else:
            if new_right_lanes == 1:
                obstacles.append({"name": "trafficlightsingle", "position": my_position, "zRot": my_z_rot,
                                  "mode": my_mode, "sign": "priority"})
            else:
                obstacles.append({"name": "trafficlightdouble", "position": my_position, "zRot": my_z_rot,
                                  "mode": my_mode, "sign": "priority"})

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
            obstacles.append({"name": "stopsign", "position": position, "zRot": z_rot})
        else:
            obstacles.append({"name": "trafficlightsingle", "position": position, "zRot": z_rot, "mode": mode,
                              "sign": "yield", "oid": oid})
    else:
        obstacles.append({"name": "trafficlightdouble", "position": position, "zRot": z_rot, "mode": mode,
                          "sign": "yield", "oid": oid})
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
        obstacles.append({"name": sign_on_my_lane, "position": position, "zRot": z_rot})
        if sign_on_my_lane.startswith("trafficlight"):
            mode = "off" if mode == "manual" else mode
            obstacles[-1]["mode"] = mode
            obstacles[-1]["sign"] = "yield"

    # Right direction.
    if number_of_ways == 4 or direction == "right" or layout == "right":
        opposite_direction(right_point, straight_point)

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
    print(connected_lanes)
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
        self.mutation_probability = 0.25
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
                                                  lanes[lane_index].get("width"), intersection)
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
        child = deepcopy(individual)
        print(colored("Mutating individual...", "grey", attrs=['bold']))
        #self._mutate_points(child)
        #self._mutate_num_lanes_and_width(child)
        #child = self._update(child)
        child = {'lanes': [{'control_points': [[1.0, 0.0], [1.584, 0.0], [2.166, 0.0], [2.745, 0.0], [3.321, 0.0], [3.894, 0.0], [4.464, 0.0], [5.031, 0.0], [5.595, 0.0], [6.157, 0.0], [6.716, 0.0], [7.272, 0.0], [7.825, 0.0], [8.375, 0.0], [8.922, 0.0], [9.466, 0.0], [10.008, 0.0], [10.547, 0.0], [11.083, 0.0], [11.616, 0.0], [12.146, 0.0], [12.673, 0.0], [13.198, 0.0], [13.719, 0.0], [14.238, 0.0], [14.754, 0.0], [15.267, 0.0], [15.777, 0.0], [16.284, 0.0], [16.789, 0.0], [17.29, 0.0], [17.789, 0.0], [18.285, 0.0], [18.778, 0.0], [19.268, 0.0], [19.755, 0.0], [20.24, 0.0], [20.721, 0.0], [21.2, 0.0], [21.676, 0.0], [22.149, 0.0], [22.619, 0.0], [23.086, 0.0], [23.551, 0.0], [24.012, 0.0], [24.471, 0.0], [24.927, 0.0], [25.38, 0.0], [25.83, 0.0], [26.277, 0.0], [26.722, 0.0], [27.163, 0.0], [27.602, 0.0], [28.038, 0.0], [28.471, 0.0], [28.901, 0.0], [29.329, 0.0], [29.753, 0.0], [30.175, 0.0], [30.593, 0.0], [31.009, 0.0], [31.422, 0.0], [31.832, 0.0], [32.24, 0.0], [32.644, 0.0], [33.046, 0.0], [33.444, 0.0], [33.84, 0.0], [34.233, 0.0], [34.624, 0.0], [35.011, 0.0], [35.395, 0.0], [35.777, 0.0], [36.156, 0.0], [36.531, 0.0], [36.904, 0.0], [37.275, 0.0], [37.642, 0.0], [38.006, 0.0], [38.368, 0.0], [38.727, 0.0], [39.083, 0.0], [39.436, 0.0], [39.786, 0.0], [40.133, 0.0], [40.478, 0.0], [40.819, 0.0], [41.158, 0.0], [41.494, 0.0], [41.827, 0.0], [42.157, 0.0], [42.484, 0.0], [42.809, 0.0], [43.13, 0.0], [43.449, 0.0], [43.765, 0.0], [44.078, 0.0], [44.388, 0.0], [44.696, 0.0], [45.0, 0.0]], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 100, 'type': 'normal', 'parked_left': False, 'parked_right': True, 'parked_rotation': 45, 'parked_offset': 3.5, 'parked_max_distance': 4, 'mutated': False}, {'control_points': [[45.0, 0.0], [45.833, 0.0], [46.667, 0.0], [47.5, 0.0], [48.333, 0.0], [49.167, 0.0], [50.0, 0.0], [50.833, 0.0], [51.667, 0.0], [52.5, 0.0], [53.333, 0.0], [54.167, 0.0], [55.0, 0.0], [55.833, 0.0], [56.667, 0.0], [57.5, 0.0], [58.333, 0.0], [59.167, 0.0], [60.0, 0.0], [60.833, 0.0], [61.667, 0.0], [62.5, 0.0], [63.333, 0.0], [64.167, 0.0], [65.0, 0.0]], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [[65.0, 0.0], [66.25, 0.0], [67.5, 0.0], [68.75, 0.0], [70.0, 0.0], [71.25, 0.0], [72.5, 0.0], [73.75, 0.0], [75.0, 0.0], [76.25, 0.0], [77.5, 0.0], [78.75, 0.0], [80.0, 0.0], [81.25, 0.0], [82.5, 0.0], [83.75, 0.0], [85.0, 0.0], [86.25, 0.0], [87.5, 0.0], [88.75, 0.0], [90.0, 0.0], [91.25, 0.0], [92.5, 0.0], [93.75, 0.0], [95.0, 0.0]], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [[65.0, 0.0], [65.574, -2.003], [66.148, -4.005], [66.723, -6.008], [67.297, -8.011], [67.871, -10.013], [68.445, -12.016], [69.02, -14.018], [69.594, -16.021], [70.168, -18.024], [70.742, -20.026], [71.317, -22.029], [71.891, -24.032], [72.465, -26.034], [73.039, -28.037], [73.614, -30.039], [74.188, -32.042], [74.762, -34.045], [75.336, -36.047], [75.911, -38.05], [76.485, -40.053], [77.059, -42.055], [77.633, -44.058], [78.208, -46.06], [78.782, -48.063]], 'width': 15, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [[65.0, 0.0], [65.652, 2.075], [66.304, 4.151], [66.955, 6.226], [67.607, 8.302], [68.259, 10.377], [68.911, 12.452], [69.562, 14.528], [70.214, 16.603], [70.866, 18.679], [71.518, 20.754], [72.169, 22.829], [72.821, 24.905], [73.473, 26.98], [74.125, 29.056], [74.776, 31.131], [75.428, 33.206], [76.08, 35.282], [76.732, 37.357], [77.383, 39.433], [78.035, 41.508], [78.687, 43.584], [79.339, 45.659], [79.99, 47.734], [80.642, 49.81]], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection', 'mutated': True}, {'control_points': [[80.642, 49.81], [83.627, 51.16], [86.439, 52.498], [89.086, 53.822], [91.574, 55.132], [93.912, 56.426], [96.104, 57.703], [98.16, 58.963], [100.086, 60.204], [101.89, 61.426], [103.577, 62.627], [105.156, 63.807], [106.634, 64.965], [108.018, 66.099], [109.314, 67.209], [110.531, 68.293], [111.675, 69.351], [112.753, 70.381], [113.772, 71.384], [114.74, 72.357], [115.664, 73.299], [116.551, 74.211], [117.407, 75.09], [118.241, 75.936], [119.059, 76.748], [119.869, 77.524], [120.674, 78.265], [121.477, 78.971], [122.278, 79.641], [123.076, 80.278], [123.872, 80.88], [124.667, 81.449], [125.46, 81.984], [126.253, 82.488], [127.045, 82.959], [127.837, 83.398], [128.629, 83.807], [129.422, 84.184], [130.215, 84.531], [131.009, 84.849], [131.805, 85.137], [132.603, 85.396], [133.402, 85.627], [134.205, 85.83], [135.009, 86.006], [135.817, 86.154], [136.629, 86.276], [137.444, 86.372], [138.263, 86.442], [139.086, 86.487], [139.915, 86.507], [140.749, 86.504], [141.589, 86.478], [142.436, 86.433], [143.292, 86.368], [144.158, 86.286], [145.034, 86.189], [145.922, 86.079], [146.822, 85.956], [147.736, 85.823], [148.664, 85.68], [149.608, 85.531], [150.569, 85.376], [151.547, 85.218], [152.544, 85.057], [153.561, 84.895], [154.599, 84.735], [155.658, 84.577], [156.74, 84.423], [157.846, 84.276], [158.977, 84.136], [160.133, 84.006], [161.317, 83.887], [162.528, 83.78], [163.769, 83.687], [165.039, 83.611], [166.337, 83.555], [167.662, 83.524], [169.011, 83.525], [170.382, 83.561], [171.773, 83.639], [173.181, 83.764], [174.604, 83.94], [176.041, 84.174], [177.488, 84.47], [178.944, 84.834], [180.407, 85.271], [181.873, 85.786], [183.342, 86.385], [184.811, 87.073], [186.277, 87.855], [187.738, 88.736], [189.193, 89.722], [190.639, 90.818], [192.073, 92.03], [193.494, 93.362], [194.9, 94.819], [196.287, 96.408], [197.655, 98.133], [199.0, 100.0]], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal', 'parked_left': True, 'parked_right': True, 'parked_rotation': 45, 'parked_offset': 3.5, 'parked_max_distance': 4, 'mutated': True}, {'control_points': [[199.0, 100.0], [199.729, 100.817], [200.458, 101.635], [201.187, 102.452], [201.916, 103.27], [202.645, 104.087], [203.373, 104.904], [204.102, 105.722], [204.831, 106.539], [205.56, 107.357], [206.289, 108.174], [207.018, 108.991], [207.747, 109.809], [208.476, 110.626], [209.205, 111.443], [209.934, 112.261], [210.663, 113.078], [211.392, 113.896], [212.12, 114.713], [212.849, 115.53], [213.578, 116.348], [214.307, 117.165], [215.036, 117.983], [215.765, 118.8], [216.494, 119.617]], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [[216.494, 119.617], [217.842, 118.029], [219.191, 116.441], [220.54, 114.854], [221.888, 113.266], [223.237, 111.678], [224.585, 110.09], [225.934, 108.502], [227.283, 106.914], [228.631, 105.326], [229.98, 103.738], [231.328, 102.15], [232.677, 100.562], [234.026, 98.974], [235.374, 97.386], [236.723, 95.798], [238.071, 94.21], [239.42, 92.622], [240.769, 91.034], [242.117, 89.446], [243.466, 87.858], [244.814, 86.27], [246.163, 84.683], [247.512, 83.095], [248.86, 81.507]], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [[216.494, 119.617], [217.275, 120.593], [218.056, 121.57], [218.837, 122.546], [219.617, 123.522], [220.398, 124.498], [221.179, 125.474], [221.96, 126.45], [222.741, 127.426], [223.522, 128.402], [224.303, 129.378], [225.083, 130.354], [225.864, 131.33], [226.645, 132.306], [227.426, 133.283], [228.207, 134.259], [228.988, 135.235], [229.769, 136.211], [230.55, 137.187], [231.33, 138.163], [232.111, 139.139], [232.892, 140.115], [233.673, 141.091], [234.454, 142.067], [235.235, 143.043]], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [[235.235, 143.043], [235.675, 143.043], [236.116, 143.044], [236.558, 143.045], [237.0, 143.048], [237.444, 143.052], [237.889, 143.057], [238.334, 143.063], [238.78, 143.069], [239.227, 143.077], [239.675, 143.086], [240.124, 143.096], [240.573, 143.107], [241.024, 143.119], [241.475, 143.132], [241.927, 143.146], [242.381, 143.161], [242.835, 143.177], [243.289, 143.194], [243.745, 143.213], [244.202, 143.232], [244.659, 143.252], [245.117, 143.273], [245.576, 143.295], [246.036, 143.319], [246.497, 143.343], [246.959, 143.368], [247.422, 143.395], [247.885, 143.422], [248.349, 143.451], [248.815, 143.48], [249.281, 143.511], [249.748, 143.542], [250.215, 143.575], [250.684, 143.608], [251.154, 143.643], [251.624, 143.679], [252.095, 143.715], [252.567, 143.753], [253.04, 143.792], [253.514, 143.832], [253.989, 143.872], [254.464, 143.914], [254.941, 143.957], [255.418, 144.001], [255.896, 144.046], [256.375, 144.092], [256.855, 144.139], [257.336, 144.187], [257.818, 144.236], [258.3, 144.286], [258.783, 144.337], [259.268, 144.389], [259.753, 144.442], [260.239, 144.497], [260.725, 144.552], [261.213, 144.608], [261.702, 144.665], [262.191, 144.724], [262.681, 144.783], [263.172, 144.843], [263.664, 144.905], [264.157, 144.967], [264.651, 145.031], [265.145, 145.095], [265.641, 145.161], [266.137, 145.227], [266.634, 145.295], [267.132, 145.363], [267.631, 145.433], [268.131, 145.503], [268.632, 145.575], [269.133, 145.648], [269.636, 145.722], [270.139, 145.796], [270.643, 145.872], [271.148, 145.949], [271.654, 146.027], [272.16, 146.106], [272.668, 146.186], [273.176, 146.267], [273.685, 146.349], [274.196, 146.432], [274.707, 146.516], [275.218, 146.601], [275.731, 146.687], [276.245, 146.774], [276.759, 146.862], [277.275, 146.951], [277.791, 147.041], [278.308, 147.133], [278.826, 147.225], [279.344, 147.318], [279.864, 147.412], [280.385, 147.508], [280.906, 147.604], [281.428, 147.702], [281.951, 147.8], [282.475, 147.9], [283.0, 148.0]], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal', 'parked_left': False, 'parked_right': False, 'parked_rotation': 45, 'parked_offset': 3.5, 'parked_max_distance': 4, 'mutated': False}], 'file_name': 'urban', 'obstacles': [{'name': 'trafficlightdouble', 'position': (57.4, -6.2), 'zRot': 360, 'mode': 'manual', 'sign': 'yield', 'oid': 'traffic_light_manual_0'}, {'name': 'trafficlightsingle', 'position': (59.481367606808945, 8.126788819012244), 'zRot': 253, 'mode': 'off', 'sign': 'priority'}, {'name': 'trafficlightdouble', 'position': (74.97, 6.2), 'zRot': 180, 'mode': 'off', 'sign': 'yield'}, {'name': 'trafficlightsingle', 'position': (74.70053060623883, -5.894514904334688), 'zRot': 466, 'mode': 'off', 'sign': 'priority'}, {'name': 'trafficlightsingle', 'position': (217.3821913004373, 109.04425270262183), 'zRot': 411, 'mode': 'manual', 'sign': 'yield', 'oid': 'traffic_light_manual_1'}, {'name': 'trafficlightsingle', 'position': (214.29185090845914, 129.19082779263218), 'zRot': 231, 'mode': 'off', 'sign': 'yield'}, {'name': 'trafficlightsingle', 'position': (226.13929751495405, 117.83795911415041), 'zRot': 490, 'mode': 'off', 'sign': 'priority'}, {'name': 'golf', 'position': array([83.719, 62.588]), 'zRot': -14.944050582190254, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([99.037, 73.195]), 'zRot': 0.548498676450464, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([107.053,  79.647]), 'zRot': -3.8255841627655656, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([108.969,  83.194]), 'zRot': 6.816933432357452, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([116.753,  88.281]), 'zRot': -6.416405418904418, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([120.372,  90.599]), 'zRot': -8.002294898783319, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([124.025,  93.019]), 'zRot': -18.818888914522123, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([127.342,  96.177]), 'zRot': -30.541617946363726, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([135.927,  96.564]), 'zRot': -44.9290958146224, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([153.086,  96.416]), 'zRot': 312.83897946041327, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([166.548,  94.759]), 'zRot': 312.2612019906128, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([175.198,  96.142]), 'zRot': -34.19788312822613, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([180.127,  97.453]), 'zRot': -27.692459102442484, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([183.944,  99.295]), 'zRot': -15.038774364800126, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([99.327, 46.151]), 'zRot': 163.1528224648971, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([103.005,  49.574]), 'zRot': 175.18572845942765, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([111.985,  55.755]), 'zRot': 167.42571681790082, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([115.833,  59.136]), 'zRot': 184.21225443240164, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([120.473,  62.236]), 'zRot': 182.04160981123127, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([127.401,  68.257]), 'zRot': 174.94828902170028, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([130.017,  72.857]), 'zRot': 167.1103821917199, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([134.797,  73.653]), 'zRot': 156.99281623363026, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([138.652,  76.439]), 'zRot': 141.53435913425514, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([152.483,  73.729]), 'zRot': 483.2094568171858, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([157.002,  73.8  ]), 'zRot': 479.6942404666886, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([165.991,  73.363]), 'zRot': 498.6264041977636, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([171.049,  72.61 ]), 'zRot': 132.23479885873877, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([180.102,  74.163]), 'zRot': 155.16055785395403, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([189.447,  77.96 ]), 'zRot': 166.5784060400357, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([193.378,  79.083]), 'zRot': 157.7448812969426, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([196.978,  80.832]), 'zRot': 176.15698086356088, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([203.42 ,  86.929]), 'zRot': 178.90643571272054, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 5}, {'name': 'golf', 'position': array([ 1.444, -9.6  ]), 'zRot': 143.0, 'color': (0.62, 0.24, 0.39, 1.2), 'lane': 0}], 'success_point': {'position': (283, 148), 'tolerance': 3}, 'ego_lanes': [0, 1, 4, 5, 6, 8, 9], 'directions': ['left', 'straight'], 'fitness': 0, 'triggers': [{'triggerPoint': {'position': (45.0, -4.0), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_0'}, 'spawnPoint': {'position': (95.0, 4.0), 'orientation': 0.0}}, {'triggerPoint': {'position': (202.73173019806046, 96.67220948242176), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_0'}, 'spawnPoint': {'position': (251.909, 84.096), 'orientation': 0.0}}, {'triggerPoint': {'position': (1.0, 4.0), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_2'}, 'spawnPoint': {'position': (41.0, 4.0), 'orientation': 180.0}}, {'triggerPoint': {'position': (78.624, 54.384), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_2'}, 'spawnPoint': {'position': (161.789, 87.341), 'orientation': 207.04784010930726}}, {'triggerPoint': {'position': (235.245, 148.043), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_2'}, 'spawnPoint': {'position': (277.45, 152.058), 'orientation': 180.20641725239523}}], 'tod': 0.4636903225584703, 'intersection_lanes': [[1, 2, 3, 4], [6, 7, 8]], 'participants': [{'id': 'ego', 'init_state': {'position': (1.0, -4.0), 'orientation': 0}, 'waypoints': [{'position': (1.0, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (2.571, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (4.143, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (5.714, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (7.286, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (8.857, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (10.429, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (12.0, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (13.571, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (15.143, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (16.714, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (18.286, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (19.857, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (21.429, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (23.0, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (24.571, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (26.143, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (27.714, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (29.286, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (30.857, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (32.429, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (34.0, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (35.571, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (37.143, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (38.714, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (40.286, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (41.857, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (43.429, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (45.0, 0.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (46.786, 0.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (48.571, 0.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (50.357, 0.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (52.143, 0.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (55.0, 0.0), 'tolerance': 2, 'lane': 1}, {'position': (56.786, 0.0), 'tolerance': 2, 'lane': 1}, {'position': (58.571, 0.0), 'tolerance': 2, 'lane': 1}, {'position': (60.357, 0.0), 'tolerance': 2, 'lane': 1}, {'position': (62.143, 0.0), 'tolerance': 2, 'lane': 1}, {'position': (71.167, 2.949), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (71.726, 4.728), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (72.284, 6.507), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (72.843, 8.286), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (73.402, 10.065), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (73.96, 11.844), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (74.519, 13.623), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (75.077, 15.402), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (75.636, 17.181), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (76.195, 18.96), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (76.753, 20.738), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (77.312, 22.517), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (77.871, 24.296), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (78.429, 26.075), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (78.988, 27.854), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (79.547, 29.633), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (80.105, 31.412), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (80.664, 33.191), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (81.223, 34.97), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (81.781, 36.749), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (82.34, 38.528), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (82.899, 40.307), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (83.457, 42.085), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (84.016, 43.864), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (84.575, 45.643), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (82.661, 45.235), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (100.288, 53.017), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (107.889, 56.378), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (109.196, 57.161), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (110.43, 58.62), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (113.278, 62.68), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (118.593, 70.107), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (124.931, 76.532), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (131.827, 80.546), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (139.279, 82.081), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (147.283, 81.087), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (154.433, 78.843), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (158.195, 77.646), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (159.789, 77.155), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (162.35, 77.092), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (168.589, 77.549), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (179.268, 78.34), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (186.567, 78.885), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (188.651, 79.049), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (190.133, 79.485), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (191.411, 80.353), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (195.461, 86.031), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (202.732, 96.672), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (203.981, 98.073), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (205.231, 99.475), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (206.48, 100.876), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (207.73, 102.277), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (208.98, 103.678), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (211.479, 106.481), 'tolerance': 2, 'lane': 2}, {'position': (212.728, 107.882), 'tolerance': 2, 'lane': 2}, {'position': (213.978, 109.283), 'tolerance': 2, 'lane': 2}, {'position': (215.227, 110.685), 'tolerance': 2, 'lane': 2}, {'position': (216.477, 112.086), 'tolerance': 2, 'lane': 2}, {'position': (217.727, 113.487), 'tolerance': 2, 'lane': 2}, {'position': (218.976, 114.888), 'tolerance': 2, 'lane': 2}, {'position': (223.076, 119.84), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (224.079, 121.095), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (225.083, 122.35), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (226.087, 123.605), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (227.091, 124.86), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (228.095, 126.115), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (229.099, 127.37), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (230.103, 128.625), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (231.107, 129.88), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (232.111, 131.135), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (233.115, 132.39), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (234.119, 133.645), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (235.123, 134.9), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (236.127, 136.155), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (237.131, 137.41), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (238.135, 138.665), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (235.225, 138.043), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (237.495, 138.039), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (239.642, 138.035), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (241.665, 138.031), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (243.564, 138.027), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (245.34, 138.024), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (246.993, 138.021), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (248.522, 138.019), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (251.209, 138.014), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (253.402, 138.011), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (255.1, 138.009), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (256.722, 138.009), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (258.3, 138.167), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (260.225, 138.534), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (262.248, 138.922), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (264.862, 139.423), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (266.39, 139.716), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (268.067, 140.038), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (269.892, 140.389), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (271.864, 140.768), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (273.984, 141.175), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (276.253, 141.611), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (278.669, 142.076), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (281.232, 142.569), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}], 'model': 'ETK800', 'color': 'White'}, {'id': 'other_0', 'init_state': {'position': (95.0, 4.0), 'orientation': 0.0}, 'waypoints': [{'position': (95.0, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (92.955, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (90.909, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (88.864, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (86.818, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (84.773, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (82.727, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (80.682, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (78.636, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (76.591, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (74.545, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (72.5, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (63.182, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (61.364, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (59.545, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (57.727, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (55.909, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (54.091, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (52.273, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (50.455, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (48.636, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (46.818, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (45.0, 4.0), 'tolerance': 2, 'lane': 2}, {'position': (251.909, 84.096), 'tolerance': 2, 'lane': 7}, {'position': (250.438, 85.828), 'tolerance': 2, 'lane': 7}, {'position': (248.967, 87.561), 'tolerance': 2, 'lane': 7}, {'position': (247.495, 89.293), 'tolerance': 2, 'lane': 7}, {'position': (246.024, 91.025), 'tolerance': 2, 'lane': 7}, {'position': (244.553, 92.757), 'tolerance': 2, 'lane': 7}, {'position': (243.082, 94.49), 'tolerance': 2, 'lane': 7}, {'position': (241.611, 96.222), 'tolerance': 2, 'lane': 7}, {'position': (240.139, 97.954), 'tolerance': 2, 'lane': 7}, {'position': (238.668, 99.687), 'tolerance': 2, 'lane': 7}, {'position': (237.197, 101.419), 'tolerance': 2, 'lane': 7}, {'position': (235.726, 103.151), 'tolerance': 2, 'lane': 7}, {'position': (234.255, 104.884), 'tolerance': 2, 'lane': 7}, {'position': (232.783, 106.616), 'tolerance': 2, 'lane': 7}, {'position': (231.312, 108.348), 'tolerance': 2, 'lane': 7}, {'position': (229.841, 110.081), 'tolerance': 2, 'lane': 7}, {'position': (228.37, 111.813), 'tolerance': 2, 'lane': 7}, {'position': (226.899, 113.545), 'tolerance': 2, 'lane': 7}, {'position': (211.172, 121.162), 'tolerance': 2, 'lane': 7}, {'position': (209.979, 119.824), 'tolerance': 2, 'lane': 7}, {'position': (208.786, 118.487), 'tolerance': 2, 'lane': 7}, {'position': (207.594, 117.149), 'tolerance': 2, 'lane': 7}, {'position': (206.401, 115.812), 'tolerance': 2, 'lane': 7}, {'position': (205.208, 114.474), 'tolerance': 2, 'lane': 7}, {'position': (204.015, 113.136), 'tolerance': 2, 'lane': 7}, {'position': (202.822, 111.799), 'tolerance': 2, 'lane': 7}, {'position': (201.63, 110.461), 'tolerance': 2, 'lane': 7}, {'position': (200.437, 109.124), 'tolerance': 2, 'lane': 7}, {'position': (199.244, 107.786), 'tolerance': 2, 'lane': 7}, {'position': (198.051, 106.449), 'tolerance': 2, 'lane': 7}, {'position': (196.859, 105.111), 'tolerance': 2, 'lane': 7}, {'position': (195.666, 103.774), 'tolerance': 2, 'lane': 7}], 'model': 'ETK800', 'color': 'Blue'}, {'id': 'other_1', 'init_state': {'position': (4.143, -4.0), 'orientation': 0}, 'waypoints': [{'position': (5.714, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (7.286, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (8.857, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (10.429, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (12.0, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (13.571, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (15.143, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (16.714, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (18.286, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (19.857, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (21.429, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (23.0, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (24.571, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (26.143, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (27.714, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (29.286, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (30.857, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (32.429, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (34.0, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (35.571, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (37.143, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (38.714, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (40.286, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (41.857, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (43.429, -4.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (45.0, 0.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (46.786, 0.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (48.571, 0.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (50.357, 0.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (52.143, 0.0), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (55.0, 0.0), 'tolerance': 2, 'lane': 1}, {'position': (56.786, 0.0), 'tolerance': 2, 'lane': 1}, {'position': (58.571, 0.0), 'tolerance': 2, 'lane': 1}, {'position': (60.357, 0.0), 'tolerance': 2, 'lane': 1}, {'position': (62.143, 0.0), 'tolerance': 2, 'lane': 1}, {'position': (71.167, 2.949), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (71.726, 4.728), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (72.284, 6.507), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (72.843, 8.286), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (73.402, 10.065), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (73.96, 11.844), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (74.519, 13.623), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (75.077, 15.402), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (75.636, 17.181), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (76.195, 18.96), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (76.753, 20.738), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (77.312, 22.517), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (77.871, 24.296), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (78.429, 26.075), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (78.988, 27.854), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (79.547, 29.633), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (80.105, 31.412), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (80.664, 33.191), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (81.223, 34.97), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (81.781, 36.749), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (82.34, 38.528), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (82.899, 40.307), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (83.457, 42.085), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (84.016, 43.864), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (84.575, 45.643), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (82.661, 45.235), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (100.288, 53.017), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (107.889, 56.378), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (109.196, 57.161), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (110.43, 58.62), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (113.278, 62.68), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (118.593, 70.107), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (124.931, 76.532), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (131.827, 80.546), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (139.279, 82.081), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (147.283, 81.087), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (154.433, 78.843), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (158.195, 77.646), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (159.789, 77.155), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (162.35, 77.092), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (168.589, 77.549), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (179.268, 78.34), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (186.567, 78.885), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (188.651, 79.049), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (190.133, 79.485), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (191.411, 80.353), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (195.461, 86.031), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (202.732, 96.672), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (203.981, 98.073), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (205.231, 99.475), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (206.48, 100.876), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (207.73, 102.277), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (208.98, 103.678), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (211.479, 106.481), 'tolerance': 2, 'lane': 2}, {'position': (212.728, 107.882), 'tolerance': 2, 'lane': 2}, {'position': (213.978, 109.283), 'tolerance': 2, 'lane': 2}, {'position': (215.227, 110.685), 'tolerance': 2, 'lane': 2}, {'position': (216.477, 112.086), 'tolerance': 2, 'lane': 2}, {'position': (217.727, 113.487), 'tolerance': 2, 'lane': 2}, {'position': (218.976, 114.888), 'tolerance': 2, 'lane': 2}, {'position': (223.076, 119.84), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (224.079, 121.095), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (225.083, 122.35), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (226.087, 123.605), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (227.091, 124.86), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (228.095, 126.115), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (229.099, 127.37), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (230.103, 128.625), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (231.107, 129.88), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (232.111, 131.135), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (233.115, 132.39), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (234.119, 133.645), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (235.123, 134.9), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (236.127, 136.155), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (237.131, 137.41), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (238.135, 138.665), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (235.225, 138.043), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (237.495, 138.039), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (239.642, 138.035), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (241.665, 138.031), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (243.564, 138.027), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (245.34, 138.024), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (246.993, 138.021), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (248.522, 138.019), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (251.209, 138.014), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (253.402, 138.011), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (255.1, 138.009), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (256.722, 138.009), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (258.3, 138.167), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (260.225, 138.534), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (262.248, 138.922), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (264.862, 139.423), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (266.39, 139.716), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (268.067, 140.038), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (269.892, 140.389), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (271.864, 140.768), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (273.984, 141.175), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (276.253, 141.611), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (278.669, 142.076), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (281.232, 142.569), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (245.496, 138.024), 'tolerance': 2, 'lane': 3}, {'position': (247.563, 138.02), 'tolerance': 2, 'lane': 3}, {'position': (249.43, 138.017), 'tolerance': 2, 'lane': 3}, {'position': (251.097, 138.014), 'tolerance': 2, 'lane': 3}, {'position': (253.831, 138.011), 'tolerance': 2, 'lane': 3}, {'position': (255.764, 138.009), 'tolerance': 2, 'lane': 3}, {'position': (257.273, 138.013), 'tolerance': 2, 'lane': 3}, {'position': (259.436, 138.383), 'tolerance': 2, 'lane': 3}, {'position': (261.736, 138.824), 'tolerance': 2, 'lane': 3}, {'position': (263.245, 139.113), 'tolerance': 2, 'lane': 3}, {'position': (264.994, 139.448), 'tolerance': 2, 'lane': 3}, {'position': (266.983, 139.83), 'tolerance': 2, 'lane': 3}, {'position': (269.211, 140.258), 'tolerance': 2, 'lane': 3}, {'position': (271.679, 140.732), 'tolerance': 2, 'lane': 3}, {'position': (274.386, 141.252), 'tolerance': 2, 'lane': 3}, {'position': (277.332, 141.819), 'tolerance': 2, 'lane': 3}, {'position': (280.519, 142.431), 'tolerance': 2, 'lane': 3}, {'position': (283.944, 143.09), 'tolerance': 2, 'lane': 3}], 'model': 'ETK800', 'color': 'Red'}, {'id': 'other_2', 'init_state': {'position': (41.0, 4.0), 'orientation': 180.0}, 'waypoints': [{'position': (41.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (39.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (37.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (35.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (33.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (31.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (29.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (27.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (25.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (23.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (21.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (19.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (17.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (15.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (13.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (11.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (9.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (7.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (5.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (3.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (1.0, 4.0), 'tolerance': 2, 'lane': 0}, {'position': (161.789, 87.341), 'tolerance': 2, 'lane': 5}, {'position': (159.686, 87.665), 'tolerance': 2, 'lane': 5}, {'position': (155.993, 88.84), 'tolerance': 2, 'lane': 5}, {'position': (149.345, 90.956), 'tolerance': 2, 'lane': 5}, {'position': (143.216, 92.903), 'tolerance': 2, 'lane': 5}, {'position': (140.444, 93.777), 'tolerance': 2, 'lane': 5}, {'position': (138.82, 93.99), 'tolerance': 2, 'lane': 5}, {'position': (137.329, 93.706), 'tolerance': 2, 'lane': 5}, {'position': (135.9, 93.114), 'tolerance': 2, 'lane': 5}, {'position': (131.059, 91.081), 'tolerance': 2, 'lane': 5}, {'position': (123.721, 87.991), 'tolerance': 2, 'lane': 5}, {'position': (119.072, 86.029), 'tolerance': 2, 'lane': 5}, {'position': (117.631, 85.397), 'tolerance': 2, 'lane': 5}, {'position': (116.357, 84.416), 'tolerance': 2, 'lane': 5}, {'position': (112.64, 79.203), 'tolerance': 2, 'lane': 5}, {'position': (107.179, 71.845), 'tolerance': 2, 'lane': 5}, {'position': (96.161, 63.338), 'tolerance': 2, 'lane': 5}, {'position': (78.624, 54.384), 'tolerance': 2, 'lane': 5}, {'position': (277.45, 152.058), 'tolerance': 2, 'lane': 9}, {'position': (275.173, 151.663), 'tolerance': 2, 'lane': 9}, {'position': (272.913, 151.289), 'tolerance': 2, 'lane': 9}, {'position': (270.672, 150.934), 'tolerance': 2, 'lane': 9}, {'position': (268.447, 150.601), 'tolerance': 2, 'lane': 9}, {'position': (266.241, 150.287), 'tolerance': 2, 'lane': 9}, {'position': (264.052, 149.995), 'tolerance': 2, 'lane': 9}, {'position': (261.88, 149.722), 'tolerance': 2, 'lane': 9}, {'position': (259.726, 149.47), 'tolerance': 2, 'lane': 9}, {'position': (257.589, 149.239), 'tolerance': 2, 'lane': 9}, {'position': (255.47, 149.028), 'tolerance': 2, 'lane': 9}, {'position': (253.369, 148.837), 'tolerance': 2, 'lane': 9}, {'position': (251.285, 148.667), 'tolerance': 2, 'lane': 9}, {'position': (249.219, 148.518), 'tolerance': 2, 'lane': 9}, {'position': (247.17, 148.389), 'tolerance': 2, 'lane': 9}, {'position': (245.138, 148.28), 'tolerance': 2, 'lane': 9}, {'position': (243.125, 148.192), 'tolerance': 2, 'lane': 9}, {'position': (241.128, 148.124), 'tolerance': 2, 'lane': 9}, {'position': (239.15, 148.077), 'tolerance': 2, 'lane': 9}, {'position': (237.188, 148.05), 'tolerance': 2, 'lane': 9}, {'position': (235.245, 148.043), 'tolerance': 2, 'lane': 9}], 'model': 'ETK800', 'color': 'Purple'}], 'parked_color': (0.62, 0.24, 0.39, 1.2)}
        child = self._mutate_traffic_signs(child)   # TODO Traffic lights or traffic sign
        child = self._mutate_traffic_light_mode(child)  # TODO Traffic light mode
        child = self._mutate_parked_cars(child) # TODO Parked cars
        child = self._mutate_traffic(child) # TODO Traffic
        self._mutate_tod(child)
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
        while i < len(lanes):
            control_points = lanes[i].get("control_points")
            mutated = False
            if lanes[i].get("type") == "normal":
                penul_point = control_points[-1]
                j = 2 if i == 0 else 1
                while j < len(control_points):
                    if random() <= self.mutation_probability:
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
                            if not intersection_check_all(control_points_lines) and MIN_DEGREES <= deg <= MAX_DEGREES\
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
                if random() <= self.mutation_probability:
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
                        if not intersection_check_all(control_points_lines)\
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
        for connection in connected_lanes:
            if random() <= self.mutation_probability:
                left_lanes = randint(1, 2)
                right_lanes = randint(1, 2)
                width_per_lane = randint(4, 5)
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
        return individual

    def _mutate_traffic_light_mode(self, individual):
        return individual

    def _mutate_parked_cars(self, individual):
        return individual

    def _mutate_traffic(self, individual):
        return individual

    def _mutate_tod(self, individual):
        """Mutates the time of day.
        :param individual: Individual to mutate.
        :return: Void.
        """
        if random() <= self.mutation_probability:
            # Mutate time of day.
            individual["tod"] = random()

    def _update(self, individual):
        """Updates obstacle positions, waypoints, trigger and spawn points, success point and initial state of vehicles
           after mutating the control points.
        :param individual: Individual to update.
        :return: Void.
        """
        # Update traffic signs and lights position.
        intersections = _create_intersections_manual(individual)
        i = 0
        for intersection in intersections:
            obs, tgr = _add_traffic_signs(intersection.get("last_point"), intersection.get("current_left_lanes"),
                                          intersection.get("current_right_lanes"), intersection.get("current_width"),
                                          intersection)
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
        temp = self._spline_population([individual])
        lanes = temp[0].get("lanes")
        for idx, lane in enumerate(lanes):
            control_points = lane.get("control_points")
            #print(lane.get("parked_left"))
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
            parked_cars.append({"name": "golf", "position": position[0], "zRot": position[1], "color": color,
                                "lane": position[2]})
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
            self.population_list = [{'lanes': [{'control_points': [(1, 0), (30, 0), (45, 0), (52, -26), (38, -51), (18, -61), (8, -73), (0, -88), (-8, -115), (-11, -138)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {'control_points': [(-11, -138), (-13.586783681355362, -157.83200822372442)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(-13.586783681355362, -157.83200822372442), (-17.466959203388402, -187.5800205593111)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(-13.586783681355362, -157.83200822372442), (-63.42791825820418, -161.81463350015224)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(-13.586783681355362, -157.83200822372442), (31.936069893591835, -178.51218719765788)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(31.936069893591835, -178.51218719765788), (50, -189)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}, {'control_points': [(50, -189), (67.29616177749516, -199.04205097411457)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(67.29616177749516, -199.04205097411457), (85.3279138972033, -152.4067004823106)], 'width': 20, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(67.29616177749516, -199.04205097411457), (93.24040444373787, -214.10512743528648)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(93.24040444373787, -214.10512743528648), (109, -216)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}], 'file_name': 'urban', 'obstacles': [{'name': 'trafficlightsingle', 'position': (-18.945737207896542, -150.98134277426828), 'zRot': 263, 'mode': 'blinking', 'sign': 'yield', 'oid': None}, {'name': 'trafficlightdouble', 'position': (-5.468653352444055, -154.71015621525086), 'zRot': 516, 'mode': 'blinking', 'sign': 'priority'}, {'name': 'trafficlightsingle', 'position': (-8.534364021054785, -167.0327666476919), 'zRot': 443, 'mode': 'blinking', 'sign': 'yield'}, {'name': 'trafficlightdouble', 'position': (-20.888091594897492, -164.63519150448695), 'zRot': 365, 'mode': 'blinking', 'sign': 'priority'}, {'name': 'trafficlightdouble', 'position': (54.41079457123487, -198.73010232476332), 'zRot': 330, 'mode': 'off', 'sign': 'yield', 'oid': None}, {'name': 'trafficlightdouble', 'position': (60.4620686421558, -188.43356045861205), 'zRot': 249, 'mode': 'off', 'sign': 'priority'}, {'name': 'trafficlightdouble', 'position': (79.14375927710574, -198.75147656501898), 'zRot': 510, 'mode': 'off', 'sign': 'yield'}], 'success_point': {'position': (109, -216), 'tolerance': 3}, 'ego_lanes': [0, 1, 4, 5, 6, 8, 9], 'directions': ['left', 'straight'], 'fitness': 0, 'triggers': [], 'tod': 0.6067928732413833, 'intersection_lanes': [[1, 2, 3, 4], [6, 7, 8]]}, {'lanes': [{'control_points': [(1, 0), (30, 0), (45, 0)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}, {'control_points': [(45, 0), (65.0, 0.0)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.0, 0.0), (95.0, 0.0)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.0, 0.0), (78.78186779084996, -48.063084796915945)], 'width': 15, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.0, 0.0), (79.61858523613682, 47.81523779815178)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(79.61858523613682, 47.81523779815178), (106, 61), (120, 81), (139, 89), (161, 82), (188, 84), (204, 104)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {'control_points': [(204, 104), (216.49390095108845, 119.61737618886059)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(216.49390095108845, 119.61737618886059), (248.86013173184494, 81.50665453409778)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(216.49390095108845, 119.61737618886059), (235.23475237772118, 143.0434404721515)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(235.23475237772118, 143.0434404721515), (257, 143), (283, 148)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}], 'file_name': 'urban', 'obstacles': [{'name': 'trafficlightdouble', 'position': (57.4, -6.2), 'zRot': 360, 'mode': 'manual', 'sign': 'yield', 'oid': 'traffic_light_manual_0'}, {'name': 'trafficlightsingle', 'position': (59.419920777893324, 8.08472113773959), 'zRot': 253, 'mode': 'off', 'sign': 'priority'}, {'name': 'trafficlightdouble', 'position': (74.97, 6.2), 'zRot': 180, 'mode': 'off', 'sign': 'yield'}, {'name': 'trafficlightsingle', 'position': (74.70053060623883, -5.894514904334688), 'zRot': 466, 'mode': 'off', 'sign': 'priority'}, {'name': 'trafficlightsingle', 'position': (217.70268586810627, 108.8023431780746), 'zRot': 411, 'mode': 'manual', 'sign': 'yield', 'oid': 'traffic_light_manual_1'}, {'name': 'trafficlightsingle', 'position': (214.29185090845914, 129.19082779263218), 'zRot': 231, 'mode': 'off', 'sign': 'yield'}, {'name': 'trafficlightsingle', 'position': (226.13929751495405, 117.83795911415041), 'zRot': 490, 'mode': 'off', 'sign': 'priority'}], 'success_point': {'position': (283, 148), 'tolerance': 3}, 'ego_lanes': [0, 1, 4, 5, 6, 8, 9], 'directions': ['left', 'straight'], 'fitness': 0, 'triggers': [{'triggerPoint': {'position': (45, 0), 'action': 'switchLights', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'traffic_light_manual_0', 'initState': 'green', 'switchTo': 'red'}}, {'triggerPoint': {'position': (204, 104), 'action': 'switchLights', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'traffic_light_manual_1', 'initState': 'green', 'switchTo': 'red'}}], 'tod': 0.4636903225584703, 'intersection_lanes': [[1, 2, 3, 4], [6, 7, 8]]}, {'lanes': [{'control_points': [(1, 0), (30, 0), (45, 0), (62, -7)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {'control_points': [(62, -7), (80.49356196949434, -14.614996105085897)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(80.49356196949434, -14.614996105085897), (108.23390492373582, -26.037490262714744)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(80.49356196949434, -14.614996105085897), (71.48465589082818, -63.79669606824062)], 'width': 20, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(80.49356196949434, -14.614996105085897), (107.27026331454304, 27.61068642436609)], 'width': 20, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(107.27026331454304, 27.61068642436609), (130, 47), (138, 75), (127, 100), (119, 123), (110, 142), (112, 157)], 'width': 20, 'left_lanes': 2, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}, {'control_points': [(112, 157), (114.64327440182038, 176.82455801365268)], 'width': 20, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(114.64327440182038, 176.82455801365268), (68.48096734299551, 196.03500847705322)], 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(114.64327440182038, 176.82455801365268), (164.55498834970186, 179.79454299422986)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(164.55498834970186, 179.79454299422986), (167, 196)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}], 'file_name': 'urban', 'obstacles': [{'name': 'trafficlightsingle', 'position': (68.79366438232307, -16.502427282560767), 'zRot': 338, 'mode': 'off', 'sign': 'yield', 'oid': None}, {'name': 'trafficlightdouble', 'position': (75.14628029758208, -4.001015762102817), 'zRot': 238, 'mode': 'off', 'sign': 'priority'}, {'name': 'trafficlightsingle', 'position': (93.54348958043869, -13.283459643282317), 'zRot': 518, 'mode': 'off', 'sign': 'yield'}, {'name': 'trafficlightdouble', 'position': (89.11763585127451, -24.14483081937119), 'zRot': 440, 'mode': 'off', 'sign': 'priority'}, {'name': 'trafficlightdouble', 'position': (123.94760029622803, 169.42999787456023), 'zRot': 442, 'mode': 'manual', 'sign': 'yield', 'oid': 'traffic_light_manual_2'}, {'name': 'trafficlightdouble', 'position': (102.9363925184761, 174.98094293196527), 'zRot': 337, 'mode': 'off', 'sign': 'priority'}, {'name': 'trafficlightdouble', 'position': (125.9443549852435, 183.70799303164893), 'zRot': 183, 'mode': 'off', 'sign': 'priority'}], 'success_point': {'position': (167, 196), 'tolerance': 3}, 'ego_lanes': [0, 1, 4, 5, 6, 8, 9], 'directions': ['left', 'right'], 'fitness': 0, 'triggers': [{'triggerPoint': {'position': (112, 157), 'action': 'switchLights', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'traffic_light_manual_2', 'initState': 'green', 'switchTo': 'red'}}], 'tod': 0.07900698944697848, 'intersection_lanes': [[1, 2, 3, 4], [6, 7, 8]]}]

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

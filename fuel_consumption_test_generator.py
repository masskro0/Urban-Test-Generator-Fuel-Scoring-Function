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
        self.mutation_probability = 0.5 # I have eight mutatable properties.
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
            #self._mutate_points(child)
        #if random() <= self.mutation_probability * 2:
            #self._mutate_num_lanes_and_width(child)
        #child = self._update(child)
        #if random() <= self.mutation_probability:
            #self._mutate_traffic_signs(child)
        #if random() <= self.mutation_probability:
            #child = self._mutate_traffic_light_mode(child)
        child = {'lanes': [{'control_points': [(1, 0), (30, 0), (45, 0), (71, -3), (100, 3)], 'width': 20.0, 'left_lanes': 2, 'right_lanes': 2, 'samples': 100, 'type': 'normal', 'parked_left': True, 'parked_right': False, 'parked_rotation': 0, 'parked_offset': 2, 'parked_max_distance': 5.5, 'mutated': False}, {'control_points': [(100, 3), (119.58520839014204, 7.052112080719045)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [(119.58520839014204, 7.052112080719045), (148.96302097535516, 13.130280201797614)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [(119.58520839014204, 7.052112080719045), (112.35821849061614, -42.42283730775581)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [(119.58520839014204, 7.052112080719045), (97.1275467815048, 51.72484912549774)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [(97.1275467815048, 51.72484912549774), (83, 72), (89, 90)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal', 'parked_left': False, 'parked_right': False, 'parked_rotation': 45, 'parked_offset': 3.5, 'parked_max_distance': 4, 'mutated': False}, {'control_points': [(89, 90), (95.32455532033674, 108.97366596101028)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [(95.32455532033674, 108.97366596101028), (46.000937629859436, 117.17005162321743)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [(95.32455532033674, 108.97366596101028), (145.32311463552404, 108.59410515468579)], 'width': 15, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [(95.32455532033674, 108.97366596101028), (104.81138830084188, 137.4341649025257)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [(104.81138830084188, 137.4341649025257), (103, 162), (116, 184), (144, 191)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal', 'parked_left': True, 'parked_right': True, 'parked_rotation': 45, 'parked_offset': 3.5, 'parked_max_distance': 4, 'mutated': True}, {'control_points': [(144, 191), (144.49390095108848, 219.6173761888606)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [(144.49390095108848, 219.6173761888606), (108.32132060048988, 254.1361309575293)], 'width': 20, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [(144.49390095108848, 219.6173761888606), (163.23475237772118, 243.04344047215147)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection', 'mutated': False}, {'control_points': [(163.23475237772118, 243.04344047215147), (178, 252)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal', 'parked_left': True, 'parked_right': True, 'parked_rotation': 0, 'parked_offset': 2, 'parked_max_distance': 5.5, 'mutated': False}], 'file_name': 'urban', 'obstacles': [{'name': 'stopsign', 'position': (112.94177063380319, 0.3674778515595194), 'zRot': 372, 'intersection_id': 2, 'facingEgo': True, 'lane_id': 1}, {'name': 'prioritysign', 'position': (111.70084893351957, 11.15832395527785), 'zRot': 297, 'intersection_id': 2}, {'name': 'stopsign', 'position': (123.52588738864132, 13.177554842739335), 'zRot': 192, 'intersection_id': 2}, {'name': 'prioritysign', 'position': (123.99345015679178, 1.2540602935439065), 'zRot': 442, 'intersection_id': 2}, {'name': 'stopsign', 'position': (97.85437744847145, 100.11928851253883), 'zRot': 432, 'intersection_id': 3, 'facingEgo': True, 'lane_id': 6}, {'name': 'prioritysign', 'position': (87.71929469336148, 102.43188403283662), 'zRot': 351, 'intersection_id': 3}, {'name': 'stopsign', 'position': (92.79473319220205, 117.82804340948174), 'zRot': 252, 'intersection_id': 3}, {'name': 'prioritysign', 'position': (102.98278870041919, 116.61575085298779), 'zRot': 540, 'intersection_id': 3}, {'name': 'stopsign', 'position': (149.51883896835096, 209.4291477892482), 'zRot': 411, 'intersection_id': 4, 'facingEgo': True, 'lane_id': 11}, {'name': 'prioritysign', 'position': (133.76247178251901, 215.75908278374266), 'zRot': 316, 'intersection_id': 4}, {'name': 'stopsign', 'position': (147.2488061108035, 231.3850691471671), 'zRot': 231, 'intersection_id': 4}, {'name': 'golf', 'position': array([15.836,  6.194]), 'zRot': 357.93105204349405, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 0}, {'name': 'golf', 'position': array([34.094,  7.463]), 'zRot': 361.91566677071154, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 0}, {'name': 'golf', 'position': array([57.635,  5.617]), 'zRot': 351.4941250445862, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 0}, {'name': 'golf', 'position': array([69.504,  5.223]), 'zRot': 1.9886324552294226, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 0}, {'name': 'golf', 'position': array([ 95.827, 150.397]), 'zRot': 35.7450121354009, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([ 97.484, 154.233]), 'zRot': 32.70158026952741, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([ 97.864, 158.242]), 'zRot': 38.71999856303255, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([ 99.6  , 161.904]), 'zRot': 27.885499839830416, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([ 99.849, 166.829]), 'zRot': 20.74522599044579, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([101.673, 170.422]), 'zRot': 22.227745317953847, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([104.016, 178.141]), 'zRot': 20.7723505325706, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([105.81 , 181.744]), 'zRot': 16.76764046708736, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([109.114, 184.493]), 'zRot': 14.910258054528782, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([109.993, 188.595]), 'zRot': 7.170368265415902, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([112.098, 191.997]), 'zRot': 7.5990164716054664, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([114.568, 196.112]), 'zRot': 16.933252845367086, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([118.224, 198.394]), 'zRot': 2.701266581844685, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([113.927, 139.053]), 'zRot': 223.5976934716822, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([112.762, 143.435]), 'zRot': 221.8141102416576, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([113.072, 151.653]), 'zRot': 220.73104287655946, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([113.885, 156.046]), 'zRot': 208.54574781929568, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([116.809, 164.647]), 'zRot': 208.27002169833327, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([119.702, 167.727]), 'zRot': 200.51533347581946, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([120.042, 171.997]), 'zRot': 202.92303998661902, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([123.829, 179.385]), 'zRot': 193.5675214623911, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([126.052, 183.004]), 'zRot': 194.6159779215276, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 10}, {'name': 'golf', 'position': array([ 8.777, 10.999]), 'zRot': 357.9926326630838, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 0}, {'name': 'golf', 'position': array([14.18 , 12.296]), 'zRot': 367.9338003997488, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 0}, {'name': 'golf', 'position': array([20.469, 11.184]), 'zRot': 363.8448323914995, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 0}, {'name': 'golf', 'position': array([26.236, 12.858]), 'zRot': 355.6386800541189, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 0}, {'name': 'golf', 'position': array([48.606, 12.021]), 'zRot': 355.90888827619307, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 0}, {'name': 'golf', 'position': array([73.091, 10.09 ]), 'zRot': 4.620357764317442, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 0}, {'name': 'golf', 'position': array([78.702, 11.882]), 'zRot': 7.700959934404273, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 0}, {'name': 'golf', 'position': array([85.322, 12.102]), 'zRot': 13.668184912994729, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 0}, {'name': 'golf', 'position': array([160.131, 249.465]), 'zRot': 25.23478468799056, 'color': (0.72, 0.95, 0.17, 1.06), 'lane': 14}], 'success_point': {'position': (178, 252), 'tolerance': 3}, 'ego_lanes': [0, 1, 4, 5, 6, 9, 10, 11, 13, 14], 'directions': ['left', 'straight', 'straight'], 'fitness': 0, 'triggers': [{'triggerPoint': {'position': (100.50651401008989, 0.5518489512322429), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_0'}, 'spawnPoint': {'position': (148.457, 15.578), 'orientation': 0.0}}, {'triggerPoint': {'position': (91.37170824512629, 89.20943058495791), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_0'}, 'spawnPoint': {'position': (145.361, 113.594), 'orientation': 0.0}}, {'triggerPoint': {'position': (146.49962775137254, 190.95685947881375), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_0'}, 'spawnPoint': {'position': (141.994, 219.661), 'orientation': 0.0}}, {'triggerPoint': {'position': (1.0, 7.5), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_2'}, 'spawnPoint': {'position': (84.453, 7.74), 'orientation': 180.0}}, {'triggerPoint': {'position': (95.076, 50.296), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_2'}, 'spawnPoint': {'position': (80.649, 72.836), 'orientation': 304.8657487126043}}, {'triggerPoint': {'position': (102.318, 137.25), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_2'}, 'spawnPoint': {'position': (115.168, 186.354), 'orientation': 274.2127439466662}}, {'triggerPoint': {'position': (161.938, 245.181), 'action': 'spawnAndStart', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'other_2'}, 'spawnPoint': {'position': (175.361, 253.323), 'orientation': 211.2392148882083}}], 'tod': 0.10629639474288721, 'intersection_lanes': [[1, 2, 3, 4], [6, 7, 8, 9], [11, 12, 13]], 'participants': [{'id': 'ego', 'init_state': {'position': (1.0, -7.5), 'orientation': 0}, 'waypoints': [{'position': (1.0, -7.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (8.535, -7.512), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (15.579, -7.547), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (22.132, -7.606), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (28.194, -7.688), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (33.766, -7.794), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (38.846, -7.923), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (43.436, -8.076), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (47.535, -8.253), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (51.144, -8.453), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (54.261, -8.676), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (56.888, -8.923), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (59.118, -9.179), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (61.144, -9.412), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (62.973, -9.622), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (64.605, -9.809), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (67.275, -10.114), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (69.155, -10.327), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (70.652, -10.481), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (72.538, -10.323), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (74.686, -9.885), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (76.444, -9.524), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (78.658, -9.068), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (81.328, -8.518), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (84.454, -7.872), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (88.036, -7.133), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (92.075, -6.298), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (96.569, -5.369), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (100.507, 0.552), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (102.255, 0.914), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (104.004, 1.275), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (105.753, 1.637), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (107.501, 1.999), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (110.299, 2.578), 'tolerance': 2, 'lane': 1}, {'position': (112.048, 2.94), 'tolerance': 2, 'lane': 1}, {'position': (113.796, 3.301), 'tolerance': 2, 'lane': 1}, {'position': (115.545, 3.663), 'tolerance': 2, 'lane': 1}, {'position': (117.294, 4.025), 'tolerance': 2, 'lane': 1}, {'position': (119.814, 12.164), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (119.012, 13.759), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (118.21, 15.355), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (117.408, 16.95), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (116.605, 18.545), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (115.803, 20.141), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (115.001, 21.736), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (114.199, 23.332), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (113.397, 24.927), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (112.595, 26.523), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (111.793, 28.118), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (110.991, 29.714), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (110.189, 31.309), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (109.387, 32.905), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (108.585, 34.5), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (107.783, 36.095), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (106.981, 37.691), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (106.179, 39.286), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (105.377, 40.882), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (104.575, 42.477), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (103.773, 44.073), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (102.97, 45.668), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (102.168, 47.264), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (101.366, 48.859), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (100.564, 50.455), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (99.179, 53.154), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (98.245, 54.526), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (97.36, 55.891), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (96.523, 57.25), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (95.735, 58.603), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (94.996, 59.95), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (94.305, 61.29), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (93.359, 63.289), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (92.522, 65.275), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (91.795, 67.246), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (91.177, 69.203), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (90.667, 71.146), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (90.267, 73.075), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (89.976, 74.991), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (89.795, 76.892), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (89.722, 78.779), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (89.758, 80.652), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (89.904, 82.512), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (90.159, 84.357), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (90.523, 86.188), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (91.372, 89.209), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (91.936, 90.904), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (92.501, 92.598), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (93.066, 94.292), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (93.63, 95.986), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 1}, {'position': (94.534, 98.696), 'tolerance': 2, 'lane': 2}, {'position': (95.099, 100.39), 'tolerance': 2, 'lane': 2}, {'position': (95.663, 102.084), 'tolerance': 2, 'lane': 2}, {'position': (96.228, 103.778), 'tolerance': 2, 'lane': 2}, {'position': (96.793, 105.473), 'tolerance': 2, 'lane': 2}, {'position': (99.052, 112.249), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (99.56, 113.774), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (100.068, 115.298), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (100.576, 116.823), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (101.084, 118.348), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (101.593, 119.872), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (102.101, 121.397), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (102.609, 122.922), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (103.117, 124.446), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (103.626, 125.971), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (104.134, 127.496), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (104.642, 129.02), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (105.15, 130.545), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (105.658, 132.07), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (106.167, 133.594), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (106.675, 135.119), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (107.305, 137.618), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (107.189, 139.3), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (107.094, 140.947), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (107.018, 142.559), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (106.962, 144.137), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (106.927, 145.68), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (106.911, 147.188), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (106.938, 150.101), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (107.045, 152.876), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (107.232, 155.512), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (107.498, 158.01), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (107.844, 160.369), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (108.269, 162.589), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (108.773, 164.672), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (109.357, 166.615), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (110.02, 168.42), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (110.763, 170.087), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (111.586, 171.615), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (112.555, 173.057), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (113.737, 174.463), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (115.134, 175.833), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (116.744, 177.168), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (118.569, 178.468), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (120.607, 179.733), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (122.858, 180.962), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (125.324, 182.155), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (128.003, 183.314), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (129.423, 183.88), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (130.896, 184.437), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (132.423, 184.985), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (134.003, 185.524), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (135.637, 186.055), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (137.324, 186.576), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (139.064, 187.089), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (140.858, 187.593), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (142.706, 188.088), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (146.5, 190.957), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (146.526, 192.49), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (146.553, 194.023), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (146.579, 195.556), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (146.605, 197.089), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (146.632, 198.622), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (146.658, 200.155), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (146.685, 201.688), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (146.711, 203.221), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 2}, {'position': (146.747, 205.266), 'tolerance': 2, 'lane': 3}, {'position': (146.773, 206.799), 'tolerance': 2, 'lane': 3}, {'position': (146.799, 208.332), 'tolerance': 2, 'lane': 3}, {'position': (146.826, 209.865), 'tolerance': 2, 'lane': 3}, {'position': (146.852, 211.398), 'tolerance': 2, 'lane': 3}, {'position': (146.879, 212.931), 'tolerance': 2, 'lane': 3}, {'position': (146.905, 214.464), 'tolerance': 2, 'lane': 3}, {'position': (146.932, 215.997), 'tolerance': 2, 'lane': 3}, {'position': (146.958, 217.53), 'tolerance': 2, 'lane': 3}, {'position': (149.123, 221.402), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (150.127, 222.657), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (151.131, 223.912), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (152.135, 225.167), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (153.139, 226.422), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (154.143, 227.677), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (155.147, 228.932), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (156.151, 230.187), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (157.155, 231.442), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (158.159, 232.697), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (159.163, 233.952), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (160.167, 235.207), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (161.171, 236.462), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (162.175, 237.717), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (163.179, 238.972), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (164.183, 240.227), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (165.322, 241.386), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (166.641, 242.185), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (167.959, 242.985), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (169.277, 243.785), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (170.596, 244.585), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (171.914, 245.384), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (173.232, 246.184), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (174.551, 246.984), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (175.869, 247.783), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}, {'position': (177.187, 248.583), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 3}], 'model': 'ETK800', 'color': 'White'}, {'id': 'other_0', 'init_state': {'position': (148.457, 15.578), 'orientation': 0.0}, 'waypoints': [{'position': (148.457, 15.578), 'tolerance': 2, 'lane': 2}, {'position': (146.453, 15.164), 'tolerance': 2, 'lane': 2}, {'position': (144.45, 14.75), 'tolerance': 2, 'lane': 2}, {'position': (142.447, 14.335), 'tolerance': 2, 'lane': 2}, {'position': (140.444, 13.921), 'tolerance': 2, 'lane': 2}, {'position': (138.441, 13.506), 'tolerance': 2, 'lane': 2}, {'position': (136.438, 13.092), 'tolerance': 2, 'lane': 2}, {'position': (134.435, 12.677), 'tolerance': 2, 'lane': 2}, {'position': (132.432, 12.263), 'tolerance': 2, 'lane': 2}, {'position': (130.429, 11.849), 'tolerance': 2, 'lane': 2}, {'position': (128.426, 11.434), 'tolerance': 2, 'lane': 2}, {'position': (126.423, 11.02), 'tolerance': 2, 'lane': 2}, {'position': (116.454, 2.916), 'tolerance': 2, 'lane': 2}, {'position': (116.126, 0.667), 'tolerance': 2, 'lane': 2}, {'position': (115.797, -1.582), 'tolerance': 2, 'lane': 2}, {'position': (115.469, -3.831), 'tolerance': 2, 'lane': 2}, {'position': (115.14, -6.08), 'tolerance': 2, 'lane': 2}, {'position': (114.812, -8.329), 'tolerance': 2, 'lane': 2}, {'position': (114.483, -10.577), 'tolerance': 2, 'lane': 2}, {'position': (114.155, -12.826), 'tolerance': 2, 'lane': 2}, {'position': (113.826, -15.075), 'tolerance': 2, 'lane': 2}, {'position': (113.498, -17.324), 'tolerance': 2, 'lane': 2}, {'position': (113.169, -19.573), 'tolerance': 2, 'lane': 2}, {'position': (112.841, -21.822), 'tolerance': 2, 'lane': 2}, {'position': (112.512, -24.071), 'tolerance': 2, 'lane': 2}, {'position': (112.184, -26.319), 'tolerance': 2, 'lane': 2}, {'position': (111.855, -28.568), 'tolerance': 2, 'lane': 2}, {'position': (111.527, -30.817), 'tolerance': 2, 'lane': 2}, {'position': (111.198, -33.066), 'tolerance': 2, 'lane': 2}, {'position': (110.87, -35.315), 'tolerance': 2, 'lane': 2}, {'position': (110.541, -37.564), 'tolerance': 2, 'lane': 2}, {'position': (110.213, -39.813), 'tolerance': 2, 'lane': 2}, {'position': (109.884, -42.061), 'tolerance': 2, 'lane': 2}, {'position': (145.361, 113.594), 'tolerance': 2, 'lane': 8}, {'position': (143.088, 113.611), 'tolerance': 2, 'lane': 8}, {'position': (140.816, 113.628), 'tolerance': 2, 'lane': 8}, {'position': (138.543, 113.646), 'tolerance': 2, 'lane': 8}, {'position': (136.27, 113.663), 'tolerance': 2, 'lane': 8}, {'position': (133.998, 113.68), 'tolerance': 2, 'lane': 8}, {'position': (131.725, 113.697), 'tolerance': 2, 'lane': 8}, {'position': (129.452, 113.715), 'tolerance': 2, 'lane': 8}, {'position': (127.18, 113.732), 'tolerance': 2, 'lane': 8}, {'position': (124.907, 113.749), 'tolerance': 2, 'lane': 8}, {'position': (122.634, 113.766), 'tolerance': 2, 'lane': 8}, {'position': (120.362, 113.784), 'tolerance': 2, 'lane': 8}, {'position': (118.089, 113.801), 'tolerance': 2, 'lane': 8}, {'position': (115.816, 113.818), 'tolerance': 2, 'lane': 8}, {'position': (113.544, 113.835), 'tolerance': 2, 'lane': 8}, {'position': (111.271, 113.853), 'tolerance': 2, 'lane': 8}, {'position': (108.998, 113.87), 'tolerance': 2, 'lane': 8}, {'position': (106.726, 113.887), 'tolerance': 2, 'lane': 8}, {'position': (92.378, 108.039), 'tolerance': 2, 'lane': 8}, {'position': (91.803, 106.314), 'tolerance': 2, 'lane': 8}, {'position': (91.228, 104.59), 'tolerance': 2, 'lane': 8}, {'position': (90.653, 102.865), 'tolerance': 2, 'lane': 8}, {'position': (90.078, 101.14), 'tolerance': 2, 'lane': 8}, {'position': (89.503, 99.415), 'tolerance': 2, 'lane': 8}, {'position': (88.928, 97.69), 'tolerance': 2, 'lane': 8}, {'position': (88.353, 95.965), 'tolerance': 2, 'lane': 8}, {'position': (87.778, 94.24), 'tolerance': 2, 'lane': 8}, {'position': (87.203, 92.515), 'tolerance': 2, 'lane': 8}, {'position': (86.628, 90.791), 'tolerance': 2, 'lane': 8}, {'position': (141.994, 219.661), 'tolerance': 2, 'lane': 11}, {'position': (141.961, 217.709), 'tolerance': 2, 'lane': 11}, {'position': (141.927, 215.758), 'tolerance': 2, 'lane': 11}, {'position': (141.893, 213.807), 'tolerance': 2, 'lane': 11}, {'position': (141.86, 211.856), 'tolerance': 2, 'lane': 11}, {'position': (141.826, 209.905), 'tolerance': 2, 'lane': 11}, {'position': (141.792, 207.953), 'tolerance': 2, 'lane': 11}, {'position': (141.759, 206.002), 'tolerance': 2, 'lane': 11}, {'position': (141.725, 204.051), 'tolerance': 2, 'lane': 11}, {'position': (141.691, 202.1), 'tolerance': 2, 'lane': 11}, {'position': (141.658, 200.149), 'tolerance': 2, 'lane': 11}, {'position': (141.624, 198.197), 'tolerance': 2, 'lane': 11}, {'position': (146.383, 228.181), 'tolerance': 2, 'lane': 11}, {'position': (144.739, 229.75), 'tolerance': 2, 'lane': 11}, {'position': (143.095, 231.319), 'tolerance': 2, 'lane': 11}, {'position': (141.451, 232.888), 'tolerance': 2, 'lane': 11}, {'position': (139.806, 234.457), 'tolerance': 2, 'lane': 11}, {'position': (138.162, 236.027), 'tolerance': 2, 'lane': 11}, {'position': (136.518, 237.596), 'tolerance': 2, 'lane': 11}, {'position': (134.874, 239.165), 'tolerance': 2, 'lane': 11}, {'position': (133.23, 240.734), 'tolerance': 2, 'lane': 11}, {'position': (131.585, 242.303), 'tolerance': 2, 'lane': 11}, {'position': (129.941, 243.872), 'tolerance': 2, 'lane': 11}, {'position': (128.297, 245.441), 'tolerance': 2, 'lane': 11}, {'position': (126.653, 247.01), 'tolerance': 2, 'lane': 11}, {'position': (125.009, 248.579), 'tolerance': 2, 'lane': 11}, {'position': (123.364, 250.148), 'tolerance': 2, 'lane': 11}, {'position': (121.72, 251.717), 'tolerance': 2, 'lane': 11}, {'position': (120.076, 253.286), 'tolerance': 2, 'lane': 11}, {'position': (118.432, 254.855), 'tolerance': 2, 'lane': 11}, {'position': (116.788, 256.424), 'tolerance': 2, 'lane': 11}, {'position': (115.143, 257.993), 'tolerance': 2, 'lane': 11}, {'position': (113.499, 259.562), 'tolerance': 2, 'lane': 11}], 'model': 'ETK800', 'color': 'White'}, {'id': 'other_1', 'init_state': {'position': (15.579, -7.547), 'orientation': 0}, 'waypoints': [{'position': (22.132, -7.606), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (28.194, -7.688), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (33.766, -7.794), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (38.846, -7.923), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (43.436, -8.076), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (47.535, -8.253), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (51.144, -8.453), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (54.261, -8.676), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (56.888, -8.923), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (59.118, -9.179), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (61.144, -9.412), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (62.973, -9.622), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (64.605, -9.809), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (67.275, -10.114), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (69.155, -10.327), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (70.652, -10.481), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (72.538, -10.323), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (74.686, -9.885), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (76.444, -9.524), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (78.658, -9.068), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (81.328, -8.518), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (84.454, -7.872), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (88.036, -7.133), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (92.075, -6.298), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (96.569, -5.369), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (100.507, 0.552), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (102.255, 0.914), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (104.004, 1.275), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (105.753, 1.637), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (107.501, 1.999), 'tolerance': 2, 'movementMode': '_BEAMNG', 'lane': 0}, {'position': (122.762, 5.157), 'tolerance': 2, 'lane': 0}, {'position': (124.765, 5.571), 'tolerance': 2, 'lane': 0}, {'position': (126.768, 5.985), 'tolerance': 2, 'lane': 0}, {'position': (128.772, 6.4), 'tolerance': 2, 'lane': 0}, {'position': (130.775, 6.814), 'tolerance': 2, 'lane': 0}, {'position': (132.778, 7.229), 'tolerance': 2, 'lane': 0}, {'position': (134.781, 7.643), 'tolerance': 2, 'lane': 0}, {'position': (136.784, 8.057), 'tolerance': 2, 'lane': 0}, {'position': (138.787, 8.472), 'tolerance': 2, 'lane': 0}, {'position': (140.79, 8.886), 'tolerance': 2, 'lane': 0}, {'position': (142.793, 9.301), 'tolerance': 2, 'lane': 0}, {'position': (144.796, 9.715), 'tolerance': 2, 'lane': 0}, {'position': (146.799, 10.13), 'tolerance': 2, 'lane': 0}, {'position': (148.802, 10.544), 'tolerance': 2, 'lane': 0}], 'model': 'ETK800', 'color': 'White'}, {'id': 'other_2', 'init_state': {'position': (84.453, 7.74), 'orientation': 180.0}, 'waypoints': [{'position': (84.453, 7.74), 'tolerance': 2, 'lane': 0}, {'position': (81.305, 7.255), 'tolerance': 2, 'lane': 0}, {'position': (78.3, 6.838), 'tolerance': 2, 'lane': 0}, {'position': (75.438, 6.487), 'tolerance': 2, 'lane': 0}, {'position': (72.72, 6.204), 'tolerance': 2, 'lane': 0}, {'position': (70.145, 5.987), 'tolerance': 2, 'lane': 0}, {'position': (67.713, 5.837), 'tolerance': 2, 'lane': 0}, {'position': (65.425, 5.753), 'tolerance': 2, 'lane': 0}, {'position': (63.28, 5.737), 'tolerance': 2, 'lane': 0}, {'position': (61.278, 5.787), 'tolerance': 2, 'lane': 0}, {'position': (59.42, 5.904), 'tolerance': 2, 'lane': 0}, {'position': (57.703, 6.084), 'tolerance': 2, 'lane': 0}, {'position': (56.105, 6.268), 'tolerance': 2, 'lane': 0}, {'position': (53.242, 6.597), 'tolerance': 2, 'lane': 0}, {'position': (50.825, 6.874), 'tolerance': 2, 'lane': 0}, {'position': (48.852, 7.099), 'tolerance': 2, 'lane': 0}, {'position': (47.325, 7.271), 'tolerance': 2, 'lane': 0}, {'position': (45.607, 7.459), 'tolerance': 2, 'lane': 0}, {'position': (43.833, 7.483), 'tolerance': 2, 'lane': 0}, {'position': (40.745, 7.489), 'tolerance': 2, 'lane': 0}, {'position': (38.594, 7.491), 'tolerance': 2, 'lane': 0}, {'position': (36.037, 7.493), 'tolerance': 2, 'lane': 0}, {'position': (33.075, 7.494), 'tolerance': 2, 'lane': 0}, {'position': (29.708, 7.496), 'tolerance': 2, 'lane': 0}, {'position': (25.936, 7.497), 'tolerance': 2, 'lane': 0}, {'position': (21.759, 7.498), 'tolerance': 2, 'lane': 0}, {'position': (17.177, 7.499), 'tolerance': 2, 'lane': 0}, {'position': (12.19, 7.5), 'tolerance': 2, 'lane': 0}, {'position': (6.797, 7.5), 'tolerance': 2, 'lane': 0}, {'position': (1.0, 7.5), 'tolerance': 2, 'lane': 0}, {'position': (80.649, 72.836), 'tolerance': 2, 'lane': 5}, {'position': (80.604, 71.3), 'tolerance': 2, 'lane': 5}, {'position': (82.347, 68.569), 'tolerance': 2, 'lane': 5}, {'position': (85.138, 64.561), 'tolerance': 2, 'lane': 5}, {'position': (89.381, 58.47), 'tolerance': 2, 'lane': 5}, {'position': (95.076, 50.296), 'tolerance': 2, 'lane': 5}, {'position': (115.168, 186.354), 'tolerance': 2, 'lane': 10}, {'position': (113.967, 185.449), 'tolerance': 2, 'lane': 10}, {'position': (113.016, 183.859), 'tolerance': 2, 'lane': 10}, {'position': (110.415, 179.461), 'tolerance': 2, 'lane': 10}, {'position': (106.219, 172.362), 'tolerance': 2, 'lane': 10}, {'position': (102.769, 166.521), 'tolerance': 2, 'lane': 10}, {'position': (101.023, 163.56), 'tolerance': 2, 'lane': 10}, {'position': (100.508, 162.149), 'tolerance': 2, 'lane': 10}, {'position': (100.64, 160.056), 'tolerance': 2, 'lane': 10}, {'position': (101.24, 151.885), 'tolerance': 2, 'lane': 10}, {'position': (102.318, 137.25), 'tolerance': 2, 'lane': 10}, {'position': (175.361, 253.323), 'tolerance': 2, 'lane': 14}, {'position': (174.019, 252.509), 'tolerance': 2, 'lane': 14}, {'position': (172.677, 251.695), 'tolerance': 2, 'lane': 14}, {'position': (171.334, 250.881), 'tolerance': 2, 'lane': 14}, {'position': (169.992, 250.066), 'tolerance': 2, 'lane': 14}, {'position': (168.65, 249.252), 'tolerance': 2, 'lane': 14}, {'position': (167.307, 248.438), 'tolerance': 2, 'lane': 14}, {'position': (165.965, 247.624), 'tolerance': 2, 'lane': 14}, {'position': (164.623, 246.809), 'tolerance': 2, 'lane': 14}, {'position': (163.28, 245.995), 'tolerance': 2, 'lane': 14}, {'position': (161.938, 245.181), 'tolerance': 2, 'lane': 14}], 'model': 'ETK800', 'color': 'Gray'}], 'parked_color': (0.72, 0.95, 0.17, 1.06)}
        if random() <= self.mutation_probability:
            child = self._mutate_parked_cars(child) # TODO Parked cars
        if random() <= self.mutation_probability:
            child = self._mutate_traffic(child) # TODO Traffic
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
        probability_attr = 1 / 3    # Because I mutate kind of obstacle, zRot and position.
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
        return individual

    def _mutate_traffic(self, individual):
        return individual

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
            self.population_list = [{'lanes': [{'control_points': [(1, 0), (30, 0), (45, 0), (59, 20), (80, 7)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {'control_points': [(80, 7), (97.00530293375724, -3.5270922923259054)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(97.00530293375724, -3.5270922923259054), (122.51325733439307, -19.317730730814766)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(97.00530293375724, -3.5270922923259054), (135.7301689376841, 28.101767203044915)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(97.00530293375724, -3.5270922923259054), (65.70267781959791, -42.51613768923711)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.70267781959791, -42.51613768923711), (75, -58), (93, -66), (103, -81), (127, -97)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}, {'control_points': [(127, -97), (143.64100588675686, -108.09400392450459)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(143.64100588675686, -108.09400392450459), (173.51531322081013, -68.0000422680341)], 'width': 20, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(143.64100588675686, -108.09400392450459), (168.6025147168922, -124.73500981126145)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(168.6025147168922, -124.73500981126145), (189, -127), (207, -114)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}], 'file_name': 'urban', 'obstacles': [{'name': 'trafficlightsingle', 'position': (85.15544100608406, -5.247462105791022), 'zRot': 328, 'mode': 'off', 'sign': 'yield', 'oid': None, 'intersection_id': 0, 'facingEgo': True, 'lane_id': 1}, {'name': 'trafficlightdouble', 'position': (97.7043496091133, 7.631372375614463), 'zRot': 219, 'mode': 'off', 'sign': 'priority', 'intersection_id': 0}, {'name': 'trafficlightsingle', 'position': (110.18157849026346, -2.6278356776622163), 'zRot': 508, 'intersection_id': 0, 'mode': 'off', 'sign': 'yield'}, {'name': 'trafficlightdouble', 'position': (98.64150736149844, -14.587057711378534), 'zRot': 411, 'mode': 'off', 'sign': 'priority', 'intersection_id': 0}, {'name': 'trafficlightdouble', 'position': (129.4739628751645, -108.50448206971126), 'zRot': 326, 'mode': 'manual', 'sign': 'yield', 'oid': 'traffic_light_manual_0', 'intersection_id': 1, 'facingEgo': True, 'lane_id': 6}, {'name': 'trafficlightdouble', 'position': (140.3014754969535, -95.5044234400095), 'zRot': 233, 'mode': 'off', 'sign': 'priority', 'intersection_id': 1}, {'name': 'trafficlightdouble', 'position': (157.0508831305018, -107.17874860073296), 'zRot': 506, 'intersection_id': 1, 'mode': 'off', 'sign': 'yield'}], 'success_point': {'position': (207, -114), 'tolerance': 3}, 'ego_lanes': [0, 1, 4, 5, 6, 8, 9], 'directions': ['right', 'straight'], 'fitness': 0, 'triggers': [{'triggerPoint': {'position': (127, -97), 'action': 'switchLights', 'tolerance': 2, 'triggeredBy': 'ego', 'triggers': 'traffic_light_manual_0', 'initState': 'green', 'switchTo': 'red'}}], 'tod': 0.6760646179165416, 'intersection_lanes': [[1, 2, 3, 4], [6, 7, 8]]}, {'lanes': [{'control_points': [(1, 0), (30, 0), (45, 0), (71, -3), (100, 3)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {'control_points': [(100, 3), (119.58520839014204, 7.052112080719045)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(119.58520839014204, 7.052112080719045), (148.96302097535516, 13.130280201797614)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(119.58520839014204, 7.052112080719045), (112.35821849061614, -42.42283730775581)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(119.58520839014204, 7.052112080719045), (97.1275467815048, 51.72484912549774)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(97.1275467815048, 51.72484912549774), (83, 72), (89, 90)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {'control_points': [(89, 90), (95.32455532033674, 108.97366596101028)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(95.32455532033674, 108.97366596101028), (46.000937629859436, 117.17005162321743)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(95.32455532033674, 108.97366596101028), (145.32311463552404, 108.59410515468579)], 'width': 15, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(95.32455532033674, 108.97366596101028), (104.81138830084188, 137.4341649025257)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(104.81138830084188, 137.4341649025257), (103, 162), (116, 184), (132, 204)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {'control_points': [(132, 204), (144.49390095108848, 219.6173761888606)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(144.49390095108848, 219.6173761888606), (108.32132060048988, 254.1361309575293)], 'width': 20, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(144.49390095108848, 219.6173761888606), (163.23475237772118, 243.04344047215147)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(163.23475237772118, 243.04344047215147), (178, 252)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}], 'file_name': 'urban', 'obstacles': [{'name': 'stopsign', 'position': (112.94177063380319, 0.3674778515595194), 'zRot': 372, 'intersection_id': 2, 'facingEgo': True, 'lane_id': 1}, {'name': 'prioritysign', 'position': (111.70084893351957, 11.15832395527785), 'zRot': 297, 'intersection_id': 2}, {'name': 'stopsign', 'position': (123.52588738864132, 13.177554842739335), 'zRot': 192, 'intersection_id': 2}, {'name': 'prioritysign', 'position': (123.99345015679178, 1.2540602935439065), 'zRot': 442, 'intersection_id': 2}, {'name': 'stopsign', 'position': (97.85437744847145, 100.11928851253883), 'zRot': 432, 'intersection_id': 3, 'facingEgo': True, 'lane_id': 6}, {'name': 'prioritysign', 'position': (87.71929469336148, 102.43188403283662), 'zRot': 351, 'intersection_id': 3}, {'name': 'stopsign', 'position': (92.79473319220205, 117.82804340948174), 'zRot': 252, 'intersection_id': 3}, {'name': 'prioritysign', 'position': (102.98278870041919, 116.61575085298779), 'zRot': 540, 'intersection_id': 3}, {'name': 'stopsign', 'position': (142.24499877989257, 208.48218696620296), 'zRot': 411, 'intersection_id': 4, 'facingEgo': True, 'lane_id': 11}, {'name': 'prioritysign', 'position': (133.76247178251901, 215.75908278374266), 'zRot': 316, 'intersection_id': 4}, {'name': 'stopsign', 'position': (147.2488061108035, 231.3850691471671), 'zRot': 231, 'intersection_id': 4}], 'success_point': {'position': (178, 252), 'tolerance': 3}, 'ego_lanes': [0, 1, 4, 5, 6, 9, 10, 11, 13, 14], 'directions': ['left', 'straight', 'straight'], 'fitness': 0, 'triggers': [], 'tod': 0.10629639474288721, 'intersection_lanes': [[1, 2, 3, 4], [6, 7, 8, 9], [11, 12, 13]]}, {'lanes': [{'control_points': [(1, 0), (30, 0), (45, 0)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}, {'control_points': [(45, 0), (65.0, 0.0)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.0, 0.0), (76.24755271719326, 48.71850323926175)], 'width': 20, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(65.0, 0.0), (95.0, 0.0)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(95.0, 0.0), (98, -16), (116, -39), (119, -68), (107, -95), (84, -111), (57, -121)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}, {'control_points': [(57, -121), (38.24502478552593, -127.94628711647186)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(38.24502478552593, -127.94628711647186), (10.11256196381484, -138.3657177911797)], 'width': 16, 'left_lanes': 2, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(38.24502478552593, -127.94628711647186), (6.5603593666849065, -89.26706818309498)], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(38.24502478552593, -127.94628711647186), (70.59990992585747, -166.06664135873928)], 'width': 15, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(70.59990992585747, -166.06664135873928), (62, -187), (69, -214)], 'width': 15, 'left_lanes': 1, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}], 'file_name': 'urban', 'obstacles': [{'name': 'trafficlightdouble', 'position': (54.9, -8.2), 'zRot': 360, 'mode': 'off', 'sign': 'yield', 'oid': None, 'intersection_id': 5, 'facingEgo': True, 'lane_id': 1}, {'name': 'trafficlightdouble', 'position': (56.883528879375916, 10.186898279067824), 'zRot': 257, 'mode': 'off', 'sign': 'priority', 'intersection_id': 5}, {'name': 'trafficlightdouble', 'position': (76.95, 8.2), 'zRot': 180, 'intersection_id': 5, 'mode': 'off', 'sign': 'yield'}, {'name': 'trafficlightdouble', 'position': (42.52393764927262, -117.61715817427819), 'zRot': 200, 'mode': 'off', 'sign': 'yield', 'oid': None, 'intersection_id': 6, 'facingEgo': True, 'lane_id': 5}, {'name': 'trafficlightdouble', 'position': (51.14304039131511, -131.2433757462813), 'zRot': 490, 'mode': 'off', 'sign': 'priority', 'intersection_id': 6}, {'name': 'trafficlightdouble', 'position': (33.96611192177923, -138.27541605866554), 'zRot': 380, 'intersection_id': 6, 'mode': 'off', 'sign': 'yield'}, {'name': 'trafficlightdouble', 'position': (25.48889587090261, -124.5251652078707), 'zRot': 309, 'mode': 'off', 'sign': 'priority', 'intersection_id': 6}], 'success_point': {'position': (69, -214), 'tolerance': 3}, 'ego_lanes': [0, 1, 3, 4, 5, 8, 9], 'directions': ['straight', 'left'], 'fitness': 0, 'triggers': [], 'tod': 0.6091846761850038, 'intersection_lanes': [[1, 2, 3], [5, 6, 7, 8]]}]

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

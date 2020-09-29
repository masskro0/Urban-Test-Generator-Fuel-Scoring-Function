from copy import deepcopy
from random import randint, random, choice, uniform

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
    calc_min_max_angles, get_roads_of_intersection, get_intersection_lines, get_width_lines, \
    get_resize_factor_intersection, multilinestrings_to_linestring, calc_speed_waypoints
from utils.validity_checks import intersection_check_width, intersection_check_last
from utils.xml_creator import build_all_xml
from xml_converter.bng_converter import b_spline

MIN_DEGREES = 90
MAX_DEGREES = 270
OID_INDEX = 0
INTERSECTION_ID = 0
COLORS = ["White", "Red", "Green", "Yellow", "Black", "Blue", "Orange", "Gray", "Purple"]
PARTICIPANTS_SAMPLES = 45


def _add_parked_cars(roads):
    """Adds parked cars to the test case.
    :param roads: List of roads. One road is a dict type, which must contain the keys "control_points"
     (list of points), "type" (intersection or road), width (int).
    :return: List of parked cars, color of the first car.
    """
    car_positions = list()
    for idx, road in enumerate(roads):
        control_points = road.get("control_points")
        if road.get("type") == "intersection" or control_points[0] == control_points[-1]:
            continue
        width = road.get("width")
        rotations = [0, 45, 90]         # One of three alignments.
        rotation = choice(rotations)
        noise = [x / 10 for x in range(-10, 10)]    # Noise for rotation.
        if rotation == 45:
            offset = 3.5        # Offset to the road border.
            min_distance = 4    # Minimum distance between two parked cars.
        elif rotation == 90:
            offset = 3
            min_distance = 4.5
        else:
            offset = 2
            min_distance = 5.5
        right = True if random() >= 0.3 else False      # Parked cars on the right side?
        left = True if random() >= 0.3 else False       # Parked cars on the left side?
        line = LineString(control_points)
        line = multilinestrings_to_linestring(line)     # Shapely bugfix.
        prev_road = LineString(roads[idx - 1].get("control_points")) if idx != 0 else None      # Previous road.
        prev_width = int(roads[idx - 1].get("width")) / 2 + offset if idx != 0 else 0
        if left:
            left_lines = [line.parallel_offset(width / 2 + offset + x, "left") for x in noise]  # Adds noise to offset.
            left_lines = [multilinestrings_to_linestring(x) for x in left_lines]
            iterator = 1
            while iterator < len(left_lines[0].coords):
                left_line = choice(left_lines)
                coords = b_spline(left_line.coords)
                point = coords[iterator]
                if abs(euclidean(point, coords[-1])) < 12:
                    # Parked cars must be at least 12 meters away from intersections.
                    break
                if len(car_positions) == 0 or (abs(euclidean(point, car_positions[-1][0])) > min_distance and
                                               (prev_road is None or Point(point).distance(prev_road) > prev_width)):
                    angle = get_angle((coords[iterator - 1][0] + 5, coords[iterator - 1][1]),
                                      coords[iterator - 1], point) - rotation + randint(-8, 8)  # Angle with noise.
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
                if len(car_positions) == 0 or (abs(euclidean(point, car_positions[-1][0])) > min_distance and
                                               (prev_road is None or Point(point).distance(prev_road) > prev_width)):
                    angle = get_angle((coords[iterator - 1][0] + 5, coords[iterator - 1][1]),
                                      coords[iterator - 1], point) + 180 - rotation + randint(-8, 8)
                    car_positions.append((point, angle, idx))
                iterator += 1
    parked_cars = list()
    color = (round(uniform(0, 1), 2), round(uniform(0, 1), 2), round(uniform(0, 1), 2), round(uniform(1, 1.3), 2))
    for position in car_positions:
        if random() <= 0.4:
            # Discard with probability of 40% the car to create gaps.
            continue
        parked_cars.append({"name": "parkedCar", "position": (position[0][0], position[0][1]), "zRot": position[1],
                            "color": color, "road": position[2]})
    return parked_cars, color


def _add_ego_car(roads, ego_roads, directions, actions, ego_waypoints=True):
    """Adds the ego car to the test case. If needed, waypoints can be added as well.
    :param roads: List of dicts with the following keys: "left_lanes" (int), "right_lanes" (int), "control_points":
     list of 2D points, "width" of the road (int).
    :param ego_roads: List of road IDs (int) which the ego-car must travserse.
    :param directions: List of directions (Strings) where the ego-car must turn at an intersection.
    :param actions: List of actions (Strings) whether the ego-car must stop at an intersection or not.
    :ego_waypoints: {@code True} if waypoints should be added.
    :return: Dict with the keys "id" ("ego"), "init_state" (initial state with init position and orientation),
     "waypoints" (list of dicts with positino, tolerance, road ID), "model" (car model as String), "color" (String).
    """
    samples = 100       # Samples to interpolate the road for the waypoints.
    waypoints = list()  # List containing waypoints.
    if ego_waypoints:
        lines = list()  # LineStrings representing the path.
        ego_index = 0
        for idx, road in enumerate(ego_roads):
            temp_points = roads[road].get("control_points")
            temp_points = LineString(temp_points)
            if temp_points.coords[0] == temp_points.coords[-1]:
                continue
            left_lanes = roads[road].get("left_lanes")
            right_lanes = roads[road].get("right_lanes")
            width = roads[road].get("width")
            width_per_lane = width / (left_lanes + right_lanes)
            left = False
            if idx + 1 < len(ego_roads) and ego_roads[idx + 1] - ego_roads[idx] != 1:
                if directions[ego_index] == "left" and right_lanes > 1:
                    left = True
                ego_index += 1
            # Calculate parallel linestring so the ego-car drives on the right lane.
            if left:
                offset = (right_lanes - left_lanes - 1) * width_per_lane / 2
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
        road_id = 0
        action_index = 0
        for idx, _ in enumerate(lines):
            control_points = list(lines[idx].coords)
            opposite_dir = False
            deleted_points = list()
            lane_change = False
            stop_flag = False
            if idx != 0 and ego_roads[idx] - ego_roads[idx - 1] != 1:
                if actions[action_index] == "stop":
                    road_id += 1
                    stop_flag = True
                action_index += 1
                opposite_dir = True
            if idx + 1 < len(ego_roads) and ego_roads[idx + 1] - ego_roads[idx] != 1:
                intersec_point = lines[idx].intersection(lines[idx + 1])
                if isinstance(intersec_point, MultiPoint):
                    intersec_point = Point((intersec_point[0]))
                lane_change = True

                # Delete half of the points for right turns.
                index = len(control_points) // 2
                deleted_points = control_points[index:]
                control_points = control_points[:index]
                if directions[ego_index] == "right":
                    control_points.append((intersec_point.x, intersec_point.y))
                ego_index += 1
            # Create waypoints.
            i = 0
            while i < len(control_points):
                if len(waypoints) == 0 or (euclidean(control_points[i], waypoints[-1].get("position")) >= 1.5
                                           and (not opposite_dir
                                           or euclidean(control_points[0], control_points[i]) > 4)):
                    waypoint = {"position": control_points[i],
                                "tolerance": 2,
                                "road": road_id}
                    waypoints.append(waypoint)
                i += 1
            del waypoints[-1]
            if lane_change:
                # Left turns need more waypoints because the car needs to traverse the whole intersection.
                i = 0
                while i < len(deleted_points):
                    if len(waypoints) == 0 \
                            or euclidean(deleted_points[i], waypoints[-1].get("position")) >= 1.5:
                        waypoint = {"position": deleted_points[i],
                                    "tolerance": 2,
                                    "road": road_id + 1 if stop_flag else road_id}
                        waypoints.append(waypoint)
                    i += 1
                del waypoints[-1]

    # Calculate spawn position of the ego-car.
    first_road = LineString(roads[0].get("control_points"))
    left_lanes = (roads[0].get("left_lanes"))
    right_lanes = (roads[0].get("right_lanes"))
    width = roads[0].get("width")
    width_per_lane = width / (left_lanes + right_lanes)
    offset = (left_lanes + right_lanes - 1) * width_per_lane / 2
    first_point = first_road.parallel_offset(offset, "right").coords[::-1][0]
    first_point = (first_point[0] + 2, first_point[1])

    init_state = {"position": first_point, "orientation": 0}
    model = "ETK800"
    ego = {"id": "ego",
           "init_state": init_state,
           "waypoints": waypoints,
           "model": model,
           "color": "White"}
    return ego


def _add_other_0(roads, ego_roads, actions):
    """Adds the first type of NPC. This one always spawns and stays at intersections and never follows the ego-car's
     route.
    :param roads: List of dicts with the following keys: "left_lanes" (int), "right_lanes" (int), "control_points":
     list of 2D points, "width" of the road (int).
    :param ego_roads: List of road IDs (int) which the ego-car must traverse.
    :param actions: List of actions (Strings) whether the ego-car must stop at an intersection or not.
    :return: Dict with the keys "id" ("ego"), "init_state" (initial state with init position and orientation),
     "waypoints" (list of dicts with positino, tolerance, road ID), "model" (car model as String), "color" (String),
     "spawn_roads" (list of road IDs (Int) where the car should spawn), "end_roads" (list of road IDs (Int) where the
     path of the car ends); list of possible spawn roads, list of trigger points to spawn and start the vehicle.
    """
    global PARTICIPANTS_SAMPLES

    # Drive from one opposite road to another at an intersection.
    # Get roads where a car can spawn, be teleported to or drive to.
    spawn_roads = list()
    triggers = list()
    for idx, _ in enumerate(roads):
        if idx not in ego_roads:
            # Append all possible spawn road IDs.
            spawn_roads.append(idx)
    i = 0
    j = 0
    action_index = 0
    waypoints = list()
    spawns = list()     # List of spawn roads.
    ends = list()       # List of end roads.
    while i < len(spawn_roads):
        if actions[action_index] == "go":
            # Skip everything, if the ego-car doesn't stop at the intersection.
            action_index += 1
            if i < len(spawn_roads) - 1 and spawn_roads[i + 1] - spawn_roads[i] == 1:
                i += 2
            else:
                i += 1
            continue
        lines = list()
        three_way = False
        if len(spawn_roads) > 1 and i < len(spawn_roads) - 1 and spawn_roads[i + 1] - spawn_roads[i] == 1:
            # Four-way intersection? Choose random spawn and end road.
            spawn_indices = [spawn_roads[i], spawn_roads[i] + 1, spawn_roads[i] + 2]
            spawn_index = choice(spawn_indices)
            end_indices = [spawn_roads[i] - 1, spawn_roads[i], spawn_roads[i] + 1]
            end_index = choice(end_indices)
            while end_index == spawn_index:
                end_index = choice(end_indices)
        else:
            # Three-way intersection.
            spawn_indices = [spawn_roads[i], spawn_roads[i] + 1]
            spawn_index = choice(spawn_indices)
            end_index = spawn_roads[i] - 1 if spawn_index == spawn_roads[i] else spawn_roads[i]
            end_index = end_index
            three_way = True
        j += 1
        spawns.append(spawn_index)
        ends.append(end_index)

        # Create path, it consists of three points: spawn point, end of the spawn road, end point.
        spawn_point = roads[spawn_index].get("control_points")[0] if three_way and spawn_index == spawn_roads[i] + 1 \
            else roads[spawn_index].get("control_points")[0]
        end_point = roads[end_index].get("control_points")[-1] if end_index != spawn_roads[i] - 1 \
            else roads[end_index].get("control_points")[0]
        middle_point = roads[spawn_index].get("control_points")[-1]
        orientation = get_angle((spawn_point[0] + 1, spawn_point[1]), spawn_point,
                                roads[spawn_index].get("control_points")[1]) + 180

        # Get path of the intersection road from where the ego-car will come from to determine the trigger point.
        road = roads[spawn_roads[i] - 1] if spawn_index != spawn_roads[i] + 2 else roads[spawn_roads[i] - 2]
        points = road.get("control_points")
        width_per_lane = road.get("width") / (road.get("left_lanes") + road.get("right_lanes"))
        offset = (road.get("left_lanes") + road.get("right_lanes") - 1) * width_per_lane / 2
        trigger_road = LineString(points)
        trigger_road = trigger_road.parallel_offset(offset, "right")
        trigger_road = multilinestrings_to_linestring(trigger_road)

        # Reversed because the car spawns from the opposite direction.
        left_lanes = roads[spawn_index].get("right_lanes")
        right_lanes = roads[spawn_index].get("left_lanes")
        width = roads[spawn_index].get("width")
        points = roads[spawn_index].get("control_points")
        points = points[::-1]
        line = LineString(points)
        width_per_lane = width / (left_lanes + right_lanes)
        angle = get_angle(spawn_point, middle_point, end_point)
        left = True if (240 <= angle <= 300) and right_lanes > 1 else False

        # Get path between spawn point and middle point.
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

        # Remove points because otherwise the car will drive to the end of the intersection and then drive reverse.
        if left:
            line.coords = line.coords[:-4]
        else:
            line.coords = line.coords[:-9]
        lines.append(line)

        # Get path between middle point and end point.
        left_lanes = roads[end_index].get("left_lanes")
        right_lanes = roads[end_index].get("right_lanes")
        width = roads[end_index].get("width")
        points = roads[end_index].get("control_points")
        line = LineString(points)
        width_per_lane = width / (left_lanes + right_lanes)
        offset = (left_lanes + right_lanes - 1) * width_per_lane / 2
        if end_index != spawn_roads[i] - 1:
            line = line.parallel_offset(offset, "right")
        else:
            line = line.parallel_offset(offset, "left")
        line = multilinestrings_to_linestring(line)
        line.coords = line.coords[::-1]
        line.coords = b_spline(list(line.coords), PARTICIPANTS_SAMPLES).tolist()

        # Remove points so the waypoints won't begin at the center point of the intersection.
        line.coords = line.coords[PARTICIPANTS_SAMPLES // 10:]
        lines.append(line)

        # Add waypoints.
        for line in lines:
            for point in list(line.coords):
                if len(waypoints) == 0 or euclidean(point, waypoints[-1].get("position")) >= 1.5:
                    waypoint = {"position": point,
                                "tolerance": 2,
                                "road": spawn_index}
                    waypoints.append(waypoint)

        # Add trigger point and spawn point.
        trigger_point = {"position": trigger_road.coords[-1],
                         "action": "spawnAndStart",
                         "tolerance": 2,
                         "triggeredBy": "ego",
                         "triggers": "other_0"}
        spawn_point = {"position": list(lines[0].coords)[0], "orientation": orientation}
        triggers.append({"triggerPoint": trigger_point, "spawnPoint": spawn_point})

        if i < len(spawn_roads) - 1 and spawn_roads[i + 1] - spawn_roads[i] == 1:
            # Four-way intersection?
            i += 2
        else:
            i += 1
        action_index += 1
    if len(waypoints) != 0:
        init_state = {"position": waypoints[0].get("position"),
                      "orientation": triggers[0].get("spawnPoint").get("orientation")}
        other = {"id": "other_0",
                 "init_state": init_state,
                 "waypoints": waypoints,
                 "model": "ETK800",
                 "color": choice(COLORS),
                 "spawn_roads": spawns,
                 "end_roads": ends}
        return other, spawn_roads, triggers
    else:
        return None, spawn_roads, list()


def _add_other_1(roads, ego_roads, participants, end_roads):
    """Currently under development. This participant should follow the ego-car from the start and has an arbitrary
     end point. This participant doesn't follow the traffic rules at the moment.
    :param roads: List of dicts with the following keys: "left_lanes" (int), "right_lanes" (int), "control_points":
     list of 2D points, "width" of the road (int).
    :param ego_roads: List of road IDs (int) which the ego-car must traverse.
    :param participants: List of participants which were previously added.
    :param end_roads: List of possible roads where this participant's path ends.
    :return: Dict with the keys "id" ("ego"), "init_state" (initial state with init position and orientation),
     "waypoints" (list of dicts with positino, tolerance, road ID), "model" (car model as String), "color" (String),
     "end_roads" (list of road IDs (Int) where the path of the car ends).
    """
    global COLORS
    global PARTICIPANTS_SAMPLES
    end_roads.append(ego_roads[-1])     # Ego-cars success point is also an end point.
    end_index = choice(end_roads)       # Choose random end point.
    temp_val = 0
    road_id = 0
    i = 1
    while i < len(end_roads):
        if end_roads[i - 1] - end_roads[i] != 1:
            temp_val += 1
        if end_index == end_roads[i]:
            road_id = temp_val
            break
        i += 1
    waypoints = list()
    ego_waypoints = None
    for participant in participants:
        if participant.get("id") == "ego":
            ego_waypoints = participant.get("waypoints")

    # Spawn in front of the ego-car.
    for waypoint in ego_waypoints[4:]:
        if waypoint.get("road") > road_id:
            break
        waypoints.append(waypoint)
    # Create path for the last road.
    left_lanes = roads[end_index].get("left_lanes")
    right_lanes = roads[end_index].get("right_lanes")
    width = roads[end_index].get("width")
    points = roads[end_index].get("control_points")
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
                        "road": road_id}
            waypoints.append(waypoint)
    init_state = {"position": ego_waypoints[4].get("position"),
                  "orientation": 0}
    other = {"id": "other_{}".format(1),
             "init_state": init_state,
             "waypoints": waypoints,
             "model": "ETK800",
             "color": choice(COLORS),
             "end_road": end_index}
    return other


def _add_other_2(roads, ego_roads, triggers=None):
    """This participant type always drives between two intersections and passes by the ego-car. This car spawns as
     soon as the ego-car leaves an intersection.
    :param roads: List of dicts with the following keys: "left_lanes" (int), "right_lanes" (int), "control_points":
     list of 2D points, "width" of the road (int).
    :param ego_roads: List of road IDs (int) which the ego-car must traverse.
    :param triggers: List of trigger points.
    :return: Dict with the keys "id" ("ego"), "init_state" (initial state with init position and orientation),
     "waypoints" (list of dicts with positino, tolerance, road ID), "model" (car model as String), "color" (String).
    """
    if triggers is None:
        triggers = list()
    global COLORS
    global PARTICIPANTS_SAMPLES
    spawn_roads = [0]           # Possible spawn positions.
    i = 1
    while i < len(ego_roads):
        if i == len(ego_roads) - 1 or (ego_roads[i + 1] - ego_roads[i] == 1 and ego_roads[i] - ego_roads[i - 1] == 1):
            spawn_roads.append(ego_roads[i])
        i += 1
    waypoints = list()
    for idx in spawn_roads:
        # Create paths (polylines) between two intersections.
        left_lanes = roads[idx].get("left_lanes")
        right_lanes = roads[idx].get("right_lanes")
        width = roads[idx].get("width")
        points = roads[idx].get("control_points")
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
                            "road": idx}
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
    init_state = {"position": triggers[-len(spawn_roads)].get("spawnPoint").get("position"),
                  "orientation": triggers[-len(spawn_roads)].get("spawnPoint").get("orientation")}
    other = {"id": "other_{}".format(2),
             "init_state": init_state,
             "waypoints": waypoints,
             "model": "ETK800",
             "color": choice(COLORS)}
    return other


def _add_other_participants(roads, ego_roads, actions):
    """Adds two implemented participant types to the test case.
    :param roads: List of dicts with the following keys: "left_lanes" (int), "right_lanes" (int), "control_points":
     list of 2D points, "width" of the road (int).
    :param ego_roads: List of road IDs (int) which the ego-car must traverse.
    :param actions: List of actions (Strings) whether the ego-car must stop at an intersection or not.
    :return: List of participants and trigger points.
    """
    participants = list()
    other, sr, triggers = _add_other_0(roads, ego_roads, actions)
    participants.append(other)
    # other = _add_other_1(roads, ego_roads, individual.get("participants"), sr)
    # participants.append(other)
    other = _add_other_2(roads, ego_roads, triggers)
    participants.append(other)
    return participants, triggers


def _merge_roads(population):
    """Merge roads for each individual of a population which will be driven by the ego car. Individual must contain
     the keys "roads" (List of roads with the key "control_points" (list of 2D points)) and "ego_roads" (list with road
     IDs (Int)).
    :param population: Population list.
    :return: Population with merged roads.
    """
    for individual in population:
        iterator = 1
        roads = individual.get("roads")
        ego_roads = individual.get("ego_roads")
        new_road_list = [roads[0]]
        while iterator < len(roads):
            if iterator in ego_roads and (iterator - 1) in ego_roads:
                new_road_list[-1].get("control_points").extend(roads[iterator].get("control_points"))
            else:
                new_road_list.append(roads[iterator])
            iterator += 1
        individual["roads"] = new_road_list
    return population


def _handle_manual_mode(trigger_pos, oid, ego_waypoints):
    """Creates trigger points for manual traffic lights and stopping the ego-car.
    :param trigger_pos: Position of the trigger point.
    :param oid: Object ID (Int).
    :param ego_waypoints: {@code True} if the ego-car has waypoints or not.
    :return: List of trigger points for traffic light switch and stopping the ego-car.
    """
    triggers = list()
    init_state = choice(["green", "red"])
    trigger_point = {"position": trigger_pos,
                     "action": "switchLights",
                     "tolerance": 2,
                     "triggeredBy": "ego",
                     "triggers": oid,
                     "initState": init_state,
                     "switchTo": "green" if init_state == "red" else "red"}
    triggers.append({"triggerPoint": trigger_point})
    if init_state == "green" and ego_waypoints:
        triggers.append({"triggerPoint": {"position": trigger_pos, "action": "stop", "tolerance": 2,
                                          "triggeredBy": "ego", "duration": 4}})
    return triggers


def _add_traffic_lights_and_signs(last_point, current_left_lanes, current_right_lanes, width, intersection, road_id,
                                  ego_waypoints):
    """Adds traffic lights and signs to the test case.
    :param last_point: Last generated point (tuple).
    :param current_left_lanes: Number of left lanes (Int).
    :param current_right_lanes: Number of right lanse (Int).
    :param width: Width of the road (Int).
    :param intersection: Intersection dict object generated after calling self._create_intersection.
    :param road_id: Road ID (Int).
    :param ego_waypoints: {@code True} if the ego-car has waypoints or not.
    :return: List of obstacles (dict with name, position, rotation, intersection ID, pole sign, traffic light mode,
     flag whether the ego-car sees the traffic light or sign or not), list of trigger points for traffic light switch
     and stopping the ego-car, list of actions at the intersection ("go" or "stop").
    """
    global OID_INDEX
    global INTERSECTION_ID
    intersection_point = intersection.get("intersection_point")
    new_left_lanes = intersection.get("new_left_lanes")             # Number of left lanes of opposite road.
    new_right_lanes = intersection.get("new_right_lanes")
    left_point = intersection.get("left_point")
    straight_point = intersection.get("straight_point")
    right_point = intersection.get("right_point")
    layout = intersection.get("layout")
    number_of_ways = intersection.get("number_of_ways")
    direction = intersection.get("direction")

    def opposite_direction(my_point, my_right_point, num_lanes):
        """Adds traffic lights and traffic signs to the opposite roads.
        :param my_point: Point of the current direction.
        :param my_right_point: The corresponding right point.
        :param num_lanes: Number of lanes of the opposite road.
        :return: Void.
        """
        line = LineString([intersection_point, my_point])

        # Calculate rotation of the traffic sign/light.
        my_z_rot = int(round(get_angle(temp_point, line.coords[0], line.coords[1]))) + 180
        angle = int(round(get_angle(my_right_point, line.coords[0], line.coords[1])))

        # Increase offset which is dependent on the angle between the opposite and current road to avoid placing the
        # traffic sign/light on the road.
        offset = 0.1 if angle <= 270 else ((angle - 270) / 10 + 0.2) * 1.3

        # Create vectors to get the obstacle position. Resize factor is dependent on road width.
        fac = (old_width * (current_left_lanes + current_right_lanes) / 2 + offset) / line.length
        vector = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        vector = affinity.rotate(vector, -90, vector.coords[1])
        fac = (new_width * (new_left_lanes + new_right_lanes) / 2 + 0.2) / vector.length
        vector = affinity.scale(vector, xfact=fac, yfact=fac, origin=vector.coords[1])
        my_position = vector.coords[0]

        # Traffic lights on opposite roads have either the mode "off" or "flashing".
        my_mode = "off" if mode is not None and mode == "manual" else mode

        # Always add the opposite sign.
        if sign_on_my_road == "stopsign":
            obstacles.append({"name": "prioritysign", "position": my_position, "zRot": my_z_rot,
                              "intersection_id": INTERSECTION_ID})
        elif sign_on_my_road == "prioritysign":
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
        """Calculate position of the traffic light/sign on the current road.
        :param my_point: Point of the current intersection road.
        :param my_right_point: The corresponding right point.
        :return: Position and rotation of the obstacle.
        """
        # Same procedure as in method opposite_direction().
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
    old_width = width / (current_left_lanes + current_right_lanes)                  # Current road width per lane.
    new_width = intersection.get("new_width") / (new_left_lanes + new_right_lanes)  # Width per lane of opposite road.
    obstacles = list()
    temp_point = (intersection_point[0] + 5, intersection_point[1])

    # Bottom (ego-car) direction.
    position, z_rot = my_direction(last_point, right_point)
    pole_sign = "yield" if random() < 0.5 else "priority"
    if current_right_lanes == 1:
        if current_left_lanes == 1 and new_left_lanes == 1 and new_right_lanes == 1 and random() < 0.5:
            if random() < 0.5:
                obstacles.append({"name": "stopsign", "position": position, "zRot": z_rot,
                                  "intersection_id": INTERSECTION_ID, "facingEgo": True, "road_id": road_id})
            else:
                obstacles.append({"name": "prioritysign", "position": position, "zRot": z_rot,
                                  "intersection_id": INTERSECTION_ID, "facingEgo": True, "road_id": road_id})
        else:
            obstacles.append({"name": "trafficlightsingle", "position": position, "zRot": z_rot, "mode": mode,
                              "sign": pole_sign, "oid": oid, "intersection_id": INTERSECTION_ID, "facingEgo": True,
                              "road_id": road_id})
    else:
        obstacles.append({"name": "trafficlightdouble", "position": position, "zRot": z_rot, "mode": mode,
                          "sign": pole_sign, "oid": oid, "intersection_id": INTERSECTION_ID, "facingEgo": True,
                          "road_id": road_id})
    sign_on_my_road = obstacles[0].get("name")

    triggers = list()
    if sign_on_my_road.startswith("trafficlight") and mode == "manual":
        triggers = _handle_manual_mode(last_point, oid, ego_waypoints)

    if sign_on_my_road.startswith("trafficlight") and mode != "manual" and pole_sign == "yield":
        triggers.append({"triggerPoint": {"position": last_point, "action": "stop", "tolerance": 2,
                                          "triggeredBy": "ego", "duration": 4}})

    if sign_on_my_road.startswith("stop"):
        triggers.append({"triggerPoint": {"position": last_point, "action": "stop", "tolerance": 2,
                                          "triggeredBy": "ego", "duration": 4}})

    if sign_on_my_road.startswith("priority") or obstacles[-1].get("sign") == "priority" \
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
        if sign_on_my_road.startswith("trafficlight"):
            if current_left_lanes == 1:
                this_sign = "trafficlightsingle"
            else:
                this_sign = "trafficlightdouble"
        else:
            this_sign = sign_on_my_road
        position, z_rot = my_direction(straight_point, left_point)
        obstacles.append({"name": this_sign, "position": position,
                          "zRot": z_rot, "intersection_id": INTERSECTION_ID})
        if sign_on_my_road.startswith("trafficlight"):
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


def _preparation(population, traffic=True, ego_waypoints=True, add_parked_cars=True):
    """Adds parked cars, the ego-car and other traffic participants to the test case for a whole population.
    :param population: List of individuals after test case generation.
    :param traffic: {@code True} if other traffic participants should be added.
    :param ego_waypoints: {@code True} if the ego-car should get waypoints.
    :param add_parked_cars: {@code True} if parked cars should be added.
    :return: Void.
    """
    for individual in population:
        if add_parked_cars:
            parked_cars, color = _add_parked_cars(individual.get("roads"))
            individual["obstacles"].extend(parked_cars)
            individual["parked_color"] = color
        ego = _add_ego_car(individual.get("roads"), individual.get("ego_roads"), individual.get("directions"),
                           individual.get("actions"), ego_waypoints)
        individual.setdefault("participants", []).extend([ego])
        if traffic:
            participants, triggers = _add_other_participants(individual.get("roads"), individual.get("ego_roads"),
                                                             individual.get("actions"))
            individual.setdefault("participants", []).extend(participants)
            individual.setdefault("triggers", []).extend(triggers)
        if traffic or ego_waypoints:
            calc_speed_waypoints(individual["participants"])


class UrbanTestGenerator:
    """Procedural content generator to generate test cases in urban-like environments."""
    def __init__(self, files_name="urban", traffic=True, spline_degree=2, max_tries=20, population_size=1,
                 min_segment_length=10, max_segment_length=30, min_nodes=6, max_nodes=16, intersection_length=30,
                 opposite_road_length=30, straight_length=20, max_left_lanes=2, max_right_lanes=2, max_width=5,
                 ego_waypoints=True):
        self.FILES_NAME = files_name                    # File name for XML file series.
        self.TRAFFIC = traffic                          # Enable traffic or not.
        self.SPLINE_DEGREE = spline_degree              # Degree of the interpolation curve.
        self.MAX_TRIES = max_tries                      # Max number of consecutive invalid generated points/segments.
        self.POPULATION_SIZE = population_size          # Minimum number of generated roads for each generation.
        self.MIN_SEGMENT_LENGTH = min_segment_length    # Minimum length of a road segment.
        self.MAX_SEGMENT_LENGTH = max_segment_length    # Maximum length of a road segment.
        self.MIN_NODES = min_nodes                      # Minimum number of control points for each road.
        self.MAX_NODES = max_nodes                      # Maximum number of control points for each road.
        self.POPULATION = list()                        # List of individuals.
        self.INTERSECTION_LENGTH = intersection_length  # Distance between last generated point to intersection center.
        self.OPPOSITE_ROAD = opposite_road_length       # Length between the left and right road of an intersection.
        self.STRAIGHT_LENGTH = straight_length          # Length between intersection center and straight road segment.
        self.MAX_LEFT_LANES = max_left_lanes            # Maximum allowed number of left lanes.
        self.MAX_RIGHT_LANES = max_right_lanes          # Maximum allowed number of right lanes.
        self.MAX_WIDTH = max_width                      # Maximum allowed width per lane.
        self.EGO_WAYPOINTS = ego_waypoints              # Add waypoints for ego-car.

    def _bspline(self, roads):
        """Calculate samples of a b-spline interpolation curve. This is the road representation function.
        :param roads: List of roads.
        :return: List of arrays with samples, representing a bspline of the given control points of the roads.
        """
        splined_list = list()
        for road in roads:
            samples = road.get("samples")
            # Calculate splines for each road.
            point_list = asarray(road.get("control_points"))
            count = len(point_list)
            degree = clip(self.SPLINE_DEGREE, 1, count - 1)

            # Calculate knot vector.
            kv = concatenate(([0] * degree, arange(count - degree + 1), [count - degree] * degree))

            # Calculate query range.
            u = linspace(False, (count - degree), samples)

            # Calculate result.
            splined_list.append({"control_points": around(array(splev(u, (kv, point_list.T, degree))).T, 3),
                                 "width": road.get("width")})
        return splined_list

    def _add_segment(self, last_point, penultimate_point=None):
        """Generates a new random point.
        :param last_point: Last point of the control point list as tuple.
        :param penultimate_point: Point before the last point as tuple.
        :return: A new random point as tuple.
        """
        # Create valid area.
        x_min = int(round(last_point[0] - self.MAX_SEGMENT_LENGTH))
        x_max = int(round(last_point[0] + self.MAX_SEGMENT_LENGTH))
        y_min = int(round(last_point[1] - self.MAX_SEGMENT_LENGTH))
        y_max = int(round(last_point[1] + self.MAX_SEGMENT_LENGTH))
        while True:
            # Generate random point.
            x_pos = randint(x_min, x_max)
            y_pos = randint(y_min, y_max)
            point = (x_pos, y_pos)
            dist = Point(last_point).distance(Point(point))
            deg = None
            if penultimate_point is not None:
                deg = get_angle((penultimate_point[0], penultimate_point[1]), (last_point[0], last_point[1]), point)
            if self.MAX_SEGMENT_LENGTH >= dist >= self.MIN_SEGMENT_LENGTH:
                # New point within valid area?
                if penultimate_point is not None:
                    if MIN_DEGREES <= deg <= MAX_DEGREES:
                        return point
                else:
                    return point

    def _create_urban_environment(self):
        """Generates a test case in an urban-like scenario.
        :return: Test case. Dict type with the keys "roads" (List of roads), "success_point" (goal location of
         ego-car), "ego_roads" (list of road IDs which the ego-car must traverse (Ints)), "obstacles" (list of traffic
         lights and signs), "directions" (list of directions at intersections), "triggers" (trigger points for traffic
         light switches, stopping the ego-car, spawn and start other traffic participants), "tod" (time of day),
         "intersection_roads" (list of road IDs (Int) which are of type intersection, "actions" (list of actions ("go"
         and "stop")).
        """
        global MIN_DEGREES, MAX_DEGREES
        print(colored("Creating urban scenario...", "grey", attrs=['bold']))

        # Create first three points by myself.
        p0 = (1, 0)
        p1 = (30, 0)
        p2 = (45, 0)

        # Choose random road properties.
        left_lanes = randint(1, self.MAX_LEFT_LANES)
        right_lanes = randint(1, self.MAX_RIGHT_LANES)
        MIN_DEGREES, MAX_DEGREES = calc_min_max_angles(left_lanes + right_lanes)
        roads = [{"control_points": [p0, p1, p2], "width": calc_width(left_lanes, right_lanes),
                  "left_lanes": left_lanes, "right_lanes": right_lanes, "samples": 100, "type": "normal"}]
        ego_roads = [0]             # Roads which the ego-car must travserse.
        intersection_roads = list()
        obstacles = list()
        directions = list()
        triggers = list()
        actions = list()
        tries = 0
        road_index = 0                  # Current road ID.
        number_of_pieces = 3            # Points and intersections count as one piece.
        one_intersection = False        # {@code True} if at least one intersection was added to the test case.
        intersection_possible = True    # {@code True} if it's possible to add an intersection.
        intersection_probability = 0.25
        lines_of_roads = convert_points_to_lines(roads)
        last_point = p2
        while (number_of_pieces <= self.MAX_NODES and tries <= self.MAX_TRIES) \
                or len(roads[road_index].get("control_points")) == 1:
            control_points = roads[road_index].get("control_points")
            if intersection_possible and ((number_of_pieces == self.MAX_NODES - 1 and not one_intersection)
                                          or random() <= intersection_probability) and len(control_points) > 1:
                # Add intersection, if possible.
                intersection = self._create_intersection(control_points[-1], control_points[-2])
                intersection_items = get_roads_of_intersection(intersection, control_points[-1],
                                                               roads[road_index].get("width"),
                                                               roads[road_index].get("left_lanes"),
                                                               roads[road_index].get("right_lanes"), road_index)

                # Temporarily add new intersection to the road network to validate the network.
                new_line, new_road_line = get_intersection_lines(control_points[-1], intersection)
                temp_list = deepcopy(roads)
                temp_list.extend(intersection_items.get("roads"))
                temp_list = self._bspline(temp_list)
                polyline = convert_points_to_lines(temp_list)
                width_lines = get_width_lines(temp_list)
                intersection_roads_temp = deepcopy(intersection_roads)
                intersection_roads_temp.extend(intersection_items.get("intersection_roads"))
                if not intersection_check_last(lines_of_roads, new_line) \
                        and not intersection_check_last(lines_of_roads, new_road_line) \
                        and not intersection_check_width(width_lines, polyline, intersection_roads_temp):
                    left_lanes = intersection_items.get("left_lanes")
                    right_lanes = intersection_items.get("right_lanes")
                    obs, trs, action = _add_traffic_lights_and_signs(control_points[-1],
                                                                     roads[road_index].get("left_lanes"),
                                                                     roads[road_index].get("right_lanes"),
                                                                     roads[road_index].get("width"), intersection,
                                                                     road_index + 1, self.EGO_WAYPOINTS)
                    # Add intersection and its corresponding elements to the road network.
                    actions.append(action)
                    obstacles.extend(obs)
                    triggers.extend(trs)
                    directions.append(intersection.get("direction"))
                    roads.extend(intersection_items.get("roads"))
                    ego_roads.extend(intersection_items.get("ego_roads"))
                    last_point = intersection_items.get("last_point")
                    intersection_roads.extend(intersection_items.get("intersection_roads"))
                    road_index = intersection_items.get("road_index")
                    MIN_DEGREES, MAX_DEGREES = calc_min_max_angles(left_lanes + right_lanes)
                    lines_of_roads = convert_points_to_lines(roads)
                    number_of_pieces += 1
                    one_intersection = True
                    tries = 0
                else:
                    tries += 1
                intersection_possible = False
            # Add new point, if possible.
            control_points = roads[road_index].get("control_points")
            if len(control_points) == 1:
                new_point = self._add_segment(control_points[0])
            else:
                new_point = self._add_segment(control_points[-1], control_points[-2])
            # Temporarily add new point to the road network to validate it.
            new_line = LineString([(control_points[-1][0], control_points[-1][1]),
                                   (new_point[0], new_point[1])])
            temp_list = deepcopy(roads)
            temp_list[road_index].get("control_points").append(new_point)
            temp_list = self._bspline(temp_list)
            polyline = convert_points_to_lines(temp_list)
            width_lines = get_width_lines(temp_list)
            if not intersection_check_last(lines_of_roads, new_line, max_intersections=0) \
                    and not intersection_check_width(width_lines, polyline, intersection_roads):
                roads[road_index].get("control_points").append(new_point)
                intersection_possible = True
                tries = 0
                number_of_pieces += 1
                lines_of_roads = convert_points_to_lines(roads)
                last_point = new_point
            else:
                tries += 1
        if number_of_pieces >= self.MIN_NODES and one_intersection:
            print(colored("Finished creating urban scenario!", "grey", attrs=['bold']))
            return {"roads": roads, "success_point": {"position": last_point, "tolerance": 10}, "ego_roads": ego_roads,
                    "obstacles": obstacles, "directions": directions, "triggers": triggers, "tod": random(),
                    "intersection_roads": intersection_roads, "actions": actions, "damage_requests": ["ego"]}
        else:
            print(colored("Couldn't create a valid road network. Restarting...", "grey", attrs=['bold']))

    def _create_intersection(self, last_point, penultimate_point):
        """Generates an intersection with the correspoinding points road properties of the new (opposite) road.
        :param last_point: Last generated point.
        :param penultimate_point: Penultimate generated point.
        :return: Dict with the keys "intersection_point", "straight_point", "left_point", "right_point", "direction"
         (where should the ego-car go at this intersection?), "number_of_ways" (three-way/four-way intersection),
         "layout" (one of three possibilities of a three-way intersection), "new_left_lanes" (number of left lanes of
         the opposite road), "new_right_lanes", "new_width" (width of the opposite road).
        """
        layout = None
        random_number = random()

        # Choose random layout, number of ways and direction.
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
        # Generate straight point.
        line = LineString([(penultimate_point[0], penultimate_point[1]),
                           (last_point[0], last_point[1])])
        fac = get_resize_factor_intersection(line.length, self.INTERSECTION_LENGTH)
        angle = randint(-10, 10)
        line_rot = affinity.rotate(line, angle, line.coords[0])
        line_intersection = affinity.scale(line_rot, xfact=fac, yfact=fac, origin=line_rot.coords[0])
        straight_point = list(shape(line_intersection).coords)[1]

        # Generate intersection center point.
        fac = get_resize_factor_intersection(line.length, self.STRAIGHT_LENGTH)
        line_intersection = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        intersection_point = list(shape(line_intersection).coords)[1]
        line = LineString([(intersection_point[0], intersection_point[1]),
                           (line.coords[1][0], line.coords[1][1])])
        fac = get_resize_factor_intersection(line.length, self.OPPOSITE_ROAD)

        # Generate right point.
        angle = randint(-110, -70)
        line_rot1 = affinity.rotate(line, angle, line.coords[0])
        line_rot1 = affinity.scale(line_rot1, xfact=fac, yfact=fac,
                                   origin=line_rot1.coords[0])

        # Generate left point.
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
        """Interpolates all roads of a population.
        :param population: List of individuals.
        :return: List of individuals with bsplined control points.
        """
        for individual in population:
            splined_list = self._bspline(individual.get("roads"))
            iterator = 0
            while iterator < len(splined_list):
                road = splined_list[iterator]
                individual.get("roads")[iterator]["control_points"] = road.get("control_points").tolist()
                iterator += 1
        return population

    def _create_start_population(self):
        """Generates the initial test case population.
        :return:
        """
        i = 0
        while len(self.POPULATION) < self.POPULATION_SIZE:
            urban = self._create_urban_environment()
            if urban is not None:
                self.POPULATION.append({"roads": urban.get("roads"),
                                        "file_name": self.FILES_NAME,
                                        "obstacles": urban.get("obstacles"),
                                        "success_point": urban.get("success_point"),
                                        "ego_roads": urban.get("ego_roads"),
                                        "directions": urban.get("directions"),
                                        "fitness": 0,
                                        "triggers": urban.get("triggers"),
                                        "tod": urban.get("tod"),
                                        "intersection_roads": urban.get("intersection_roads"),
                                        "actions": urban.get("actions"),
                                        "damage_requests": urban.get("damage_requests")})
                i += 1

    def create_test_cases(self):
        """Generates a list of test cases and adds traffic participants, parked cars and interpolates the roads.
        :return: Void.
        """
        self._create_start_population()
        temp_list = deepcopy(self.POPULATION)
        temp_list = self._spline_population(temp_list)
        _preparation(population=temp_list, traffic=self.TRAFFIC, ego_waypoints=self.EGO_WAYPOINTS)
        temp_list = _merge_roads(temp_list)
        build_all_xml(temp_list)
        print(colored("Population finished.", "grey", attrs=['bold']))

    def get_test(self):
        """Returns the two first test files starting with "files_name".
        :return: Tuple of the path to the dbe and dbc file.
        """
        destination_path = path.dirname(path.realpath(__file__)) + "\\scenario"
        xml_names = destination_path + "\\" + self.FILES_NAME + "*"
        i = 0
        self.create_test_cases()
        matches = glob(xml_names)
        while i < self.POPULATION_SIZE * 2 - 1:
            yield Path(matches[i + 1]), Path(matches[i])
            i += 2

    def on_test_finished(self, score):
        """This method is called after a test was finished. Updates the fitness value of an individual.
        :return: Void.
        """
        i = 0
        while i < self.POPULATION_SIZE:
            self.POPULATION[i]["fitness"] = score
            i += 1
            yield

# TODO Desired features:
#       TODO Test setup.py
#       TODO Retest experiments
#       TODO Remove TODOS

# TODO Future Work:
#       TODO Buggy traffic
#       TODO Right turns are buggy
#       TODO Obstacles are on the street
#       TODO Only ego-car should be in onRaceStart.
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

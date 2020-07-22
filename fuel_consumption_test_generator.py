from copy import deepcopy
from random import randint, random, choice, uniform

from numpy import asarray, clip, concatenate, arange, linspace, array, around
from scipy.interpolate import splev
from shapely import affinity
from shapely.geometry import LineString, shape, Point
from termcolor import colored
from os import path
from glob import glob
from pathlib import Path
from scipy.spatial.distance import euclidean

from utils.utility_functions import convert_points_to_lines, get_angle, calc_width, \
    calc_min_max_angles, get_lanes_of_intersection, get_intersection_lines, get_width_lines, \
    get_resize_factor_intersection, multilinestrings_to_linestring
from utils.validity_checks import intersection_check_width, intersection_check_last
from utils.xml_creator import build_all_xml
from xml_converter.converter import b_spline

MIN_DEGREES = 90
MAX_DEGREES = 270


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
                                "movementMode": "_BEAMNG",
                                "lane": same_lane + 1}
                    waypoints.append(waypoint)
                iterator += 1
            del waypoints[-1]

    init_state = {"position": waypoints[0].get("position"),
                  "orientation": 0,
                  "movementMode": "_BEAMNG",
                  "speed": 50}
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
                    car_positions.append((point, angle))
                iterator += 1
        if right:
            right_lines = [line.parallel_offset(width / 2 + offset + x, "right") for x in noise]
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
                    car_positions.append((point, angle))
                iterator += 1
    parked_cars = list()
    color = (round(uniform(0, 1), 2), round(uniform(0, 1), 2), round(uniform(0, 1), 2), round(uniform(1, 1.3), 2))
    for position in car_positions:
        if random() <= 0.4:
            continue
        parked_cars.append({"name": "golf", "position": position[0], "zRot": position[1], "color": color})
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
    spawn_lanes.append(ego_lanes[-1])
    samples = 45
    i = 0
    waypoints = list()
    while i < len(spawn_lanes) - 1:
        lines = list()
        if spawn_lanes[i + 1] - spawn_lanes[i] == 1:
            spawn_indices = [spawn_lanes[i], spawn_lanes[i] + 1, spawn_lanes[i] + 2]
            spawn_index = choice(spawn_indices)
            end_indices = [spawn_lanes[i] - 1, spawn_lanes[i], spawn_lanes[i] + 1]
            end_index = choice(end_indices)
            while end_index == spawn_index:
                end_index = choice(end_indices)
            spawn_point = lanes[spawn_index].get("control_points")[-1]
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
                line.coords = line.coords[::-1]
            line.coords = b_spline(list(line.coords), samples).tolist()
            line.coords = line.coords[:-4]
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
                                    "movementMode": "_BEAMNG",
                                    "lane": spawn_index}
                        waypoints.append(waypoint)
            trigger_point = {"position": temp_line.coords[-1],
                                   "action": "spawn_and_start",
                                   "tolerance": 2,
                                   "triggeredBy": "ego",
                                   "triggers": "other_0"}
            spawn_point = {"position": list(lines[0].coords)[0], "orientation": orientation}
            triggers.append({"triggerPoint": trigger_point, "spawnPoint": spawn_point})

        i += 1
    if len(waypoints) != 0:
        init_state = {"position": waypoints[0].get("position"),
                      "orientation": triggers[0].get("spawnPoint").get("orientation"),
                      "movementMode": "_BEAMNG",
                      "speed": 50}
        other = {"id": "other_{}".format(0),
                 "init_state": init_state,
                 "waypoints": waypoints,
                 "model": "ETK800",
                 "color": choice(colors)}
        individual["participants"].append(other)
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


def _add_traffic_signs(last_point, current_left_lanes, current_right_lanes, width, intersection):
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
        if sign_on_my_lane == "stopsign":
            obstacles.append({"name": "prioritysign", "position": my_position, "zRot": my_z_rot})
        else:
            if new_right_lanes == 1:
                obstacles.append({"name": "trafficlightsingle", "position": my_position, "zRot": my_z_rot,
                                  "mode": mode, "sign": "priority"})
            else:
                obstacles.append({"name": "trafficlightdouble", "position": my_position, "zRot": my_z_rot,
                                  "mode": mode, "sign": "priority"})

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

    modes = ["off", "blinking"]
    mode = choice(modes)

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
                              "sign": "yield"})
    else:
        obstacles.append({"name": "trafficlightdouble", "position": position, "zRot": z_rot, "mode": mode,
                          "sign": "yield"})
    sign_on_my_lane = obstacles[0].get("name")

    # Left direction.
    if number_of_ways == 4 or direction == "left" or layout == "left":
        opposite_direction(left_point, last_point)

    # Top direction.
    if number_of_ways == 4 or direction == "straight" or layout == "straight":
        position, z_rot = my_direction(straight_point, left_point)
        obstacles.append({"name": sign_on_my_lane, "position": position, "zRot": z_rot})
        if sign_on_my_lane.startswith("trafficlight"):
            obstacles[-1]["mode"] = mode
            obstacles[-1]["sign"] = "yield"

    # Right direction.
    if number_of_ways == 4 or direction == "right" or layout == "right":
        opposite_direction(right_point, straight_point)

    return obstacles


def _preparation(population):
    for individual in population:
        _add_parked_cars(individual)
        _add_ego_car(individual)
        _add_other_participants(individual)


class FuelConsumptionTestGenerator:

    def __init__(self):
        self.files_name = "urban"
        self.SPLINE_DEGREE = 3  # Sharpness of curves
        self.MAX_TRIES = 20  # Maximum number of invalid generated points/segments
        self.POPULATION_SIZE = 1  # Minimum number of generated roads for each generation
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
        print(colored("##################", attrs=["bold"]))
        print(colored("##################", "red", attrs=["bold"]))
        print(colored("##################", "yellow", attrs=["bold"]))

    def _bspline(self, lanes):
        """Calculate {@code samples} samples on a bspline. This is the road representation function.
        :param lanes: List of lanes.
        :return: List of arrays with samples, representing a bspline of the given control points of the lanes.
        """
        splined_list = []
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
        :param last_point: Last point of the control point list as dict type.
        :param penultimate_point: Point before the last point as dict type.
        :return: A new random point as dict type.
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
        tries = 0
        lane_index = 0
        number_of_pieces = 3
        one_intersection = False
        intersection_possible = True
        intersection_probability = 0.25
        lines_of_roads = convert_points_to_lines(lanes)
        last_point = p2
        while number_of_pieces <= self.MAX_NODES and tries <= self.MAX_TRIES:
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
                    obstacles.extend(_add_traffic_signs(control_points[-1],
                                                        lanes[lane_index].get("left_lanes"),
                                                        lanes[lane_index].get("right_lanes"),
                                                        lanes[lane_index].get("width"), intersection))
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
            return {"lanes": lanes, "success_point": last_point, "ego_lanes": ego_lanes, "obstacles": obstacles,
                    "directions": directions}
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
        print(population)
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
                                 "type": "urban",
                                 "file_name": self.files_name,
                                 "score": 0,
                                 "obstacles": urban.get("obstacles"),
                                 "success_point": urban.get("success_point"),
                                 "ego_lanes": urban.get("ego_lanes"),
                                 "directions": urban.get("directions"),
                                 "fitness": 0,
                                 "triggers": urban.get("triggers")})
                i += 1
        return startpop

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
        """
        if len(self.population_list) == 0:
            self.population_list = self._create_start_population()
        print(colored("Population finished.", "grey", attrs=['bold']))
        temp_list = deepcopy(self.population_list)
        # plot_all(temp_list)
        """
        temp_list = [{'lanes': [{'control_points': [(1, 0), (30, 0), (45, 0), (69, 4), (94, -11), (116, -24), (139, -9), (164, 7), (187, -9)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}, {'control_points': [(187, -9), (203.41810403570975, -20.421289763972)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(203.41810403570975, -20.421289763972), (226.75633814376346, 23.797787891692373)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(203.41810403570975, -20.421289763972), (163.05747038638617, -49.93432504374416)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(203.41810403570975, -20.421289763972), (228.0452600892744, -37.55322440992999)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(228.0452600892744, -37.55322440992999), (225, -56), (215, -76)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 100, 'type': 'normal'}, {'control_points': [(215, -76), (206.05572809000085, -93.88854381999832)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(206.05572809000085, -93.88854381999832), (192.63932022500205, -120.72135954999578)], 'width': 12, 'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {'control_points': [(206.05572809000085, -93.88854381999832), (169.2302368117555, -60.067201562248044)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(206.05572809000085, -93.88854381999832), (249.10834648419518, -119.31435844615905)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [(249.10834648419518, -119.31435844615905)], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}], 'type': 'urban', 'file_name': 'urban', 'score': 0, 'obstacles': [{'name': 'trafficlightdouble', 'position': (193.53190699907663, -21.096573521266844), 'zRot': 325, 'mode': 'off', 'sign': 'yield'}, {'name': 'trafficlightsingle', 'position': (202.16602273061554, -11.653097680912136), 'zRot': 242, 'mode': 'off', 'sign': 'priority'}, {'name': 'trafficlightdouble', 'position': (211.14532039164706, -18.24410640271481), 'zRot': 505, 'mode': 'off', 'sign': 'yield'}, {'name': 'trafficlightsingle', 'position': (201.5634623995885, -28.21938596763386), 'zRot': 396, 'mode': 'off', 'sign': 'priority'}, {'name': 'trafficlightdouble', 'position': (203.7928272967711, -84.55072394595918), 'zRot': 243, 'mode': 'blinking', 'sign': 'yield'}, {'name': 'trafficlightsingle', 'position': (213.95243225521324, -92.51302089139372), 'zRot': 509, 'mode': 'blinking', 'sign': 'priority'}, {'name': 'trafficlightdouble', 'position': (209.01628209221056, -101.83105727607759), 'zRot': 423, 'mode': 'blinking', 'sign': 'yield'}, {'name': 'trafficlightsingle', 'position': (198.0455985592489, -93.5921911574903), 'zRot': 317, 'mode': 'blinking', 'sign': 'priority'}], 'success_point': (249.10834648419518, -119.31435844615905), 'ego_lanes': [0, 1, 4, 5, 6, 9, 10], 'directions': ['straight', 'left'], 'fitness': 0}]
        temp_list = self._spline_population(temp_list)
        _preparation(temp_list)
        temp_list = _merge_lanes(temp_list)
        build_all_xml(temp_list)
        self.population_list = []

    def get_test(self):
        """Returns the two first test files starting with "files_name".
        :return: Tuple of the path to the dbe and dbc file.
        """
        destination_path = path.dirname(path.realpath(__file__)) + "\\scenario"
        xml_names = destination_path + "\\" + self.files_name + "*"
        matches = glob(xml_names)
        iterator = 0
        self.genetic_algorithm()
        while iterator < self.POPULATION_SIZE * 2 - 1:
            yield Path(matches[iterator + 1]), Path(matches[iterator])
            iterator += 2

# TODO Desired features:
#       TODO Traffic lights
#       TODO Teleporting cars shouldnt be visible to ego(line triggered by ego, teleport by other)
#       TODO Add other participants, add traffic for 3-way-lanes
#       TODO Mutation
#       TODO Crossover
#       TODO Refactor
#       TODO Comments
#       TODO Fix lane markings

# TODO Verifier:
#       TODO Throttle misbehavior
#       TODO Accelerate and stop misbehaviour
#       TODO Left turn misbehavior
#       TODO Right turn misbehavior
#       TODO Brake misbehavior
#       TODO Safety distance misbehavior
#       TODO Engine idling misbehavior
#       TODO Varying speed
#       TODO Calculation

# TODO May-have/Improvements:
#       TODO Remove redundant XML information
#       TODO Daytime
#       TODO Add weather presets
#       TODO Improve performance
#       TODO Make all objects collidable
#       TODO Converter:
#           TODO Add input checking
#           TODO Implement Sensor deployment
#       TODO Parked cars on the road and adjust waypoints
#       TODO Improve speed of car
#       TODO Improve traffic sign positioning
#       TODO Test generator should calc speed of waypoints, not converter
#       TODO Parallel offset instead of width lines

from copy import deepcopy
from random import randint, random, choice

from numpy import asarray, clip, concatenate, arange, linspace, array
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
    get_resize_factor_intersection
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
    samples = 65
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
            left_line = line.parallel_offset(width / 2 + offset, "left")
            coords = b_spline(left_line.coords)
            iterator = 1
            while iterator < len(coords):
                point = coords[iterator]
                if abs(euclidean(point, coords[-1])) < 12:
                    break
                if len(car_positions) == 0 or (abs(euclidean(point, car_positions[-1][0])) > max_distance and
                                               (prev_lane is None or Point(point).distance(prev_lane) > prev_width)):
                    angle = get_angle((coords[iterator - 1][0] + 5, coords[iterator - 1][1]),
                                      coords[iterator - 1], point) - rotation
                    car_positions.append((point, angle))
                iterator += 1
        if right:
            right_line = line.parallel_offset(width / 2 + offset, "right")
            coords = right_line.coords[::-1]
            coords = b_spline(coords)
            iterator = 1
            while iterator < len(coords):
                point = coords[iterator]
                if abs(euclidean(point, coords[-1])) < 12:
                    break
                if len(car_positions) == 0 or (abs(euclidean(point, car_positions[-1][0])) > max_distance and
                                               (prev_lane is None or Point(point).distance(prev_lane) > prev_width)):
                    angle = get_angle((coords[iterator - 1][0] + 5, coords[iterator - 1][1]),
                                      coords[iterator - 1], point) + 180 - rotation
                    car_positions.append((point, angle))
                iterator += 1
    parked_cars = list()
    for position in car_positions:
        if random() <= 0.4:
            continue
        parked_cars.append({"name": "golf", "position": position[0], "zRot": position[1]})
    individual["obstacles"].extend(parked_cars)


def _add_other_participants(individual):
    colors = ["White", "Red", "Green", "Yellow", "Black", "Blue", "Orange", "Gray", "Purple"]
    ego_lanes = individual.get("ego_lanes")
    lanes = individual.get("lanes")
    print(ego_lanes)

    # Drive from one opposite lane to another at an intersection.
    # Get lanes where a car can spawn, be teleported to or drive to.
    spawn_lanes = [0]
    for idx, lane in enumerate(lanes):
        if idx not in ego_lanes:
            spawn_lanes.append(idx)
    spawn_lanes.append(ego_lanes[-1])
    spawn_points = list()
    trigger_points = list()
    samples = 50
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
            tolerance = lanes[spawn_lanes[i] - 1].get("width") / 2 + 2
            trigger = lanes[spawn_lanes[i] - 1].get("control_points")[0] if spawn_index != spawn_lanes[i] + 2 \
                else lanes[spawn_lanes[i] - 2].get("control_points")[0]
            trigger_points.append({"position": trigger,
                                   "action": "spawn_and_start",
                                   "tolerance": tolerance,
                                   "vid": "ego"})

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
            line.coords = line.coords[::-1]
            line.coords = b_spline(list(line.coords), samples).tolist()
            line.coords = line.coords[samples//10:]
            lines.append(line)
            for line in lines:
                for point in list(line.coords):
                    if len(waypoints) == 0 or euclidean(point, waypoints[-1].get("position")) >= 1.5:
                        waypoint = {"position": point,
                                    "tolerance": 2,
                                    "movementMode": "_BEAMNG",
                                    "lane": spawn_index}
                        waypoints.append(waypoint)
            spawn_points.append({"position": list(lines[0].coords)[0], "orientation": orientation})

        i += 1
    init_state = {"position": waypoints[0].get("position"),
                  "orientation": spawn_points[0].get("orientation"),
                  "movementMode": "_BEAMNG",
                  "speed": 50}
    triggers = {"triggerPoints": trigger_points,
                "spawnPoints": spawn_points}
    other = {"id": "other_{}".format(0),
             "init_state": init_state,
             "waypoints": waypoints,
             "model": "ETK800",
             "color": choice(colors),
             "triggerPoints": triggers}
    individual["participants"].append(other)


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


def _add_traffic_signs(last_point, intersection_point, current_left_lanes, current_right_lanes,
                       new_left_lanes, new_right_lanes, layout, number_of_ways, direction, left_point,
                       straight_point, right_point, width, width_opposite):
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
            if new_left_lanes + new_right_lanes == 2:
                obstacles.append({"name": "trafficlightsingle", "position": my_position, "zRot": my_z_rot})
            else:
                obstacles.append({"name": "trafficlightdouble", "position": my_position, "zRot": my_z_rot})

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

    # Calculate traffic sign position.
    old_width = width / (current_left_lanes + current_right_lanes)
    new_width = width_opposite / (new_left_lanes + new_right_lanes)
    obstacles = list()
    temp_point = (intersection_point[0] + 5, intersection_point[1])

    # Bottom direction.
    position, z_rot = my_direction(last_point, right_point)
    if current_left_lanes + current_right_lanes == 2:
        if random() <= 0.5:
            obstacles.append({"name": "stopsign", "position": position, "zRot": z_rot})
        else:
            obstacles.append({"name": "trafficlightsingle", "position": position, "zRot": z_rot})
    else:
        obstacles.append({"name": "trafficlightdouble", "position": position, "zRot": z_rot})
    sign_on_my_lane = obstacles[0].get("name")

    # Left direction.
    if number_of_ways == 4 or direction == "left" or layout == "left":
        opposite_direction(left_point, last_point)

    # Top direction.
    if number_of_ways == 4 or direction == "straight" or layout == "straight":
        position, z_rot = my_direction(straight_point, left_point)
        obstacles.append({"name": sign_on_my_lane, "position": position, "zRot": z_rot})

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
            splined_list.append({"control_points": array(splev(u, (kv, point_list.T, degree))).T,
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
                    obstacles.extend(_add_traffic_signs(control_points[-1], intersection.get("intersection_point"),
                                                        lanes[lane_index].get("left_lanes"),
                                                        lanes[lane_index].get("right_lanes"),
                                                        left_lanes, right_lanes, intersection.get("layout"),
                                                        intersection.get("number_of_ways"),
                                                        intersection.get("direction"),
                                                        intersection.get("left_point"),
                                                        intersection.get("straight_point"),
                                                        intersection.get("right_point"),
                                                        lanes[lane_index].get("width"),
                                                        intersection.get("new_width")))
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

    def _repair(self):
        pass

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
                                 "fitness": 0})
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
        temp_list = self._spline_population(temp_list)
        """
        temp_list = [{'lanes': [{'control_points': [[1.0, 0.0], [1.5844301601877362, 0.0], [2.166003469033772, 0.0],
                                                    [2.744719926538109, 0.0], [3.3205795327007452, 0.0],
                                                    [3.893582287521682, 0.0], [4.463728191000919, 0.0],
                                                    [5.031017243138455, 0.0], [5.595449443934293, 0.0],
                                                    [6.15702479338843, 0.0], [6.715743291500868, 0.0],
                                                    [7.271604938271604, 0.0], [7.824609733700642, 0.0],
                                                    [8.374757677787983, 0.0], [8.922048770533621, 0.0],
                                                    [9.466483011937559, 0.0], [10.008060401999797, 0.0],
                                                    [10.546780940720335, 0.0], [11.082644628099173, 0.0],
                                                    [11.615651464136315, 0.0], [12.145801448831754, 0.0],
                                                    [12.673094582185492, 0.0], [13.197530864197532, 0.0],
                                                    [13.719110294867871, 0.0], [14.237832874196512, 0.0],
                                                    [14.753698602183452, 0.0], [15.266707478828693, 0.0],
                                                    [15.776859504132233, 0.0], [16.284154678094072, 0.0],
                                                    [16.78859300071421, 0.0], [17.290174471992657, 0.0],
                                                    [17.788899091929395, 0.0], [18.284766860524435, 0.0],
                                                    [18.77777777777778, 0.0], [19.267931843689425, 0.0],
                                                    [19.75522905825936, 0.0], [20.239669421487605, 0.0],
                                                    [20.721252933374146, 0.0], [21.19997959391899, 0.0],
                                                    [21.67584940312213, 0.0], [22.148862360983575, 0.0],
                                                    [22.619018467503324, 0.0], [23.086317722681358, 0.0],
                                                    [23.5507601265177, 0.0], [24.01234567901235, 0.0],
                                                    [24.47107438016529, 0.0], [24.926946229976533, 0.0],
                                                    [25.379961228446078, 0.0], [25.830119375573922, 0.0],
                                                    [26.277420671360066, 0.0], [26.721865115804512, 0.0],
                                                    [27.163452708907258, 0.0], [27.602183450668303, 0.0],
                                                    [28.038057341087644, 0.0], [28.47107438016529, 0.0],
                                                    [28.901234567901234, 0.0], [29.328537904295487, 0.0],
                                                    [29.75298438934803, 0.0], [30.174574023058874, 0.0],
                                                    [30.59330680542802, 0.0], [31.009182736455465, 0.0],
                                                    [31.42220181614121, 0.0], [31.83236404448526, 0.0],
                                                    [32.239669421487605, 0.0], [32.64411794714825, 0.0],
                                                    [33.045709621467196, 0.0], [33.44444444444444, 0.0],
                                                    [33.84032241608, 0.0], [34.233343536373845, 0.0],
                                                    [34.623507805325985, 0.0], [35.010815222936436, 0.0],
                                                    [35.39526578920519, 0.0], [35.77685950413223, 0.0],
                                                    [36.155596367717585, 0.0], [36.531476379961234, 0.0],
                                                    [36.904499540863185, 0.0], [37.27466585042343, 0.0],
                                                    [37.641975308641975, 0.0], [38.00642791551883, 0.0],
                                                    [38.36802367105397, 0.0], [38.72676257524743, 0.0],
                                                    [39.082644628099175, 0.0], [39.435669829609225, 0.0],
                                                    [39.78583817977758, 0.0], [40.13314967860423, 0.0],
                                                    [40.47760432608918, 0.0], [40.81920212223243, 0.0],
                                                    [41.15794306703398, 0.0], [41.49382716049382, 0.0],
                                                    [41.82685440261198, 0.0], [42.15702479338843, 0.0],
                                                    [42.484338332823185, 0.0], [42.808795020916236, 0.0],
                                                    [43.13039485766758, 0.0], [43.44913784307724, 0.0],
                                                    [43.7650239771452, 0.0], [44.07805325987144, 0.0],
                                                    [44.388225691256, 0.0], [44.69554127129885, 0.0], [45.0, 0.0]],
                                 'width': 12, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {
                                    'control_points': [[45.0, 0.0], [45.833333333333336, 0.0],
                                                       [46.666666666666664, 0.0], [47.5, 0.0], [48.33333333333333, 0.0],
                                                       [49.166666666666664, 0.0], [50.0, 0.0], [50.83333333333333, 0.0],
                                                       [51.66666666666667, 0.0], [52.5, 0.0], [53.33333333333333, 0.0],
                                                       [54.16666666666667, 0.0], [55.0, 0.0], [55.83333333333333, 0.0],
                                                       [56.66666666666667, 0.0], [57.5, 0.0], [58.33333333333333, 0.0],
                                                       [59.16666666666667, 0.0], [60.0, 0.0], [60.83333333333333, 0.0],
                                                       [61.66666666666667, 0.0], [62.5, 0.0], [63.33333333333333, 0.0],
                                                       [64.16666666666667, 0.0], [65.0, 0.0]], 'width': 12,
                                    'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {
                                    'control_points': [[65.0, 0.0], [66.25, 0.0], [67.5, 0.0], [68.75, 0.0],
                                                       [70.0, 0.0], [71.25, 0.0], [72.5, 0.0], [73.75, 0.0],
                                                       [75.0, 0.0], [76.25, 0.0], [77.5, 0.0], [78.75, 0.0],
                                                       [80.0, 0.0], [81.25, 0.0], [82.5, 0.0], [83.75, 0.0],
                                                       [85.0, 0.0], [86.25, 0.0], [87.5, 0.0], [88.75, 0.0],
                                                       [90.0, 0.0], [91.25, 0.0], [92.5, 0.0], [93.75, 0.0],
                                                       [95.0, 0.0]], 'width': 12, 'left_lanes': 2, 'right_lanes': 1,
                                    'samples': 25, 'type': 'intersection'}, {
                                    'control_points': [[65.0, 0.0], [64.39089228182763, -1.9923015749229898],
                                                       [63.78178456365526, -3.9846031498459795],
                                                       [63.1726768454829, -5.9769047247689695],
                                                       [62.56356912731053, -7.969206299691959],
                                                       [61.954461409138155, -9.961507874614949],
                                                       [61.34535369096579, -11.953809449537939],
                                                       [60.73624597279342, -13.946111024460928],
                                                       [60.12713825462105, -15.938412599383918],
                                                       [59.51803053644868, -17.930714174306907],
                                                       [58.90892281827631, -19.923015749229897],
                                                       [58.29981510010394, -21.915317324152888],
                                                       [57.690707381931574, -23.907618899075878],
                                                       [57.08159966375921, -25.899920473998865],
                                                       [56.47249194558684, -27.892222048921855],
                                                       [55.86338422741447, -29.88452362384485],
                                                       [55.2542765092421, -31.876825198767836],
                                                       [54.645168791069736, -33.86912677369082],
                                                       [54.03606107289736, -35.86142834861381],
                                                       [53.42695335472499, -37.853729923536804],
                                                       [52.81784563655262, -39.846031498459794],
                                                       [52.20873791838025, -41.838333073382785],
                                                       [51.59963020020789, -43.830634648305775],
                                                       [50.990522482035516, -45.822936223228766],
                                                       [50.38141476386315, -47.815237798151756]], 'width': 15,
                                    'left_lanes': 1, 'right_lanes': 2, 'samples': 25, 'type': 'intersection'}, {
                                    'control_points': [[65.0, 0.0], [65.21776763180762, 2.071920615350569],
                                                       [65.43553526361522, 4.143841230701138],
                                                       [65.65330289542283, 6.215761846051708],
                                                       [65.87107052723044, 8.287682461402277],
                                                       [66.08883815903806, 10.359603076752846],
                                                       [66.30660579084567, 12.431523692103417],
                                                       [66.52437342265328, 14.503444307453984],
                                                       [66.74214105446089, 16.575364922804553],
                                                       [66.9599086862685, 18.647285538155124],
                                                       [67.17767631807611, 20.71920615350569],
                                                       [67.39544394988371, 22.791126768856262],
                                                       [67.61321158169133, 24.863047384206833],
                                                       [67.83097921349894, 26.9349679995574],
                                                       [68.04874684530655, 29.006888614907968],
                                                       [68.26651447711416, 31.078809230258543],
                                                       [68.48428210892178, 33.150729845609106],
                                                       [68.7020497407294, 35.22265046095968],
                                                       [68.919817372537, 37.29457107631025],
                                                       [69.1375850043446, 39.36649169166082],
                                                       [69.35535263615222, 41.43841230701138],
                                                       [69.57312026795984, 43.51033292236196],
                                                       [69.79088789976744, 45.582253537712525],
                                                       [70.00865553157506, 47.654174153063096],
                                                       [70.22642316338266, 49.72609476841367]], 'width': 15,
                                    'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {
                                    'control_points': [[70.22642316338266, 49.72609476841367],
                                                       [69.76601508400034, 50.4918912492673],
                                                       [69.30764878433754, 51.25754647853126],
                                                       [68.85139088454025, 52.023086884586334],
                                                       [68.39730800475442, 52.78853889581321],
                                                       [67.94546676512604, 53.55392894059264],
                                                       [67.49593378580107, 54.31928344730533],
                                                       [67.0487756869255, 55.084628844332],
                                                       [66.6040590886453, 55.84999156005342],
                                                       [66.16185061110642, 56.61539802285023],
                                                       [65.7222168744549, 57.38087466110325],
                                                       [65.28522449883666, 58.14644790319314],
                                                       [64.8509401043977, 58.91214417750065],
                                                       [64.41943031128397, 59.67798991240651],
                                                       [63.99076173964146, 60.444011536291434],
                                                       [63.565001009616175, 61.210235477536166],
                                                       [63.14221474135403, 61.97668816452139],
                                                       [62.72246955500106, 62.74339602562788],
                                                       [62.30583207070319, 63.510385489236334],
                                                       [61.89236890860645, 64.27768298372749],
                                                       [61.482146688856766, 65.04531493748206],
                                                       [61.07523203160011, 65.81330777888077],
                                                       [60.67169155698253, 66.58168793630438],
                                                       [60.271591885149896, 67.35048183813355],
                                                       [59.87499963624827, 68.11971591274907],
                                                       [59.48198143042358, 68.88941658853163],
                                                       [59.09260388782184, 69.65961029386197],
                                                       [58.706933628588985, 70.43032345712082],
                                                       [58.325037272871, 71.20158250668888],
                                                       [57.946981440813865, 71.97341387094687],
                                                       [57.57283275256358, 72.74584397827557],
                                                       [57.20265782826607, 73.51889925705565],
                                                       [56.83652328806734, 74.29260613566787],
                                                       [56.47449575211338, 75.06699104249294],
                                                       [56.11664184055015, 75.8420804059116],
                                                       [55.76302817352361, 76.61790065430453],
                                                       [55.413721371179754, 77.39447821605252],
                                                       [55.06878805366456, 78.17183951953625],
                                                       [54.728294841123976, 78.95001099313644],
                                                       [54.392308353703996, 79.72901906523384],
                                                       [54.06089521155063, 80.50889016420922],
                                                       [53.73412203480981, 81.28965071844323],
                                                       [53.4120554436275, 82.07132715631658],
                                                       [53.09476205814971, 82.85394590621007],
                                                       [52.78230849852242, 83.63753339650442],
                                                       [52.47476138489156, 84.42211605558028],
                                                       [52.172187337403116, 85.20772031181843],
                                                       [51.874652976203116, 85.9943725935996],
                                                       [51.582224921437486, 86.78209932930451],
                                                       [51.2949697932522, 87.57092694731386],
                                                       [51.01295421179328, 88.36088187600845],
                                                       [50.73624479720665, 89.15199054376889],
                                                       [50.464908169638306, 89.94427937897598],
                                                       [50.199010949234214, 90.73777481001044],
                                                       [49.93861975614037, 91.53250326525297],
                                                       [49.68380121050273, 92.32849117308433],
                                                       [49.434621932467266, 93.12576496188524],
                                                       [49.19114854217998, 93.9243510600364],
                                                       [48.953447659786825, 94.72427589591851],
                                                       [48.72158590543377, 95.52556589791237],
                                                       [48.495629899266824, 96.32824749439867],
                                                       [48.27564626143192, 97.13234711375813],
                                                       [48.061701612075055, 97.93789118437147],
                                                       [47.85386257134222, 98.74490613461946],
                                                       [47.65219575937935, 99.55341839288275],
                                                       [47.456767796332464, 100.36345438754212],
                                                       [47.2676453023475, 101.1750405469783],
                                                       [47.084894897570464, 101.98820329957198],
                                                       [46.90858320214731, 102.8029690737039],
                                                       [46.73877683622402, 103.6193642977548],
                                                       [46.57554241994657, 104.43741540010538],
                                                       [46.418946573460936, 105.25714880913637],
                                                       [46.26905591691309, 106.07859095322853],
                                                       [46.12593707044901, 106.90176826076257],
                                                       [45.989656654214684, 107.72670716011916],
                                                       [45.86028128835606, 108.5534340796791],
                                                       [45.737877593019135, 109.38197544782308],
                                                       [45.62251218834989, 110.21235769293185],
                                                       [45.514251694494256, 111.0446072433861],
                                                       [45.413162731598256, 111.87875052756655],
                                                       [45.31931191980786, 112.71481397385399],
                                                       [45.23276587926902, 113.5528240106291],
                                                       [45.15359123012772, 114.39280706627258],
                                                       [45.08185459252995, 115.23478956916522],
                                                       [45.01762258662167, 116.07879794768766],
                                                       [44.96096183254886, 116.92485863022073],
                                                       [44.91193895045749, 117.77299804514506],
                                                       [44.87062056049355, 118.62324262084144],
                                                       [44.83707328280299, 119.47561878569056],
                                                       [44.81136373753181, 120.33015296807318],
                                                       [44.793558544825984, 121.18687159636997],
                                                       [44.783724324831475, 122.04580109896169],
                                                       [44.78192769769426, 122.90696790422909],
                                                       [44.78823528356032, 123.77039844055284],
                                                       [44.802713702575616, 124.63611913631371],
                                                       [44.82542957488615, 125.50415641989241],
                                                       [44.85644952063788, 126.37453671966966],
                                                       [44.89584015997677, 127.2472864640262],
                                                       [44.94366811304883, 128.12243208134274], [45.0, 129.0]],
                                    'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {
                                    'control_points': [[45.0, 129.0], [45.05733507634615, 129.83135860701915],
                                                       [45.1146701526923, 130.6627172140383],
                                                       [45.17200522903845, 131.49407582105746],
                                                       [45.229340305384596, 132.32543442807662],
                                                       [45.286675381730745, 133.15679303509577],
                                                       [45.344010458076895, 133.98815164211493],
                                                       [45.40134553442304, 134.81951024913405],
                                                       [45.45868061076919, 135.65086885615324],
                                                       [45.516015687115335, 136.48222746317236],
                                                       [45.573350763461484, 137.31358607019152],
                                                       [45.63068583980764, 138.14494467721067],
                                                       [45.68802091615378, 138.97630328422983],
                                                       [45.745355992499924, 139.80766189124898],
                                                       [45.80269106884608, 140.63902049826814],
                                                       [45.86002614519223, 141.4703791052873],
                                                       [45.91736122153838, 142.30173771230645],
                                                       [45.97469629788452, 143.13309631932557],
                                                       [46.03203137423067, 143.96445492634473],
                                                       [46.08936645057682, 144.79581353336388],
                                                       [46.14670152692297, 145.62717214038304],
                                                       [46.20403660326912, 146.4585307474022],
                                                       [46.261371679615266, 147.28988935442135],
                                                       [46.318706755961415, 148.1212479614405],
                                                       [46.376041832307564, 148.95260656845966]], 'width': 15,
                                    'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {
                                    'control_points': [[46.376041832307564, 148.95260656845966],
                                                       [44.318046943905124, 149.2765430083475],
                                                       [42.26005205550268, 149.60047944823532],
                                                       [40.20205716710024, 149.92441588812315],
                                                       [38.14406227869781, 150.24835232801098],
                                                       [36.08606739029537, 150.5722887678988],
                                                       [34.02807250189292, 150.89622520778664],
                                                       [31.970077613490485, 151.22016164767447],
                                                       [29.912082725088045, 151.5440980875623],
                                                       [27.8540878366856, 151.86803452745013],
                                                       [25.79609294828316, 152.19197096733797],
                                                       [23.738098059880723, 152.5159074072258],
                                                       [21.68010317147828, 152.83984384711363],
                                                       [19.622108283075843, 153.16378028700143],
                                                       [17.564113394673402, 153.4877167268893],
                                                       [15.506118506270957, 153.8116531667771],
                                                       [13.44812361786852, 154.13558960666495],
                                                       [11.390128729466081, 154.45952604655275],
                                                       [9.332133841063637, 154.7834624864406],
                                                       [7.274138952661199, 155.10739892632841],
                                                       [5.21614406425876, 155.43133536621625],
                                                       [3.158149175856316, 155.75527180610408],
                                                       [1.1001542874538774, 156.0792082459919],
                                                       [-0.9578406009485609, 156.40314468587974],
                                                       [-3.015835489351005, 156.72708112576757]], 'width': 8,
                                    'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {
                                    'control_points': [[46.376041832307564, 148.95260656845966],
                                                       [48.41343401569698, 149.38770525708267],
                                                       [50.45082619908639, 149.82280394570572],
                                                       [52.488218382475814, 150.25790263432873],
                                                       [54.525610565865236, 150.69300132295177],
                                                       [56.56300274925465, 151.12810001157482],
                                                       [58.600394932644065, 151.56319870019783],
                                                       [60.63778711603348, 151.99829738882087],
                                                       [62.6751792994229, 152.4333960774439],
                                                       [64.71257148281231, 152.86849476606693],
                                                       [66.74996366620172, 153.30359345468995],
                                                       [68.78735584959115, 153.738692143313],
                                                       [70.82474803298057, 154.173790831936],
                                                       [72.86214021636998, 154.60888952055905],
                                                       [74.8995323997594, 155.04398820918206],
                                                       [76.93692458314882, 155.47908689780508],
                                                       [78.97431676653824, 155.91418558642812],
                                                       [81.01170894992765, 156.34928427505113],
                                                       [83.04910113331707, 156.78438296367418],
                                                       [85.08649331670648, 157.2194816522972],
                                                       [87.1238855000959, 157.65458034092023],
                                                       [89.16127768348532, 158.08967902954328],
                                                       [91.19866986687474, 158.5247777181663],
                                                       [93.23606205026414, 158.9598764067893],
                                                       [95.27345423365357, 159.39497509541235]], 'width': 8,
                                    'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {
                                    'control_points': [[46.376041832307564, 148.95260656845966],
                                                       [46.46204444682679, 150.19964447898838],
                                                       [46.54804706134601, 151.4466823895171],
                                                       [46.63404967586523, 152.69372030004584],
                                                       [46.72005229038446, 153.9407582105746],
                                                       [46.806054904903675, 155.1877961211033],
                                                       [46.89205751942289, 156.43483403163202],
                                                       [46.97806013394212, 157.68187194216074],
                                                       [47.064062748461346, 158.9289098526895],
                                                       [47.15006536298057, 160.1759477632182],
                                                       [47.236067977499786, 161.42298567374695],
                                                       [47.32207059201902, 162.67002358427567],
                                                       [47.40807320653823, 163.9170614948044],
                                                       [47.49407582105746, 165.1640994053331],
                                                       [47.58007843557668, 166.41113731586182],
                                                       [47.6660810500959, 167.65817522639057],
                                                       [47.75208366461513, 168.90521313691931],
                                                       [47.838086279134345, 170.15225104744803],
                                                       [47.924088893653575, 171.39928895797675],
                                                       [48.01009150817279, 172.6463268685055],
                                                       [48.09609412269201, 173.89336477903421],
                                                       [48.18209673721124, 175.14040268956293],
                                                       [48.268099351730456, 176.38744060009165],
                                                       [48.35410196624968, 177.63447851062037],
                                                       [48.4401045807689, 178.88151642114912]], 'width': 15,
                                    'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {
                                    'control_points': [[48.4401045807689, 178.88151642114912],
                                                       [48.276286908038706, 179.40235453551347],
                                                       [48.11446117953754, 179.92734428937823],
                                                       [47.954513367911495, 180.45642457879663],
                                                       [47.79632944580664, 180.98953429982168],
                                                       [47.63979538586905, 181.52661234850672],
                                                       [47.484797160744826, 182.06759762090476],
                                                       [47.33122074307999, 182.61242901306892],
                                                       [47.17895210552068, 183.16104542105245],
                                                       [47.027877220712924, 183.7133857409084],
                                                       [46.877882061302834, 184.26938886869002],
                                                       [46.72885259993646, 184.8289937004504],
                                                       [46.58067480925989, 185.39213913224268],
                                                       [46.43323466191921, 185.95876406012007],
                                                       [46.28641813056048, 186.52880738013562],
                                                       [46.14011118782979, 187.1022079883425],
                                                       [45.99419980637319, 187.67890478079394],
                                                       [45.848569958836805, 188.25883665354303],
                                                       [45.703107617866664, 188.84194250264287],
                                                       [45.55769875610888, 189.42816122414672],
                                                       [45.41222934620949, 190.01743171410766],
                                                       [45.26658536081459, 190.6096928685788],
                                                       [45.12065277257028, 191.2048835836134],
                                                       [44.97431755412259, 191.80294275526444],
                                                       [44.82746567811765, 192.40380927958526],
                                                       [44.67998311720149, 193.0074220526288],
                                                       [44.531755844020225, 193.61371997044847],
                                                       [44.3826698312199, 194.2226419290972],
                                                       [44.23261105144659, 194.83412682462819],
                                                       [44.0814654773464, 195.44811355309457],
                                                       [43.92911908156538, 196.06454101054962],
                                                       [43.775457836749624, 196.68334809304633],
                                                       [43.6203677155452, 197.30447369663787],
                                                       [43.463734690598194, 197.9278567173775],
                                                       [43.305444734554676, 198.55343605131833],
                                                       [43.14538382006071, 199.18115059451335],
                                                       [42.98343791976238, 199.81093924301587],
                                                       [42.819493006305784, 200.44274089287907],
                                                       [42.65343505233696, 201.0764944401559],
                                                       [42.485150030502005, 201.71213878089966],
                                                       [42.314523913447026, 202.34961281116355],
                                                       [42.14144267381806, 202.98885542700063],
                                                       [41.96579228426117, 203.62980552446393],
                                                       [41.787458717422474, 204.27240199960684],
                                                       [41.60632794594803, 204.91658374848242],
                                                       [41.42228594248391, 205.56228966714366],
                                                       [41.23521867967619, 206.20945865164384],
                                                       [41.04501213017097, 206.85802959803618],
                                                       [40.85155226661429, 207.50794140237375],
                                                       [40.654725061652236, 208.15913296070963],
                                                       [40.454416487930914, 208.81154316909712],
                                                       [40.25051251809637, 209.46511092358926],
                                                       [40.0428991247947, 210.11977512023918],
                                                       [39.831462280671964, 210.7754746551001],
                                                       [39.61608795837424, 211.43214842422515],
                                                       [39.39666213054761, 212.0897353236674],
                                                       [39.17307076983816, 212.74817424948014],
                                                       [38.94519984889194, 213.40740409771638],
                                                       [38.71293534035506, 214.06736376442933],
                                                       [38.47616321687357, 214.72799214567215],
                                                       [38.23476945109356, 215.38922813749798],
                                                       [37.9886400156611, 216.05101063595995],
                                                       [37.73766088322227, 216.71327853711125],
                                                       [37.48171802642315, 217.37597073700493],
                                                       [37.220697417909804, 218.03902613169421],
                                                       [36.95448503032832, 218.70238361723227],
                                                       [36.68296683632477, 219.36598208967223],
                                                       [36.40602880854524, 220.02976044506715],
                                                       [36.12355691963578, 220.6936575794703],
                                                       [35.835437142242505, 221.3576123889348],
                                                       [35.541555449011454, 222.02156376951373],
                                                       [35.24179781258874, 222.6854506172603],
                                                       [34.936050205620404, 223.34921182822768],
                                                       [34.62419860075254, 224.01278629846894],
                                                       [34.306128970631235, 224.67611292403728],
                                                       [33.981727287902544, 225.33913060098587],
                                                       [33.65087952521255, 226.0017782253678],
                                                       [33.313471655207344, 226.66399469323625],
                                                       [32.969389650532975, 227.3257189006443],
                                                       [32.61851948383554, 227.98688974364518],
                                                       [32.26074712776112, 228.64744611829207],
                                                       [31.895958554955783, 229.30732692063805],
                                                       [31.524039738065596, 229.96647104673622],
                                                       [31.14487664973666, 230.62481739263984],
                                                       [30.758355262615023, 231.28230485440196],
                                                       [30.364361549346775, 231.93887232807583],
                                                       [29.962781482577995, 232.5944587097145],
                                                       [29.553501034954756, 233.24900289537118],
                                                       [29.136406179123135, 233.90244378109895],
                                                       [28.71138288772921, 234.5547202629511],
                                                       [28.27831713341906, 235.20577123698058],
                                                       [27.837094888838752, 235.8555355992407],
                                                       [27.387602126634366, 236.5039522457845],
                                                       [26.929724819451987, 237.15096007266519],
                                                       [26.46334893993768, 237.79649797593592],
                                                       [25.988360460737532, 238.44050485164982],
                                                       [25.50464535449761, 239.08291959586],
                                                       [25.012089593863998, 239.7236811046197],
                                                       [24.510579151482766, 240.36272827398196], [24.0, 241.0]],
                                    'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {
                                    'control_points': [[24.0, 241.0], [23.475668142776698, 241.64770405892293],
                                                       [22.951336285553392, 242.2954081178458],
                                                       [22.427004428330086, 242.94311217676872],
                                                       [21.902672571106784, 243.59081623569162],
                                                       [21.378340713883482, 244.23852029461455],
                                                       [20.854008856660176, 244.88622435353744],
                                                       [20.32967699943687, 245.53392841246034],
                                                       [19.805345142213568, 246.18163247138324],
                                                       [19.281013284990266, 246.82933653030614],
                                                       [18.75668142776696, 247.47704058922903],
                                                       [18.232349570543658, 248.12474464815196],
                                                       [17.708017713320352, 248.77244870707486],
                                                       [17.18368585609705, 249.42015276599776],
                                                       [16.659353998873744, 250.06785682492065],
                                                       [16.13502214165044, 250.71556088384358],
                                                       [15.610690284427136, 251.36326494276648],
                                                       [15.086358427203834, 252.01096900168938],
                                                       [14.562026569980528, 252.65867306061227],
                                                       [14.037694712757226, 253.3063771195352],
                                                       [13.513362855533922, 253.9540811784581],
                                                       [12.989030998310616, 254.601785237381],
                                                       [12.464699141087312, 255.2494892963039],
                                                       [11.94036728386401, 255.89719335522682],
                                                       [11.416035426640704, 256.5448974141497]], 'width': 15,
                                    'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {
                                    'control_points': [[11.416035426640704, 256.5448974141497],
                                                       [9.94266476404094, 255.07198985820193],
                                                       [8.469294101441177, 253.5990823022541],
                                                       [6.995923438841412, 252.1261747463063],
                                                       [5.52255277624165, 250.65326719035852],
                                                       [4.049182113641886, 249.1803596344107],
                                                       [2.5758114510421217, 247.7074520784629],
                                                       [1.102440788442359, 246.23454452251508],
                                                       [-0.3709298741574045, 244.7616369665673],
                                                       [-1.844300536757169, 243.28872941061948],
                                                       [-3.3176711993569317, 241.8158218546717],
                                                       [-4.791041861956695, 240.34291429872388],
                                                       [-6.264412524556461, 238.87000674277607],
                                                       [-7.737783187156223, 237.39709918682826],
                                                       [-9.211153849755984, 235.92419163088044],
                                                       [-10.684524512355752, 234.45128407493266],
                                                       [-12.157895174955515, 232.97837651898485],
                                                       [-13.631265837555276, 231.50546896303706],
                                                       [-15.104636500155042, 230.03256140708925],
                                                       [-16.578007162754805, 228.55965385114143],
                                                       [-18.051377825354567, 227.08674629519365],
                                                       [-19.524748487954334, 225.61383873924584],
                                                       [-20.998119150554096, 224.14093118329805],
                                                       [-22.47148981315386, 222.66802362735024],
                                                       [-23.944860475753625, 221.19511607140242]], 'width': 10,
                                    'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {
                                    'control_points': [[11.416035426640704, 256.5448974141497],
                                                       [12.889406089240467, 258.01780497009753],
                                                       [14.36277675184023, 259.4907125260453],
                                                       [15.836147414439992, 260.9636200819931],
                                                       [17.309518077039755, 262.4365276379409],
                                                       [18.782888739639517, 263.9094351938887],
                                                       [20.25625940223928, 265.3823427498365],
                                                       [21.729630064839043, 266.8552503057843],
                                                       [23.203000727438805, 268.3281578617321],
                                                       [24.676371390038568, 269.8010654176799],
                                                       [26.14974205263833, 271.27397297362774],
                                                       [27.623112715238094, 272.7468805295755],
                                                       [29.096483377837856, 274.2197880855233],
                                                       [30.56985404043762, 275.6926956414711],
                                                       [32.04322470303738, 277.1656031974189],
                                                       [33.516595365637144, 278.6385107533667],
                                                       [34.98996602823691, 280.1114183093145],
                                                       [36.46333669083666, 281.5843258652623],
                                                       [37.93670735343643, 283.0572334212101],
                                                       [39.410078016036195, 284.5301409771579],
                                                       [40.88344867863596, 286.0030485331057],
                                                       [42.35681934123572, 287.4759560890535],
                                                       [43.83019000383548, 288.9488636450013],
                                                       [45.303560666435246, 290.4217712009491],
                                                       [46.77693132903501, 291.8946787568969]], 'width': 10,
                                    'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {
                                    'control_points': [[46.77693132903501, 291.8946787568969],
                                                       [46.88019464894375, 292.1684698805646],
                                                       [46.98345796885248, 292.4422610042323],
                                                       [47.08672128876122, 292.71605212790007],
                                                       [47.18998460866996, 292.9898432515677],
                                                       [47.2932479285787, 293.2636343752355],
                                                       [47.396511248487435, 293.53742549890313],
                                                       [47.49977456839617, 293.81121662257084],
                                                       [47.60303788830491, 294.08500774623855],
                                                       [47.70630120821364, 294.35879886990625],
                                                       [47.80956452812238, 294.63258999357396],
                                                       [47.91282784803112, 294.90638111724166],
                                                       [48.016091167939855, 295.1801722409094],
                                                       [48.1193544878486, 295.45396336457713],
                                                       [48.22261780775733, 295.72775448824484],
                                                       [48.325881127666065, 296.0015456119125],
                                                       [48.4291444475748, 296.2753367355802],
                                                       [48.53240776748354, 296.54912785924796],
                                                       [48.63567108739228, 296.8229189829156],
                                                       [48.73893440730102, 297.0967101065834],
                                                       [48.84219772720976, 297.3705012302511],
                                                       [48.94546104711849, 297.6442923539188],
                                                       [49.04872436702723, 297.9180834775865],
                                                       [49.15198768693596, 298.1918746012542],
                                                       [49.2552510068447, 298.4656657249219],
                                                       [49.358514326753436, 298.73945684858955],
                                                       [49.46177764666218, 299.0132479722573],
                                                       [49.56504096657092, 299.287039095925],
                                                       [49.66830428647965, 299.56083021959273],
                                                       [49.77156760638839, 299.83462134326044],
                                                       [49.87483092629713, 300.10841246692814],
                                                       [49.97809424620586, 300.38220359059585],
                                                       [50.0813575661146, 300.65599471426356],
                                                       [50.18462088602334, 300.92978583793126],
                                                       [50.28788420593208, 301.20357696159897],
                                                       [50.391147525840815, 301.4773680852667],
                                                       [50.49441084574955, 301.7511592089344],
                                                       [50.59767416565829, 302.0249503326021],
                                                       [50.700937485567025, 302.2987414562698],
                                                       [50.80420080547576, 302.5725325799375],
                                                       [50.90746412538451, 302.8463237036052],
                                                       [51.01072744529324, 303.12011482727297],
                                                       [51.113990765201976, 303.3939059509406],
                                                       [51.21725408511071, 303.6676970746083],
                                                       [51.32051740501945, 303.9414881982761],
                                                       [51.423780724928186, 304.21527932194374],
                                                       [51.52704404483693, 304.48907044561145],
                                                       [51.63030736474566, 304.76286156927915],
                                                       [51.733570684654396, 305.03665269294686],
                                                       [51.83683400456313, 305.31044381661457],
                                                       [51.94009732447187, 305.5842349402823],
                                                       [52.04336064438061, 305.85802606395],
                                                       [52.14662396428935, 306.1318171876177],
                                                       [52.24988728419808, 306.4056083112854],
                                                       [52.35315060410682, 306.67939943495315],
                                                       [52.456413924015564, 306.9531905586208],
                                                       [52.5596772439243, 307.22698168228857],
                                                       [52.66294056383303, 307.5007728059563],
                                                       [52.76620388374177, 307.774563929624],
                                                       [52.869467203650515, 308.0483550532917],
                                                       [52.97273052355925, 308.3221461769594],
                                                       [53.075993843467984, 308.5959373006271],
                                                       [53.179257163376725, 308.8697284242948],
                                                       [53.28252048328546, 309.1435195479625],
                                                       [53.385783803194194, 309.4173106716302],
                                                       [53.48904712310293, 309.6911017952979],
                                                       [53.59231044301167, 309.96489291896563],
                                                       [53.695573762920404, 310.23868404263334],
                                                       [53.798837082829145, 310.51247516630104],
                                                       [53.90210040273788, 310.78626628996875],
                                                       [54.00536372264662, 311.06005741363646],
                                                       [54.108627042555355, 311.33384853730416],
                                                       [54.21189036246409, 311.60763966097187],
                                                       [54.31515368237284, 311.88143078463963],
                                                       [54.41841700228157, 312.1552219083073],
                                                       [54.521680322190306, 312.42901303197505],
                                                       [54.62494364209904, 312.70280415564275],
                                                       [54.72820696200778, 312.9765952793104],
                                                       [54.831470281916516, 313.25038640297817],
                                                       [54.93473360182525, 313.5241775266459],
                                                       [55.03799692173399, 313.7979686503136],
                                                       [55.141260241642726, 314.0717597739812],
                                                       [55.24452356155147, 314.345550897649],
                                                       [55.3477868814602, 314.61934202131664],
                                                       [55.45105020136894, 314.8931331449844],
                                                       [55.55431352127768, 315.1669242686521],
                                                       [55.65757684118641, 315.4407153923198],
                                                       [55.76084016109515, 315.7145065159875],
                                                       [55.864103481003895, 315.98829763965523],
                                                       [55.96736680091263, 316.26208876332294],
                                                       [56.07063012082136, 316.5358798869906],
                                                       [56.173893440730104, 316.80967101065835],
                                                       [56.27715676063884, 317.08346213432606],
                                                       [56.38042008054757, 317.35725325799376],
                                                       [56.483683400456314, 317.63104438166147],
                                                       [56.586946720365056, 317.9048355053292],
                                                       [56.69021004027379, 318.1786266289968],
                                                       [56.793473360182524, 318.4524177526646],
                                                       [56.896736680091266, 318.7262088763323], [57.0, 319.0]],
                                    'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}],
                      'type': 'urban', 'file_name': 'urban', 'score': 0,
                      'obstacles': [{'name': 'trafficlightdouble', 'position': (55.03, -6.2), 'zRot': 360},
                                    {'name': 'trafficlightdouble', 'position': (57.97980503159698, 6.871452728907405),
                                     'zRot': 264},
                                    {'name': 'trafficlightdouble', 'position': (73.53999999999999, 6.2), 'zRot': 180},
                                    {'name': 'trafficlightdouble', 'position': (70.58007922210668, -8.08472113773959),
                                     'zRot': 433},
                                    {'name': 'trafficlightdouble', 'position': (53.77570678554149, 144.33254611648698),
                                     'zRot': 446},
                                    {'name': 'trafficlightdouble', 'position': (38.21542061660158, 145.98540900615114),
                                     'zRot': 351},
                                    {'name': 'trafficlightdouble', 'position': (39.03210657328211, 154.3807475864549),
                                     'zRot': 266},
                                    {'name': 'trafficlightdouble', 'position': (55.12189363662843, 155.11504733627703),
                                     'zRot': 192},
                                    {'name': 'trafficlightdouble', 'position': (20.60973189729495, 257.4257749342849),
                                     'zRot': 489},
                                    {'name': 'trafficlightsingle', 'position': (9.052771666157474, 246.8296215869595),
                                     'zRot': 405},
                                    {'name': 'trafficlightsingle', 'position': (13.779299187123936, 266.26017324133994),
                                     'zRot': 225}], 'success_point': (57, 319),
                      'ego_lanes': [0, 1, 4, 5, 6, 9, 10, 11, 13, 14], 'directions': ['left', 'straight', 'right'],
                      'fitness': 0}]
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
#       TODO Add other participants
#       TODO Mutation
#       TODO Repair function
#       TODO Mutation validity checks
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
#       TODO Double test cases by placing spawn point on the other side
#       TODO Improve performance
#       TODO Make all objects collidable
#       TODO Converter:
#           TODO Add input checking
#           TODO Implement Sensor deployment
#       TODO Control traffic lights (not possible)
#       TODO Parked cars on the road and adjust waypoints
#       TODO Improve speed of car
#       TODO Improve traffic sign positioning
#       TODO Place trigger on right lane instead of middle of road
#       TODO ai_set_line via Lua
#       TODO Test generator should calc speed of waypoints, not converter
#       TODO Main lane and append side lanes instead of explicitly adding intersection

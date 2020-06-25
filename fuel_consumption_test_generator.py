from copy import deepcopy
from random import randint, random

import numpy as np
from scipy.interpolate import splev
from shapely import affinity
from shapely.geometry import LineString, shape
from termcolor import colored
from os import path
from glob import glob
from pathlib import Path

from utils.plotter import plot_all
from utils.utility_functions import convert_points_to_lines, convert_splines_to_lines, get_angle, calc_width, \
    calc_min_max_angles, get_lanes_of_intersection, get_intersection_lines, get_width_lines, \
    get_resize_factor_intersection
from utils.validity_checks import intersection_check_width, intersection_check_last
from utils.xml_creator import build_all_xml

MIN_DEGREES = 70
MAX_DEGREES = 290


def _add_ego_car(individual):
    """Adds the ego car to the criteria xml file. Movement mode can be assigned manually. Each control point is one
    waypoint.
    :param individual: Individual of the population.
    :return: Void.
    """
    # TODO Turning requires lane switch for multiple lanes.
    # TODO Adopt to different number of lanes.
    lanes = individual.get("lanes")
    ego_lanes = individual.get("ego_lanes")
    waypoints = []
    for lane in ego_lanes:
        iterator = 4
        control_points = lanes[lane].get("control_points")
        while iterator < len(control_points):
            waypoint = {"x": control_points[iterator].get("x"),
                        "y": control_points[iterator].get("y"),
                        "tolerance": 3,
                        "movementMode": "_BEAMNG"}
            waypoints.append(waypoint)
            iterator += 4
    y = (lanes[0].get("left_lanes") + lanes[0].get("right_lanes") - 1) * -2.5 + waypoints[0].get("y")
    init_state = {"x": lanes[0].get("control_points")[0].get("x"),
                  "y": str(y),
                  "orientation": 0,
                  "movementMode": "_BEAMNG",
                  "speed": 50}
    model = "ETK800"
    ego = {"id": "ego",
           "init_state": init_state,
           "waypoints": waypoints,
           "model": model}
    participants = [ego]
    individual["participants"] = participants


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


class FuelConsumptionTestGenerator:

    def __init__(self):
        self.files_name = "urban"
        self.SPLINE_DEGREE = 3  # Sharpness of curves
        self.MAX_TRIES = 60  # Maximum number of invalid generated points/segments
        self.POPULATION_SIZE = 1  # Minimum number of generated roads for each generation
        self.NUMBER_ELITES = 2  # Number of best kept roads
        self.MIN_SEGMENT_LENGTH = 15  # Minimum length of a road segment
        self.MAX_SEGMENT_LENGTH = 30  # Maximum length of a road segment
        self.MIN_NODES = 6  # Minimum number of control points for each road
        self.MAX_NODES = 12  # Maximum number of control points for each road
        self.population_list = []
        self.intersection_length = 50
        self.opposite_lane = 30
        self.intersecting_length = 20
        self.MAX_LEFT_LANES = 2
        self.MAX_RIGHT_LANES = 2
        self.MAX_WIDTH = 5

    def _bspline(self, lanes):
        """Calculate {@code samples} samples on a bspline. This is the road representation function.
        :param lanes: List of lanes.
        :return: List of arrays with samples, representing a bspline of the given control points of the lanes.
        """
        splined_list = []
        for lane in lanes:
            samples = lane.get("samples")
            # Calculate splines for each lane.
            point_list = []
            for point in lane.get("control_points"):
                point_list.append((point.get("x"), point.get("y")))
            point_list = np.asarray(point_list)
            count = len(point_list)
            degree = np.clip(self.SPLINE_DEGREE, 1, count - 1)

            # Calculate knot vector.
            kv = np.concatenate(([0] * degree, np.arange(count - degree + 1), [count - degree] * degree))

            # Calculate query range.
            u = np.linspace(False, (count - degree), samples)

            # Calculate result.
            splined_list.append({"control_points": np.array(splev(u, (kv, point_list.T, degree))).T,
                                 "width": lane.get("width")})
        return splined_list

    def _add_segment(self, last_point, penultimate_point=None):
        """Generates a new random point within a given range.
        :param last_point: Last point of the control point list as dict type.
        :param penultimate_point: Point before the last point as dict type.
        :return: A new random point as dict type.
        """
        last_point_tmp = (last_point.get("x"), last_point.get("y"))
        last_point_tmp = np.asarray(last_point_tmp)
        x_min = int(round(last_point.get("x") - self.MAX_SEGMENT_LENGTH))
        x_max = int(round(last_point.get("x") + self.MAX_SEGMENT_LENGTH))
        y_min = int(round(last_point.get("y") - self.MAX_SEGMENT_LENGTH))
        y_max = int(round(last_point.get("y") + self.MAX_SEGMENT_LENGTH))
        while True:
            x_pos = randint(x_min, x_max)
            y_pos = randint(y_min, y_max)
            point = (x_pos, y_pos)
            dist = np.linalg.norm(np.asarray(point) - last_point_tmp)
            if penultimate_point is not None:
                deg = get_angle((penultimate_point.get("x"), penultimate_point.get("y")),
                                (last_point.get("x"), last_point.get("y")),
                                point)
            if self.MAX_SEGMENT_LENGTH >= dist >= self.MIN_SEGMENT_LENGTH:
                if penultimate_point is not None:
                    if MIN_DEGREES <= deg <= MAX_DEGREES:
                        return {"x": point[0], "y": point[1], "type": "segment"}
                else:
                    return {"x": point[0], "y": point[1], "type": "segment"}

    def _create_urban_environment(self):
        global MIN_DEGREES, MAX_DEGREES
        print(colored("Creating urban scenario...", "grey", attrs=['bold']))
        p0 = {"x": 1, "y": 0, "type": "segment"}
        p1 = {"x": 50, "y": 0, "type": "segment"}
        p2 = {"x": 65, "y": 0, "type": "segment"}
        left_lanes = randint(1, self.MAX_LEFT_LANES)
        right_lanes = randint(1, self.MAX_RIGHT_LANES)
        lanes = [{"control_points": [p0, p1, p2], "width": calc_width(left_lanes, right_lanes),
                  "left_lanes": left_lanes, "right_lanes": right_lanes, "samples": 75}]
        ego_lanes = [0]
        intersection_lanes = []
        obstacles = []
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
                control_points_lines = convert_splines_to_lines(temp_list)
                width_lines = get_width_lines(temp_list)
                intersection_lanes_temp = deepcopy(intersection_lanes)
                intersection_lanes_temp.extend(intersection_items.get("intersection_lanes"))
                if not intersection_check_last(lines_of_roads, new_line) \
                        and not intersection_check_last(lines_of_roads, new_lane_line) \
                        and not intersection_check_width(width_lines, control_points_lines, intersection_lanes_temp):
                    left_lanes = intersection_items.get("left_lanes")
                    right_lanes = intersection_items.get("right_lanes")
                    obstacles.append(self._add_traffic_sign(control_points[-1], intersection.get("intersection_point"),
                                                            lanes[lane_index].get("left_lanes"),
                                                            lanes[lane_index].get("right_lanes"),
                                                            left_lanes, right_lanes))
                    lanes[lane_index].get("control_points")[-1]["type"] = "intersection"
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
            new_line = LineString([(control_points[-1].get("x"), control_points[-1].get("y")),
                                   (new_point.get("x"), new_point.get("y"))])
            temp_list = deepcopy(lanes)
            temp_list[lane_index].get("control_points").append(new_point)
            temp_list = self._bspline(temp_list)
            control_points_lines = convert_splines_to_lines(temp_list)
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
            return {"lanes": lanes, "success_point": last_point, "ego_lanes": ego_lanes, "obstacles": obstacles}
        else:
            print(colored("Couldn't create a valid road network. Restarting...", "grey", attrs=['bold']))

    def _repair(self):
        pass

    def _add_traffic(self):
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
        line = LineString([(penultimate_point.get("x"), penultimate_point.get("y")),
                           (last_point.get("x"), last_point.get("y"))])
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
        return {"intersection_point": {"x": intersection_point[0], "y": intersection_point[1], "type": "intersection"},
                "straight_point": {"x": straight_point[0], "y": straight_point[1], "type": "intersection"},
                "left_point": {"x": p1[0], "y": p1[1], "type": "intersection"},
                "right_point": {"x": p2[0], "y": p2[1], "type": "intersection"},
                "direction": direction,
                "number_of_ways": number_of_ways,
                "layout": layout,
                "new_left_lanes": left_lanes,
                "new_right_lanes": right_lanes,
                "new_width": calc_width(left_lanes, right_lanes)}

    def _add_traffic_sign(self, last_point, intersection_point, current_left_lanes, current_right_lanes,
                          new_left_lanes, new_right_lanes):
        # Calculate traffic sign position.
        line = LineString([(intersection_point.get("x"), intersection_point.get("y")),
                           (last_point.get("x"), last_point.get("y"))])
        temp_line = LineString([(intersection_point.get("x"), intersection_point.get("y")),
                                (intersection_point.get("x") + 5, intersection_point.get("y"))])
        angle = int(round(get_angle(temp_line.coords[1], line.coords[0], line.coords[1]))) + 180
        fac = (self.MAX_WIDTH * (current_left_lanes + current_right_lanes) / 2 + 0.5) / line.length
        vector = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        vector = affinity.rotate(vector, 90, line.coords[0])
        fac = (self.MAX_WIDTH * (new_left_lanes + new_right_lanes) / 2 + 2) / vector.length
        vector2 = LineString([(vector.coords[1][0], vector.coords[1][1]),
                              (vector.coords[0][0], vector.coords[0][1])])
        vector2 = affinity.scale(vector2, xfact=fac, yfact=fac, origin=vector2.coords[0])
        vector2 = affinity.rotate(vector2, 90, vector2.coords[0])
        position = vector2.coords[1]

        if current_left_lanes + current_right_lanes == 2:
            if random() <= 0.5:
                return {"name": "stopsign", "x": position[0], "y": position[1], "zRot": angle}
            else:
                return {"name": "trafficlightsingle", "x": position[0], "y": position[1], "zRot": angle}
        else:
            return {"name": "trafficlightdouble", "x": position[0], "y": position[1], "zRot": angle}

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
                control_points = []
                for spline in lane.get("control_points"):
                    point = {"x": spline[0], "y": spline[1]}
                    control_points.append(point)
                individual.get("lanes")[iterator]["control_points"] = control_points
                iterator += 1
            _add_ego_car(individual)
        return population

    def _create_start_population(self):
        """Creates and returns an initial population."""
        startpop = []
        iterator = 0
        while len(startpop) < self.POPULATION_SIZE:
            urban = self._create_urban_environment()
            if urban is not None:
                individual = {"lanes": urban.get("lanes"),
                              "type": "urban",
                              "file_name": self.files_name,
                              "score": 0,
                              "obstacles": urban.get("obstacles"),
                              "success_point": urban.get("success_point"),
                              "ego_lanes": urban.get("ego_lanes")}
                startpop.append(individual)
                iterator += 1
        return startpop

    def genetic_algorithm(self):
        if len(self.population_list) == 0:
            self.population_list = self._create_start_population()
        print(colored("Population finished.", "grey", attrs=['bold']))
        temp_list = deepcopy(self.population_list)
        plot_all(temp_list)
        temp_list = self._spline_population(temp_list)
        temp_list = _merge_lanes(temp_list)
        build_all_xml(temp_list)

        # Comment out if you want to see the generated roads (blocks until you close all images).

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

# TODO  Desired features:
#       TODO Lane switch when turning for multiple lanes
#       TODO Calculate parallel coords for waypoints (shapely's parallel offset)
#       TODO Add other participants
#       TODO Control traffic lights
#       TODO Parked cars
#       TODO Create init population
#       TODO Mutation
#       TODO Repair function
#       TODO Mutation validity checks
#       TODO Crossover
#       TODO Fix lane markings
#       TODO Improve performance
#       TODO Double test cases by placing spawn point on the other side
#       TODO Fix/Improve waypoints
#       TODO Signs on opposite lanes
#       TODO Improve traffic signs positioning
#       TODO Add traffic signs for opposite lanes
#       TODO Add yield sign
#       TODO Implement own converter
#       TODO Add weather presets
#       TODO Daytime
#       TODO One width per lane
#       TODO Add drivebuild level folder

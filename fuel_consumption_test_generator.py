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
from time import time

from utils.plotter import plotter, plot_all, plot_splines_and_width
from utils.utility_functions import convert_points_to_lines, convert_splines_to_lines, get_angle, calc_width,\
    calc_min_max_angles, get_lanes_of_intersection, get_intersection_lines
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
        iterator = 0
        control_points = lanes[lane].get("control_points")
        while iterator < len(control_points):
            waypoint = {"x": control_points[iterator].get("x"),
                        "y": control_points[iterator].get("y"),
                        "tolerance": 2,
                        "movementMode": "_BEAMNG"}
            waypoints.append(waypoint)
            iterator += 9
    init_state = {"x": waypoints[0].get("x"),
                  "y": waypoints[0].get("y"),
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
        self.population_list_urban = []
        self.population_list_highway = []
        self.intersection_length = 75
        self.opposite_lane = 50
        self.intersecting_length = 40
        self.MAX_LEFT_LANES = 2
        self.MAX_RIGHT_LANES = 2

    def _bspline(self, lanes):
        """Calculate {@code samples} samples on a bspline. This is the road representation function.
        :param lanes: List of lanes.
        :param samples: Number of samples of each lane.
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
        p0 = {"x": 1, "y": 0, "type": "segment"}
        p1 = {"x": 50, "y": 0, "type": "segment"}
        p2 = {"x": 65, "y": 0, "type": "segment"}
        left_lanes = randint(1, self.MAX_LEFT_LANES)
        right_lanes = randint(1, self.MAX_RIGHT_LANES)
        lanes = [{"control_points": [p0, p1, p2], "width": calc_width(left_lanes, right_lanes),
                  "left_lanes": left_lanes, "right_lanes": right_lanes, "samples": 75}]
        ego_lanes = [0]
        intersection_lanes = []
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
                                          or random() <= intersection_probability)\
                                     and len(control_points) > 1:
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
                width_lines = self._get_width_lines(temp_list)
                intersection_lanes_temp = deepcopy(intersection_lanes)
                intersection_lanes_temp.extend(intersection_items.get("intersection_lanes"))
                if not intersection_check_last(lines_of_roads, new_line) \
                        and not intersection_check_last(lines_of_roads, new_lane_line)\
                        and not intersection_check_width(width_lines, control_points_lines, intersection_lanes_temp):
                    lanes[lane_index].get("control_points")[-1]["type"] = "intersection"
                    lanes.extend(intersection_items.get("lanes"))
                    ego_lanes.extend(intersection_items.get("ego_lanes"))
                    last_point = intersection_items.get("last_points")
                    left_lanes = intersection_items.get("left_lanes")
                    right_lanes = intersection_items.get("right_lanes")
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
            width_lines = self._get_width_lines(temp_list)
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
            plotter(lanes)
            print(colored("Finished creating urban scenario!", "grey", attrs=['bold']))
            return {"lanes": lanes, "success_point": last_point, "ego_lanes": ego_lanes}
        else:
            print(colored("Couldn't create a valid road network. Restarting...", "grey", attrs=['bold']))

    def _repair(self):
        pass

    def _add_traffic(self):
        pass

    def _get_width_lines(self, splined_lanes):
        """Determines the width lines of the road by flipping the LineString
         between two points by 90 degrees in both directions.
        :param control_points: List of control points.
        :return: List of LineStrings which represent the width of the road.
        """
        complete_width_list = []
        for spline_list in splined_lanes:
            linestring_list = []
            iterator = 0

            # Triple width to have more space between road pieces.
            width = spline_list.get("width") * 3
            control_points = spline_list.get("control_points")
            while iterator < (len(control_points) - 1):
                p1 = (control_points[iterator][0], control_points[iterator][1])
                p2 = (control_points[iterator + 1][0], control_points[iterator + 1][1])
                line = LineString([p1, p2])

                # Rotate counter-clockwise and resize to the half of the road length.
                line_rot1 = affinity.rotate(line, 90, line.coords[0])
                line_rot1 = affinity.scale(line_rot1, xfact=self._get_resize_factor(line_rot1.length, width),
                                           yfact=self._get_resize_factor(line_rot1.length, width),
                                           origin=line_rot1.coords[0])

                # Rotate clockwise and resize to the half of the road length.
                line_rot2 = affinity.rotate(line, -90, line.coords[0])
                line_rot2 = affinity.scale(line_rot2, xfact=self._get_resize_factor(line_rot2.length, width),
                                           yfact=self._get_resize_factor(line_rot2.length, width),
                                           origin=line_rot2.coords[0])

                line = LineString([line_rot1.coords[1], line_rot2.coords[1]])
                linestring_list.append(line)

                if iterator == len(control_points) - 2:
                    line = LineString([p1, p2])
                    line_rot1 = affinity.rotate(line, -90, line.coords[1])
                    line_rot1 = affinity.scale(line_rot1, xfact=self._get_resize_factor(line_rot1.length, width),
                                               yfact=self._get_resize_factor(line_rot1.length, width),
                                               origin=line_rot1.coords[0])

                    line_rot2 = affinity.rotate(line, 90, line.coords[1])
                    line_rot2 = affinity.scale(line_rot2, xfact=self._get_resize_factor(line_rot2.length, width),
                                               yfact=self._get_resize_factor(line_rot2.length, width),
                                               origin=line_rot2.coords[0])
                    line = LineString([line_rot1.coords[1], line_rot2.coords[1]])
                    line = affinity.scale(line, xfact=self._get_resize_factor(line.length, width)*2,
                                          yfact=self._get_resize_factor(line.length, width)*2)
                    linestring_list.append(line)
                iterator += 1
            complete_width_list.append(linestring_list)
        return complete_width_list

    def _get_resize_factor(self, length, width):
        """Returns the resize factor for the width lines so all lines have
        one specific length.
        :param length: Length of a LineString.
        :return: Resize factor.
        """
        if length == 0:
            return 0
        return width / length

    def _get_resize_factor_intersection(self, linestring_length, intersection_length):
        if linestring_length == 0:
            return 0
        return (linestring_length + intersection_length) / linestring_length

    def _create_intersection(self, last_point, penultimate_point):
        # TODO Check for number of lanes. Stop signs only for single lane roads?
        # TODO Add traffic lights/stop signs
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
        """
        if random() <= 0.5:
            self._add_stop_sign()
        else:
            self._add_traffic_lights()
        """
        line = LineString([(penultimate_point.get("x"), penultimate_point.get("y")),
                          (last_point.get("x"), last_point.get("y"))])
        fac = self._get_resize_factor_intersection(line.length, self.intersection_length)
        line_intersection = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        new_point = list(shape(line_intersection).coords)[1]
        fac = self._get_resize_factor_intersection(line.length, self.intersecting_length)
        line_intersection = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        intersection_point = list(shape(line_intersection).coords)[1]
        line = LineString([(intersection_point[0], intersection_point[1]),
                          (new_point[0], new_point[1])])

        # Right turn.
        line_rot1 = affinity.rotate(line, -90, line.coords[0])
        line_rot1 = affinity.scale(line_rot1, xfact=3, yfact=3,
                                   origin=line_rot1.coords[0])

        # Left turn.
        line_rot2 = affinity.rotate(line, 90, line.coords[0])
        line_rot2 = affinity.scale(line_rot2, xfact=3, yfact=3,
                                   origin=line_rot2.coords[0])
        p1 = (list(shape(line_rot1).coords)[1][0], list(shape(line_rot1).coords)[1][1])     # Right side point.
        p2 = (list(shape(line_rot2).coords)[1][0], list(shape(line_rot2).coords)[1][1])     # Left side point.
        left_lanes = randint(1, self.MAX_LEFT_LANES)
        right_lanes = randint(1, self.MAX_RIGHT_LANES)
        return {"intersection_point": {"x": intersection_point[0], "y": intersection_point[1], "type": "intersection"},
                "straight_point": {"x": new_point[0], "y": new_point[1], "type": "intersection"},
                "left_point": {"x": p2[0], "y": p2[1], "type": "intersection"},
                "right_point": {"x": p1[0], "y": p1[1], "type": "intersection"},
                "direction": direction,
                "number_of_ways": number_of_ways,
                "layout": layout,
                "new_left_lanes": left_lanes,
                "new_right_lanes": right_lanes,
                "new_width": calc_width(left_lanes, right_lanes)}

    def _add_stop_sign(self):
        print("added stop sign")

    def _add_traffic_lights(self):
        print("added traffic lights")

    def _add_highway(self):
        pass

    def _spline_population(self, population_list, samples=75):
        """Converts the control points list of every individual to a bspline
         list and adds the width parameter as well as the ego car.
        :param population_list: List of individuals.
        :param samples: Number of samples for b-spline interpolation.
        :return: List of individuals with bsplined control points.
        """
        for individual in population_list:
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
        return population_list

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
        if len(self.population_list_urban) == 0:
            self.population_list_urban = self._create_start_population()

        print(colored("Population finished.", "grey", attrs=['bold']))
        temp_list = deepcopy(self.population_list_urban)
        temp_list = self._spline_population(temp_list)
        build_all_xml(temp_list)

        # Comment out if you want to see the generated roads (blocks until you close all images).
        #plot_all(temp_list)
        self.population_list_urban = []

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
#       TODO Find out why some individuals dont have a intersection
#       TODO Reduce opposite lanes
#       TODO Reduce segment length
#       TODO Reduce intersection length
#       TODO New lanes are cut
#       TODO Validate depending on layout
#       TODO Add different angles of intersection
#       TODO Add more types of intersections
#       TODO Adding traffic signs and lights(depending on num lanes)
#       TODO Highways
#       TODO Create init population
#       TODO Mutation
#       TODO -> Init state, Lanes austauschen, Init state of cars and speed, final position
#       TODO Repair function
#       TODO Mutation validity checks
#       TODO Crossover
#       TODO Calculate parallel coords for waypoints (shapely's parallel offset)
#       TODO Lane switch when turning for multiple lanes
#       TODO Control traffic lights
#       TODO Fix lane markings
#       TODO Add other participants
#       TODO Improve performance
#       TODO Refactoring
#       TODO Parked cars

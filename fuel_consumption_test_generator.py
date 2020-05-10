from copy import deepcopy
from math import degrees, atan2
from random import randint, random

import numpy as np
import scipy.interpolate as si
from shapely import affinity
from shapely.geometry import LineString, shape
from termcolor import colored

from utils.plotter import plot_splines_and_width, plotter, plot_all
from utils.utility_functions import convert_points_to_lines, convert_splines_to_lines
from utils.validity_checks import spline_intersection_check, intersection_check_all_np, intersection_check_width, \
    intersection_check_last
from utils.xml_creator import build_all_xml

MIN_DEGREES = 70
MAX_DEGREES = 290


def _add_ego_car(individual):
    """Adds the ego car to the criteria xml file. Movement mode can be assigned manually. Each control point is one
    waypoint.
    :param individual: Individual of the population.
    :return: Void.
    """
    lanes = individual.get("lanes")
    ego_lanes = individual.get("ego_lanes")
    waypoints = []
    for lane in ego_lanes:
        for point in lanes[lane].get("control_points"):
            waypoint = {"x": point.get("x"),
                        "y": point.get("y"),
                        "tolerance": 2,
                        "movementMode": "_BEAMNG"}
            waypoints.append(waypoint)
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


def get_angle(a, b, c):
    """Returns the angle between three points (two lines so to say).
    :param a: First point.
    :param b: Second point.
    :param c: Third point.
    :return: Angle in degrees.
    """
    ang = degrees(atan2(c[1] - b[1], c[0] - b[0]) - atan2(a[1] - b[1], a[0] - b[0]))
    return ang + 360 if ang < 0 else ang


class FuelConsumptionTestGenerator:

    def __init__(self):
        self.files_name = "urban"
        self.SPLINE_DEGREE = 3  # Sharpness of curves
        self.MAX_TRIES = 600  # Maximum number of invalid generated points/segments
        self.POPULATION_SIZE = 1  # Minimum number of generated roads for each generation
        self.NUMBER_ELITES = 2  # Number of best kept roads
        self.MIN_SEGMENT_LENGTH = 55  # Minimum length of a road segment
        self.MAX_SEGMENT_LENGTH = 80  # Maximum length of a road segment
        self.WIDTH_OF_STREET = 4  # Width of all segments
        self.MIN_NODES = 6  # Minimum number of control points for each road
        self.MAX_NODES = 20  # Maximum number of control points for each road
        self.population_list_urban = []
        self.population_list_highway = []
        self.intersection_length = 150
        self.opposite_lane = 100
        self.intersecting_length = 80

    def _bspline(self, lanes, samples=75):
        """Calculate {@code samples} samples on a bspline. This is the road representation function.
        :param control_points: List of control points.
        :param samples: Number of samples to return.
        :return: Array with samples, representing a bspline of the given function as a numpy array.
        """
        splined_list = []
        for lane in lanes:
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
            splined_list.append({"control_points": np.array(si.splev(u, (kv, point_list.T, degree))).T,
                                "width": lane.get("width")})
        return splined_list

    def _add_segment(self, last_point, penultimate_point):
        """Generates a random point within a given range.
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
        tries = 0
        while tries < self.MAX_TRIES / 5:
            x_pos = randint(x_min, x_max)
            y_pos = randint(y_min, y_max)
            point = (x_pos, y_pos)
            # TODO Min und Max angle depenend on number of lanes
            deg = get_angle((penultimate_point.get("x"), penultimate_point.get("y")),
                            (last_point.get("x"), last_point.get("y")),
                            point)
            dist = np.linalg.norm(np.asarray(point) - last_point_tmp)
            if (self.MAX_SEGMENT_LENGTH >= dist >= self.MIN_SEGMENT_LENGTH) and (MIN_DEGREES <= deg <= MAX_DEGREES):
                return {"x": point[0], "y": point[1], "type": "segment"}
            tries += 1

    def _create_urban_environment(self):
        p0 = {"x": 1, "y": 0, "type": "segment"}
        p1 = {"x": 50, "y": 0, "type": "segment"}
        p2 = {"x": 65, "y": 0, "type": "segment"}
        # TODO number of lanes random and width too.
        lanes = [{"control_points": [p0, p1, p2], "width": 8, "left_lanes": 1, "right_lanes": 1}]
        ego_lanes = [0]
        tries = 0
        lane_index = 0
        number_of_pieces = 3
        one_intersection = False
        intersection_possible = True
        intersection_probability = 0.25
        lines_of_roads = convert_points_to_lines(lanes)
        last_point = p2
        while number_of_pieces <= self.MAX_NODES and tries <= self.MAX_TRIES:
            if intersection_possible and ((number_of_pieces == self.MAX_NODES - 1 and not one_intersection)
                                          or random() <= intersection_probability):
                # Add intersection, if possible.
                control_points = lanes[lane_index].get("control_points")
                intersection = self._add_intersection(control_points[-1], control_points[-2])
                new_point = intersection[1]
                new_line = LineString([(control_points[-1].get("x"), control_points[-1].get("y")),
                                       (new_point.get("x"), new_point.get("y"))])
                new_lane_line = LineString([(intersection[2].get("x"), intersection[2].get("y")),
                                            (intersection[3].get("x"), intersection[3].get("y"))])
                temp_list = deepcopy(lanes)
                temp_list.append({"control_points": [intersection[2], intersection[3]],
                                  "width": 8, "left_lanes": 1, "right_lanes": 1})
                temp_list[lane_index].get("control_points").append(new_point)
                temp_list = self._bspline(temp_list)
                control_points_lines = convert_splines_to_lines(temp_list)
                width_lines = self._get_width_lines(temp_list)
                if not intersection_check_last(lines_of_roads, new_line, max_intersections=0) \
                        and not intersection_check_last(lines_of_roads, new_lane_line, max_intersections=0)\
                        and not intersection_check_width(width_lines, control_points_lines):
                    lanes[lane_index].get("control_points").append(intersection[0])
                    if intersection[-1] == "straight":
                        lanes.append({"control_points": [intersection[2], intersection[3]],
                                      "width": 8, "left_lanes": 1, "right_lanes": 1})
                        lanes.append(({"control_points": [intersection[0], intersection[1]],
                                       "width": 8, "left_lanes": 1, "right_lanes": 1}))
                        lane_index += 2
                        last_point = new_point
                    elif intersection[-1] == "right":
                        lanes.append({"control_points": [intersection[2], intersection[0]],
                                      "width": 8, "left_lanes": 1, "right_lanes": 1})
                        lanes.append({"control_points": [intersection[0], intersection[1]],
                                      "width": 8, "left_lanes": 1, "right_lanes": 1})
                        lane_index += 3
                        lanes.append({"control_points": [intersection[0], intersection[3]],
                                      "width": 8, "left_lanes": 1, "right_lanes": 1})
                    elif intersection[-1] == "left":
                        lanes.append({"control_points": [intersection[0], intersection[3]],
                                      "width": 8, "left_lanes": 1, "right_lanes": 1})
                        lanes.append({"control_points": [intersection[0], intersection[1]],
                                      "width": 8, "left_lanes": 1, "right_lanes": 1})
                        lane_index += 3
                        lanes.append({"control_points": [intersection[0], intersection[2]],
                                      "width": 8, "left_lanes": 1, "right_lanes": 1})
                    ego_lanes.append(lane_index)
                    lines_of_roads = convert_points_to_lines(lanes)
                    number_of_pieces += 1
                    one_intersection = True
                else:
                    intersection_possible = False
            # Add segment, if possible.
            control_points = lanes[lane_index].get("control_points")
            new_point = self._add_segment(control_points[-1], control_points[-2])
            new_line = LineString([(control_points[-1].get("x"), control_points[-1].get("y")),
                                   (new_point.get("x"), new_point.get("y"))])
            temp_list = deepcopy(lanes)
            temp_list[lane_index].get("control_points").append(new_point)
            temp_list = self._bspline(temp_list)
            control_points_lines = convert_splines_to_lines(temp_list)
            width_lines = self._get_width_lines(temp_list)
            if not intersection_check_last(lines_of_roads, new_line, max_intersections=0) \
                    and not intersection_check_width(width_lines, control_points_lines):
                lanes[lane_index].get("control_points").append(new_point)
                intersection_possible = True
                tries = 0
                number_of_pieces += 1
                lines_of_roads = convert_points_to_lines(lanes)
                last_point = new_point
            else:
                tries += 1
        if number_of_pieces >= self.MIN_NODES:
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

            # Duplicate width to have more space between road pieces.
            width = spline_list.get("width") * 3.5
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

    def _add_intersection(self, last_point, penultimate_point):
        # TODO Check for number of lanes. Stop signs only for single lane roads?
        # TODO Add traffic lights/stop signs
        # TODO Three-direction intersection.
        if random() <= 0.33:
            direction = "straight"
        elif 0.33 < random() <= 0.66:
            direction = "left"
        else:
            direction = "right"
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
        line_rot1 = affinity.rotate(line, -90, line.coords[0])
        line_rot1 = affinity.scale(line_rot1, xfact=3, yfact=3,
                                   origin=line_rot1.coords[0])
        line_rot2 = affinity.rotate(line, 90, line.coords[0])
        line_rot2 = affinity.scale(line_rot2, xfact=3, yfact=3,
                                   origin=line_rot2.coords[0])
        p1 = (list(shape(line_rot1).coords)[1][0], list(shape(line_rot1).coords)[1][1])
        p2 = (list(shape(line_rot2).coords)[1][0], list(shape(line_rot2).coords)[1][1])
        p1 = {"x": p1[0], "y": p1[1], "type": "intersection"}
        p2 = {"x": p2[0], "y": p2[1], "type": "intersection"}
        return [{"x": intersection_point[0], "y": intersection_point[1], "type": "intersection"},
                {"x": new_point[0], "y": new_point[1], "type": "intersection"},
                p1, p2, direction]

    def _turn_right(self):
        print("left turn")

    def _turn_left(self):
        print("right turn")

    def _go_straight(self):
        print("go straight")

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
            splined_list = self._bspline(individual.get("lanes"), samples=samples)
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
        plot_all(temp_list)

# TODO  Desired features:
#       TODO Refactoring
#       TODO Variable width
#       TODO Add obstacles
#       TODO Add other participants
#       TODO Calculate parallel coords for waypoints
#       TODO Three-lane intersection
#       TODO Angle dependency on num of lanes and width
#       TODO Adding traffic signs and lights(depending on num lanes)
#       TODO Highways
#       TODO Fix lane markings
#       TODO Control traffic lights
#       TODO Representation checking
#       TODO Create init population
#       TODO Mutation
#       TODO Mutation validity checks
#       TODO Crossover
#       TODO Repair function
#       TODO Improve performance

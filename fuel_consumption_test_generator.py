from copy import deepcopy
from math import degrees, atan2
from random import randint, random

import numpy as np
import scipy.interpolate as si
from shapely import affinity
from shapely.geometry import LineString, shape
from termcolor import colored

from utils.utility_functions import convert_points_to_lines
from utils.validity_checks import spline_intersection_check, intersection_check_all_np, intersection_check_width, \
    intersection_check_last
from scenario_types import ScenarioType

MIN_DEGREES = 70
MAX_DEGREES = 290


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
        self.files_name = "exampleTest"
        self.SPLINE_DEGREE = 5  # Sharpness of curves
        self.MAX_TRIES = 500  # Maximum number of invalid generated points/segments
        self.POPULATION_SIZE = 1  # Minimum number of generated roads for each generation
        self.NUMBER_ELITES = 4  # Number of best kept roads
        self.MIN_SEGMENT_LENGTH = 40  # Minimum length of a road segment
        self.MAX_SEGMENT_LENGTH = 65  # Maximum length of a road segment
        self.WIDTH_OF_STREET = 4  # Width of all segments
        self.MIN_NODES = 8  # Minimum number of control points for each road
        self.MAX_NODES = 12  # Maximum number of control points for each road
        self.population_list_urban = []
        self.population_list_highway = []
        self.intersection_length = 100
        self.intersecting_length = 75

    def _bspline(self, control_points, samples=75):
        """Calculate {@code samples} samples on a bspline. This is the road representation function.
        :param control_points: List of control points.
        :param samples: Number of samples to return.
        :return: Array with samples, representing a bspline of the given function as a numpy array.
        """
        point_list = []
        for point in control_points:
            point_list.append((point.get("x"), point.get("y")))
        point_list = np.asarray(point_list)
        count = len(point_list)
        degree = np.clip(self.SPLINE_DEGREE, 1, count - 1)

        # Calculate knot vector.
        kv = np.concatenate(([0] * degree, np.arange(count - degree + 1), [count - degree] * degree))

        # Calculate query range.
        u = np.linspace(False, (count - degree), samples)

        # Calculate result.
        return np.array(si.splev(u, (kv, point_list.T, degree))).T

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

    def _generate_random_points(self):
        """Generates random valid points and returns when the list is full or
        the number of invalid nodes equals the number of maximum tries.
        :return: Array of valid control points.
        """

        # Generating the first two points by myself.
        p0 = {"x": 1,
              "y": 0}
        p1 = {"x": 65,
              "y": 0}
        control_points = [p0, p1]
        tries = 0
        while len(control_points) != self.MAX_NODES and tries <= self.MAX_TRIES:
            new_point = self._generate_random_point(control_points[-1], control_points[-2])
            temp_list = deepcopy(control_points)
            temp_list.append(new_point)
            spline_list = self._bspline(temp_list, 100)
            control_points_lines = convert_points_to_lines(spline_list)
            width_list = self._get_width_lines(spline_list)
            if not (intersection_check_last(control_points, new_point) or spline_intersection_check(spline_list)
                    or intersection_check_width(width_list, control_points_lines)):
                control_points.append(new_point)
                tries = 0
            else:
                tries += 1

        spline_list = self._bspline(control_points, 100)
        if spline_intersection_check(spline_list):
            control_points.pop()
        if len(control_points) < self.MIN_NODES or intersection_check_all_np(spline_list):
            print(colored("Couldn't create enough valid nodes. Restarting...", "blue"))
        else:
            print(colored("Finished list!", "blue"))
            return control_points

    def _create_urban_environment(self):
        p0 = {"x": 1,
              "y": 0,
              "type": "segment"}
        p1 = {"x": 50,
              "y": 0,
              "type": "segment"}
        p2 = {"x": 65,
              "y": 0,
              "type": "segment"}
        control_points = [p0, p1, p2]
        lanes = []
        tries = 0
        one_intersection = False
        intersection_possible = True
        intersection_probability = 0.3
        # TODO Control points immer auf leer setzen und appenden, sobald left oder right turns passieren
        # TODO Lines vorher erstellen um Laufzeit zu verbessern
        # TODO Kurze Lines prüfen ob sie intersecten nach dem Einfügen
        while len(control_points) <= self.MAX_NODES and tries <= self.MAX_TRIES:
            if intersection_possible and len(control_points) == self.MAX_NODES - 1 and not one_intersection:
                intersection = self._add_intersection(control_points[-1], control_points[-2])
                new_point = intersection[0]
                if not intersection_check_last(lanes, control_points[-1], new_point, max_intersections=0):
                    one_intersection = True
                    control_points.append(new_point)
                    lanes.append({"control_points": [intersection[1], intersection[2]]})
            elif intersection_possible and random() <= intersection_probability \
                    and control_points[-1].get("type") != "intersection":
                intersection = self._add_intersection(control_points[-1], control_points[-2])
                new_point = intersection[0]
                if not intersection_check_last(lanes, control_points[-1], new_point, max_intersections=0):
                    one_intersection = True
                    control_points.append(new_point)
                    lanes.append({"control_points": [intersection[1], intersection[2]]})
                else:
                    tries += 1
                    intersection_possible = False
            else:
                while tries < self.MAX_TRIES:
                    new_point = self._add_segment(control_points[-1], control_points[-2])
                    if not intersection_check_last(lanes, control_points[-1], new_point, max_intersections=0):
                        control_points.append(new_point)
                        intersection_possible = True
                        tries = 0
                        break
                    else:
                        tries += 1
        main_lane = {"control_points": control_points}
        lanes.append(main_lane)
        return lanes

    def _repair(self):
        pass

    def _add_traffic(self):
        pass

    def _get_width_lines(self, control_points):
        """Determines the width lines of the road by flipping the LineString
         between two points by 90 degrees in both directions.
        :param control_points: List of control points.
        :return: List of LineStrings which represent the width of the road.
        """
        spline_list = deepcopy(control_points)
        linestring_list = []
        iterator = 0
        while iterator < (len(spline_list) - 1):
            p1 = (spline_list[iterator][0], spline_list[iterator][1])
            p2 = (spline_list[iterator + 1][0], spline_list[iterator + 1][1])
            line = LineString([p1, p2])

            # Rotate counter-clockwise and resize to the half of the road length.
            line_rot1 = affinity.rotate(line, 90, line.coords[0])
            line_rot1 = affinity.scale(line_rot1, xfact=self._get_resize_factor(line_rot1.length),
                                       yfact=self._get_resize_factor(line_rot1.length),
                                       origin=line_rot1.coords[0])

            # Rotate clockwise and resize to the half of the road length.
            line_rot2 = affinity.rotate(line, -90, line.coords[0])
            line_rot2 = affinity.scale(line_rot2, xfact=self._get_resize_factor(line_rot2.length),
                                       yfact=self._get_resize_factor(line_rot2.length),
                                       origin=line_rot2.coords[0])

            line = LineString([line_rot1.coords[1], line_rot2.coords[1]])
            linestring_list.append(line)

            if iterator == len(spline_list) - 2:
                line = LineString([p1, p2])
                line_rot1 = affinity.rotate(line, -90, line.coords[1])
                line_rot1 = affinity.scale(line_rot1, xfact=self._get_resize_factor(line_rot1.length),
                                           yfact=self._get_resize_factor(line_rot1.length),
                                           origin=line_rot1.coords[0])

                line_rot2 = affinity.rotate(line, 90, line.coords[1])
                line_rot2 = affinity.scale(line_rot2, xfact=self._get_resize_factor(line_rot2.length),
                                           yfact=self._get_resize_factor(line_rot2.length),
                                           origin=line_rot2.coords[0])
                line = LineString([line_rot1.coords[1], line_rot2.coords[1]])
                line = affinity.scale(line, xfact=self._get_resize_factor(line.length)*2,
                                      yfact=self._get_resize_factor(line.length)*2)
                linestring_list.append(line)
            iterator += 1
        return linestring_list

    def _get_resize_factor(self, length):
        """Returns the resize factor for the width lines so all lines have
        one specific length.
        :param length: Length of a LineString.
        :return: Resize factor.
        """
        if length == 0:
            return 0
        return self.WIDTH_OF_STREET / length

    def _get_resize_factor_intersection(self, linestring_length, intersection_length):
        if linestring_length == 0:
            return 0
        return (linestring_length + intersection_length) / linestring_length

    def _add_new_lane(self, first_point, second_point):
        pass

    def _add_intersection(self, last_point, penultimate_point):
        # TODO Check for number of lanes. Stop signs only for single lane roads?
        """
        if random() <= 0.33:
            self._go_straight()
        elif 0.33 < random() <= 0.66:
            self._turn_left()
        else:
            self._turn_right()
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
        return [{"x": new_point[0],
              "y": new_point[1],
              "type": "intersection"}, p1, p2]

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

    def _create_start_population(self):
        """Creates and returns an initial population."""
        startpop = []
        iterator = 0
        while len(startpop) < self.POPULATION_SIZE:
            point_list = self._create_urban_environment()
            if point_list is not None:
                individual = {"control_points": point_list,
                              "type": None,
                              "file_name": self.files_name,
                              "score": 0}
                startpop.append(individual)
                iterator += 1
        return startpop


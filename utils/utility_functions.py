"""This file offers some utility functions that can be used universally."""

from shapely.geometry import LineString
from math import degrees, atan2


def convert_points_to_lines(lanes):
    """Turns a list of points into a list of LineStrings.
    :param lanes: Lanes of an individual.
    :return: List of LineStrings.
    """
    lanes_lines = []
    for lane in lanes:
        control_points = lane.get("control_points")
        lines = []
        iterator = 0
        while iterator < (len(control_points) - 1):
            p1 = (control_points[iterator].get("x"), control_points[iterator].get("y"))
            p2 = (control_points[iterator + 1].get("x"), control_points[iterator + 1].get("y"))
            line = LineString([p1, p2])
            lines.append(line)
            iterator += 1
        lanes_lines.append(lines)
    return lanes_lines


def convert_splines_to_lines(lanes):
    """Turns a list of points into a list of LineStrings.
    :param lanes: Lanes of an individual.
    :return: List of LineStrings.
    """
    lanes_lines = []
    for lane in lanes:
        control_points = lane.get("control_points")
        lines = []
        iterator = 0
        while iterator < (len(control_points) - 1):
            p1 = (control_points[iterator][0], control_points[iterator][1])
            p2 = (control_points[iterator + 1][0], control_points[iterator + 1][1])
            line = LineString([p1, p2])
            lines.append(line)
            iterator += 1
        lanes_lines.append(lines)
    return lanes_lines


def get_angle(a, b, c):
    """Returns the angle between three points (two lines so to say).
    :param a: First point.
    :param b: Second point.
    :param c: Third point.
    :return: Angle in degrees.
    """
    ang = degrees(atan2(c[1] - b[1], c[0] - b[0]) - atan2(a[1] - b[1], a[0] - b[0]))
    return ang + 360 if ang < 0 else ang

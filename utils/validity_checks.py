"""This file offers various validity methods to check for intersections."""

from shapely.geometry import LineString, shape

from utils.plotter import plotter, plot_splines_and_width, plot_lines
from utils.utility_functions import convert_points_to_lines
import matplotlib.pyplot as plt


def intersection_check_last(lines_of_roads, new_line, max_intersections=0):
    """Checks for intersections between the line of the last two points and
     every other possible line.
    :param control_points: List of dicts containing points.
    :param point: Last inserted point, which should be checked for validity.
    :return: {@code True} if the last line intersects with another one, {@code False} if not.
    """
    last = list(shape(new_line).coords)[0]
    last = (last[0], last[1])
    intersections = 0
    for lane in lines_of_roads:
        for line in lane:
            if line.intersects(new_line) and not list(shape(line).coords)[1] == last:
                intersections += 1
                if intersections > max_intersections:
                    return True
    return False


def intersection_check_width(width_lines, control_points_lines):
    """Checks for intersections between the width lines of a control point and
     any other line between two control points.
    :param width_lines: Width lines of a control point (e.g. LineString). They should be flipped
                        by 90 degrees and in a list form.
    :param control_points_lines: List of lines between two control points (e.g. LineStrings).
    :return: {@code True} if two lines intersect, {@code False} if no line intersect.
    """
    for width_list in width_lines:
        for width_line in width_list:
            intersections = []
            for control_list in control_points_lines:
                for control_line in control_list:
                    if width_line.intersects(control_line):
                        intersec_point = width_line.intersection(control_line)
                        if intersec_point not in intersections and (len(intersections) == 0
                                            or intersec_point.distance(intersections[0]) > 0.01):
                            intersections.append(intersec_point)
                    # One line intersects always with its origin, therefore we need to check for another intersection.
                    if len(intersections) >= 2:
                        for it in intersections:
                            print(it)
                        return True
    return False


def spline_intersection_check(control_points):
    """Checks for intersection of a splined list. New point must be already
     added to the list.
    :param control_points: List of dicts containing points.
    :return: {@code True} if the last line intersects with any other, {@code False} if not.
    """
    iterator = 0
    while iterator <= (len(control_points) - 4):
        p1 = (control_points[iterator][0], control_points[iterator][1])
        p2 = (control_points[iterator + 1][0], control_points[iterator + 1][1])
        p3 = (control_points[-2][0], control_points[-2][1])
        p4 = (control_points[-1][0], control_points[-1][1])
        line1 = LineString([p1, p2])
        line2 = LineString([p3, p4])
        if line1.intersects(line2):
            return True
        iterator += 1
    return False


def intersection_check_all(control_points):
    """Checks for intersection between all lines of two connected control points.
    :param control_points: List of dicts containing points.
    :return: {@code True} if two lines intersects, {@code False} if not.
    """
    iterator = 0
    while iterator < (len(control_points) - 1):
        jterator = iterator + 2
        p1 = (control_points[iterator].get("x"), control_points[iterator].get("y"))
        p2 = (control_points[iterator + 1].get("x"), control_points[iterator + 1].get("y"))
        line1 = LineString([p1, p2])
        while jterator < (len(control_points) - 1):
            p3 = (control_points[jterator].get("x"), control_points[jterator].get("y"))
            p4 = (control_points[jterator + 1].get("x"), control_points[jterator + 1].get("y"))
            line2 = LineString([p3, p4])
            if line1.intersects(line2):
                return True
            jterator += 1
        iterator += 1
    return False


def intersection_check_all_np(control_points):
    """Checks for intersection between all lines of two connected control points.
    :param control_points: Numpy array containing points.
    :return: {@code True} if two lines intersects, {@code False} if not.
    """
    iterator = 0
    while iterator < (len(control_points) - 1):
        jterator = iterator + 2
        p1 = (control_points[iterator][0], control_points[iterator][1])
        p2 = (control_points[iterator + 1][0], control_points[iterator + 1][1])
        line1 = LineString([p1, p2])
        while jterator < (len(control_points) - 1):
            p3 = (control_points[jterator][0], control_points[jterator][1])
            p4 = (control_points[jterator + 1][0], control_points[jterator + 1][1])
            line2 = LineString([p3, p4])
            if line1.intersects(line2):
                return True
            jterator += 1
        iterator += 1
    return False

"""This file offers various validity methods to check for intersections."""

from shapely.geometry import LineString, shape
from utils.utility_functions import convert_points_to_lines


def intersection_check_last(lanes, last_point, new_point, max_intersections = 0):
    """Checks for intersections between the line of the last two points and
     every other possible line.
    :param control_points: List of dicts containing points.
    :param point: Last inserted point, which should be checked for validity.
    :return: {@code True} if the last line intersects with another one, {@code False} if not.
    """
    last = (last_point.get("x"), last_point.get("y"))
    new_line = LineString([last, (new_point.get("x"), new_point.get("y"))])
    lines_of_lanes = convert_points_to_lines(lanes)
    intersections = 0
    for lane in lines_of_lanes:
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
    iterator = 0
    while iterator < len(width_lines):
        intersections = 0
        jterator = 0
        while jterator < len(control_points_lines):
            if width_lines[iterator].intersects(control_points_lines[jterator]):
                intersections += 1
            # One line intersects always with its origin, therefore we need to check for another
            # intersection.
            if intersections >= 3:
                return True
            jterator += 1
        iterator += 1
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

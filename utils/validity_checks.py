"""This file offers various validity methods to check for intersections."""

from shapely.geometry import shape


def intersection_check_last(lines_of_roads, new_line, max_intersections=0):
    """Checks for intersections between the line of the last two points and
     every other possible line.
    :param lines_of_roads: List of lanes containing lines.
    :param new_line: Last inserted line, which should be checked for validity.
    :param max_intersections: Number of maximum intersections.
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


def intersection_check_width(width_lines, control_points_lines, intersection_lanes):
    """Checks for intersections between the width lines of a control point and
     any other line between two control points.
    :param width_lines: Width lines of a control point (e.g. LineString). They should be flipped
                        by 90 degrees and in a list form.
    :param control_points_lines: List of lines between two control points (e.g. LineStrings).
    :param intersection_lanes: List containing lists containing indices of lanes belonging to an intersection.
    :return: {@code True} if two lines intersect, {@code False} if no line intersect.
    """
    i = 0
    while i < len(width_lines):
        intersection_list = list()
        for piece in intersection_lanes:
            if i in piece:
                intersection_list = piece
                break
        for width_line in width_lines[i]:
            intersections = list()
            j = 0
            while j < len(control_points_lines):
                for control_line in control_points_lines[j]:
                    if j not in intersection_list and control_line.intersects(width_line):
                        intersec_point = control_line.intersection(width_line)
                        if intersec_point not in intersections and (len(intersections) == 0
                                                                    or intersec_point.distance(
                                    intersections[0]) > 0.01):
                            intersections.append(intersec_point)
                        # One line intersects always with its origin, therefore we need to check for another one.
                        if len(intersections) >= 2:
                            return True
                j += 1
        i += 1
    return False


def intersection_check_all(lines):
    """Checks for intersections between all lines.
    :param lines: List of LineStrings between two points.
    :return: {@code True} if two LineStrings intersects with each other, else {@code False}.
    """
    i0 = 0
    while i0 < len(lines):
        lane0 = lines[i0]
        j0 = 0
        while j0 < len(lane0):
            line0 = lane0[j0]
            i1 = i0
            while i1 < len(lines):
                lane1 = lines[i1]
                j1 = j0 + 1
                while j1 < len(lane1):
                    line1 = lane1[j1]
                    if line0.coords[1] != line1.coords[0] and line0.intersects(line1):
                        return True
                    j1 += 1
                i1 += 1
            j0 += 1
        i0 += 1
    return False

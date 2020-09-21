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
     by 90 degrees and in a list.
    :param control_points_lines: List of lines between two control points (e.g. LineStrings).
    :param intersection_lanes: List containing lists containing indices of lanes belonging to an intersection.
    :return: {@code True} if any poly- and width line intersect, {@code False} if not.
    """
    i = 0
    while i < len(width_lines):
        intersection_list = list()
        for piece in intersection_lanes:
            # Intersections of junctions should be ignored.
            if i in piece:
                intersection_list = piece
                break
        for width_line in width_lines[i]:
            intersections = list()      # This list stores intersection points to avoid duplicates.
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

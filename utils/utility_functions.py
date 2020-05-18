"""This file offers some utility functions that can be used universally."""
from shapely import affinity
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


def get_resize_factor(length, width):
    """Returns the resize factor for the width lines so all lines have
    one specific length.
    :param length: Length of a LineString.
    :param width: Width of the lane.
    :return: Resize factor.
    """
    if length == 0:
        return 0
    return width / length


def get_resize_factor_intersection(linestring_length, intersection_length):
    if linestring_length == 0:
        return 0
    return (linestring_length + intersection_length) / linestring_length


def get_width_lines(splined_lanes):
    """Determines the width lines of the road by flipping the LineString
     between two points by 90 degrees in both directions.
    :param splined_lanes: List of splined lanes.
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
            line_rot1 = affinity.scale(line_rot1, xfact=get_resize_factor(line_rot1.length, width),
                                       yfact=get_resize_factor(line_rot1.length, width),
                                       origin=line_rot1.coords[0])

            # Rotate clockwise and resize to the half of the road length.
            line_rot2 = affinity.rotate(line, -90, line.coords[0])
            line_rot2 = affinity.scale(line_rot2, xfact=get_resize_factor(line_rot2.length, width),
                                       yfact=get_resize_factor(line_rot2.length, width),
                                       origin=line_rot2.coords[0])

            line = LineString([line_rot1.coords[1], line_rot2.coords[1]])
            linestring_list.append(line)

            if iterator == len(control_points) - 2:
                line = LineString([p1, p2])
                line_rot1 = affinity.rotate(line, -90, line.coords[1])
                line_rot1 = affinity.scale(line_rot1, xfact=get_resize_factor(line_rot1.length, width),
                                           yfact=get_resize_factor(line_rot1.length, width),
                                           origin=line_rot1.coords[0])

                line_rot2 = affinity.rotate(line, 90, line.coords[1])
                line_rot2 = affinity.scale(line_rot2, xfact=get_resize_factor(line_rot2.length, width),
                                           yfact=get_resize_factor(line_rot2.length, width),
                                           origin=line_rot2.coords[0])
                line = LineString([line_rot1.coords[1], line_rot2.coords[1]])
                line = affinity.scale(line, xfact=get_resize_factor(line.length, width) * 2,
                                      yfact=get_resize_factor(line.length, width) * 2)
                linestring_list.append(line)
            iterator += 1
        complete_width_list.append(linestring_list)
    return complete_width_list


def get_angle(a, b, c):
    """Returns the angle between three points (two lines so to say).
    :param a: First point.
    :param b: Second point.
    :param c: Third point.
    :return: Angle in degrees.
    """
    ang = degrees(atan2(c[1] - b[1], c[0] - b[0]) - atan2(a[1] - b[1], a[0] - b[0]))
    return ang + 360 if ang < 0 else ang


def calc_width(left_lanes, right_lanes):
    """Calculates the width depending on the number of lanes. Width per lane can be 4 or 5.
    :param left_lanes: Number of left lanes.
    :param right_lanes: Number of right lanes.
    :return: Total width.
    """
    from random import randint
    multiplier = randint(4, 5)
    return (left_lanes + right_lanes) * multiplier


def calc_min_max_angles(num_lanes):
    """Calculates the minimum and maximum angle depending on the number of lanes.
    :param num_lanes: Number of lanes.
    :return: Minimum and maximum angles.
    """
    from fuel_consumption_test_generator import MIN_DEGREES, MAX_DEGREES
    return MIN_DEGREES + (num_lanes - 1) * 15, MAX_DEGREES - (num_lanes - 1) * 15


def get_lanes_of_intersection(intersection, last_point, width, left_lanes, right_lanes, lane_index):
    """Splits the intersection into lanes, assigns the right number of lanes and width to the lanes,
    determines the lanes that the ego car needs for its waypoints.
    :param intersection: Dict after calling _create_intersection.
    :param last_point: Latest added point of the individual.
    :param width: Width of the current lane.
    :param left_lanes: Number of left lanes of the current lane.
    :param right_lanes: Number of right lanes of the current lane.
    :param lane_index: Current lane index.
    :return: New lanes, lanes for the ego car, latest added point, number of left/right lanes of the lane which
    should receive new points, lane indices for this intersection as list type, current lane index.
    """
    lanes = []
    ego_lanes = []
    intersection_lanes = []
    new_left_lanes = intersection.get("new_left_lanes")
    new_right_lanes = intersection.get("new_right_lanes")
    new_width = intersection.get("new_width")
    left_point = intersection.get("left_point")
    right_point = intersection.get("right_point")
    straight_point = intersection.get("straight_point")
    intersec_point = intersection.get("intersection_point")
    number_of_ways = intersection.get("number_of_ways")
    layout = intersection.get("layout")

    lanes.append({"control_points": [last_point, intersec_point],
                  "width": width, "left_lanes": left_lanes, "right_lanes": right_lanes, "samples": 25})
    ego_lanes.append(lane_index + 1)

    if layout == "straight":
        lanes.append({"control_points": [intersec_point, straight_point],
                      "width": width, "left_lanes": left_lanes, "right_lanes": right_lanes, "samples": 25})
    elif layout == "left":
        lanes.append({"control_points": [left_point, intersec_point],
                      "width": new_width, "left_lanes": new_left_lanes, "right_lanes": new_right_lanes, "samples": 25})
    elif layout == "right":
        lanes.append({"control_points": [intersec_point, right_point],
                      "width": new_width, "left_lanes": new_left_lanes, "right_lanes": new_right_lanes, "samples": 25})
    if intersection.get("direction") == "straight":
        if number_of_ways == 4:
            lanes.extend([{"control_points": [left_point, intersec_point], "width": new_width,
                          "left_lanes": new_left_lanes, "right_lanes": new_right_lanes, "samples": 25},
                         {"control_points": [intersec_point, right_point], "width": new_width,
                          "left_lanes": new_left_lanes, "right_lanes": new_right_lanes, "samples": 25}])
            intersection_lanes.append([lane_index + 1, lane_index + 2, lane_index + 3, lane_index + 4])
            lane_index += 5
        else:
            intersection_lanes.append([lane_index + 1, lane_index + 2, lane_index + 3])
            lane_index += 4
        lanes.extend([{"control_points": [intersec_point, straight_point],
                       "width": width, "left_lanes": left_lanes, "right_lanes": right_lanes, "samples": 25},
                      {"control_points": [straight_point],
                       "width": width, "left_lanes": left_lanes, "right_lanes": right_lanes, "samples": 75}])
        last_point = straight_point
    else:
        if number_of_ways == 4:
            lanes.append({"control_points": [intersec_point, straight_point],
                          "width": width, "left_lanes": left_lanes, "right_lanes": right_lanes, "samples": 25})
            intersection_lanes.append([lane_index + 1, lane_index + 2, lane_index + 3, lane_index + 4])
            lane_index += 5
        else:
            intersection_lanes.append([lane_index + 1, lane_index + 2, lane_index + 3])
            lane_index += 4

        if intersection.get("direction") == "left":
            if number_of_ways == 4:
                lanes.append({"control_points": [intersec_point, right_point], "width": new_width,
                              "left_lanes": new_left_lanes, "right_lanes": new_right_lanes, "samples": 25})
            lanes.extend([{"control_points": [intersec_point, left_point], "width": new_width,
                           "left_lanes": new_left_lanes, "right_lanes": new_right_lanes, "samples": 25},
                          {"control_points": [left_point], "width": new_width, "left_lanes": new_left_lanes,
                           "right_lanes": new_right_lanes, "samples": 75}])
            last_point = left_point
        else:
            if number_of_ways == 4:
                lanes.append({"control_points": [intersec_point, left_point], "width": new_width,
                              "left_lanes": new_left_lanes, "right_lanes": new_right_lanes, "samples": 25})
            lanes.extend([{"control_points": [intersec_point, right_point], "width": new_width,
                           "left_lanes": new_left_lanes, "right_lanes": new_right_lanes, "samples": 25},
                          {"control_points": [right_point], "width": new_width, "left_lanes": new_left_lanes,
                           "right_lanes": new_right_lanes, "samples": 75}])
            last_point = right_point
        left_lanes = new_left_lanes
        right_lanes = new_right_lanes

    ego_lanes.extend([lane_index - 1, lane_index])
    return {"lanes": lanes, "ego_lanes": ego_lanes, "last_point": last_point, "left_lanes": left_lanes,
            "right_lanes": right_lanes, "intersection_lanes": intersection_lanes, "lane_index": lane_index}


def get_intersection_lines(last_point, intersection):
    """Creates and returns two LineStrings of a given intersection."""
    number_of_ways = intersection.get("number_of_ways")
    layout = intersection.get("layout")
    if number_of_ways == 4 or layout == "straight":
        new_line = LineString([(last_point.get("x"), last_point.get("y")),
                               (intersection.get("straight_point").get("x"),
                                intersection.get("straight_point").get("y"))])
    else:
        new_line = LineString([(last_point.get("x"), last_point.get("y")),
                               (intersection.get("intersection_point").get("x"),
                                intersection.get("intersection_point").get("y"))])
    if number_of_ways == 4:
        new_lane_line = LineString([(intersection.get("left_point").get("x"),
                                     intersection.get("left_point").get("y")),
                                    (intersection.get("right_point").get("x"),
                                     intersection.get("right_point").get("y"))])
    elif layout == "left":
        new_lane_line = LineString([(intersection.get("left_point").get("x"),
                                     intersection.get("left_point").get("y")),
                                    (intersection.get("intersection_point").get("x"),
                                     intersection.get("intersection_point").get("y"))])
    else:
        new_lane_line = LineString([(intersection.get("intersection_point").get("x"),
                                     intersection.get("intersection_point").get("y")),
                                    (intersection.get("right_point").get("x"),
                                     intersection.get("right_point").get("y"))])
    return new_line, new_lane_line

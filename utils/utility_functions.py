"""This file offers some utility functions that can be used universally."""


from shapely import affinity
from shapely.geometry import LineString, MultiLineString
from math import degrees, atan2
from random import randint

MIN_DEGREES = 90    # Minimum degree between three points.
MAX_DEGREES = 270   # Maximum degree between three points.


def multilinestrings_to_linestring(linestring):
    """Fixes Shapely's TopologyException sometimes. Converts MultiLineStrings to LineStrings.
    :param linestring: MultiLineString object.
    :return: Converted LineString object.
    """
    if isinstance(linestring, MultiLineString):
        temp_list = list()
        for line in linestring:
            for coord in list(line.coords):
                temp_list.append(coord)
        linestring = LineString(temp_list)
    return linestring


def convert_points_to_lines(lanes):
    """Turns a list of points into a list of LineStrings.
    :param lanes: Lanes of an individual.
    :return: List of LineStrings.
    """
    lanes_lines = list()
    for lane in lanes:
        control_points = lane.get("control_points")
        lines = list()
        i = 0
        while i < (len(control_points) - 1):
            p1 = (control_points[i][0], control_points[i][1])
            p2 = (control_points[i + 1][0], control_points[i + 1][1])
            line = LineString([p1, p2])
            lines.append(line)
            i += 1
        lanes_lines.append(lines)
    return lanes_lines


def get_resize_factor(length, width):
    """Returns the resize factor for the width lines so all lines have one specific length.
    :param length: Length of a LineString.
    :param width: Width of the lane.
    :return: Resize factor.
    """
    return width / length if length != 0 else 0


def get_resize_factor_intersection(linestring_length, intersection_length):
    """Returns the resize factor to resize lines for an intersection.
    :param linestring_length: Length of the current LineString as int.
    :param intersection_length: Desired length as int.
    :return: Resize factor as float.
    """
    return (linestring_length + intersection_length) / linestring_length if linestring_length != 0 else 0


def get_width_lines(splined_lanes):
    """Determines the width lines of the road by flipping the LineString
     between two points by 90 degrees in both directions.
    :param splined_lanes: List of splined lanes.
    :return: List of LineStrings which represent the width of the road.
    """
    complete_width_list = list()
    for spline_list in splined_lanes:
        linestring_list = list()
        i = 0

        # Triple width to have more space between road segments.
        width = spline_list.get("width") * 3
        control_points = spline_list.get("control_points")
        while i < (len(control_points) - 1):
            p1 = (control_points[i][0], control_points[i][1])
            p2 = (control_points[i + 1][0], control_points[i + 1][1])
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

            # Append last line.
            if i == len(control_points) - 2:
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
            i += 1
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
    return (left_lanes + right_lanes) * randint(4, 5)


def calc_min_max_angles(num_lanes):
    """Calculates the minimum and maximum angle depending on the number of lanes.
    :param num_lanes: Number of lanes.
    :return: Minimum and maximum angles.
    """
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
    lanes = list()
    ego_lanes = list()
    intersection_lanes = list()
    new_left_lanes = intersection.get("new_left_lanes")
    new_right_lanes = intersection.get("new_right_lanes")
    new_width = intersection.get("new_width")
    left_point = intersection.get("left_point")
    right_point = intersection.get("right_point")
    straight_point = intersection.get("straight_point")
    intersec_point = intersection.get("intersection_point")
    number_of_ways = intersection.get("number_of_ways")
    layout = intersection.get("layout")

    lanes.append({"control_points": [last_point, intersec_point], "width": width, "left_lanes": left_lanes,
                  "right_lanes": right_lanes, "samples": 25, "type": "intersection"})
    ego_lanes.append(lane_index + 1)

    if layout == "straight":
        lanes.append({"control_points": [intersec_point, straight_point], "width": width, "left_lanes": left_lanes,
                      "right_lanes": right_lanes, "samples": 25, "type": "intersection"})
    elif layout == "left":
        lanes.append({"control_points": [intersec_point, left_point], "width": new_width, "left_lanes": new_right_lanes,
                      "right_lanes": new_left_lanes, "samples": 25, "type": "intersection"})
    elif layout == "right":
        lanes.append({"control_points": [intersec_point, right_point], "width": new_width,
                      "left_lanes": new_right_lanes, "right_lanes": new_left_lanes, "samples": 25,
                      "type": "intersection"})
    if intersection.get("direction") == "straight":
        if number_of_ways == 4:
            lanes.extend([{"control_points": [intersec_point, left_point], "width": new_width,
                          "left_lanes": new_left_lanes, "right_lanes": new_right_lanes, "samples": 25,
                           "type": "intersection"},
                         {"control_points": [intersec_point, right_point], "width": new_width,
                          "left_lanes": new_right_lanes, "right_lanes": new_left_lanes, "samples": 25,
                          "type": "intersection"}])
            intersection_lanes.append([lane_index + 1, lane_index + 2, lane_index + 3, lane_index + 4])
            lane_index += 5
        else:
            intersection_lanes.append([lane_index + 1, lane_index + 2, lane_index + 3])
            lane_index += 4
        lanes.extend([{"control_points": [intersec_point, straight_point], "width": width, "left_lanes": left_lanes,
                       "right_lanes": right_lanes, "samples": 25, "type": "intersection"},
                      {"control_points": [straight_point], "width": width, "left_lanes": left_lanes,
                       "right_lanes": right_lanes, "samples": 100, "type": "normal"}])
        last_point = straight_point
    else:
        if number_of_ways == 4:
            lanes.append({"control_points": [intersec_point, straight_point], "width": width, "left_lanes": left_lanes,
                          "right_lanes": right_lanes, "samples": 25, "type": "intersection"})
            intersection_lanes.append([lane_index + 1, lane_index + 2, lane_index + 3, lane_index + 4])
            lane_index += 5
        else:
            intersection_lanes.append([lane_index + 1, lane_index + 2, lane_index + 3])
            lane_index += 4

        if intersection.get("direction") == "left":
            if number_of_ways == 4:
                lanes.append({"control_points": [intersec_point, right_point], "width": new_width,
                              "left_lanes": new_right_lanes, "right_lanes": new_left_lanes, "samples": 25,
                              "type": "intersection"})
            lanes.extend([{"control_points": [intersec_point, left_point], "width": new_width,
                           "left_lanes": new_left_lanes, "right_lanes": new_right_lanes, "samples": 25,
                           "type": "intersection"},
                          {"control_points": [left_point], "width": new_width, "left_lanes": new_left_lanes,
                           "right_lanes": new_right_lanes, "samples": 100, "type": "normal"}])
            last_point = left_point
        else:
            if number_of_ways == 4:
                lanes.append({"control_points": [intersec_point, left_point], "width": new_width,
                              "left_lanes": new_right_lanes, "right_lanes": new_left_lanes, "samples": 25,
                              "type": "intersection"})
            lanes.extend([{"control_points": [intersec_point, right_point], "width": new_width,
                           "left_lanes": new_left_lanes, "right_lanes": new_right_lanes, "samples": 25,
                           "type": "intersection"},
                          {"control_points": [right_point], "width": new_width, "left_lanes": new_left_lanes,
                           "right_lanes": new_right_lanes, "samples": 100, "type": "normal"}])
            last_point = right_point
        left_lanes = new_left_lanes
        right_lanes = new_right_lanes

    ego_lanes.extend([lane_index - 1, lane_index])
    return {"lanes": lanes, "ego_lanes": ego_lanes, "last_point": last_point, "left_lanes": left_lanes,
            "right_lanes": right_lanes, "intersection_lanes": intersection_lanes, "lane_index": lane_index}


def get_intersection_lines(last_point, intersection):
    """Creates and returns two LineStrings of a given intersection.
    :param last_point: Last point which was added as a tuple (x, y).
    :param intersection: Intersection which was created using the function _create_intersection().
    :return: Two LineStrings representing the new intersection.
    """
    number_of_ways = intersection.get("number_of_ways")
    layout = intersection.get("layout")
    if number_of_ways == 4 or layout == "straight":
        new_line = LineString([(last_point[0], last_point[1]),
                               (intersection.get("straight_point")[0],
                                intersection.get("straight_point")[1])])
    else:
        new_line = LineString([(last_point[0], last_point[1]),
                               (intersection.get("intersection_point")[0],
                                intersection.get("intersection_point")[1])])
    if number_of_ways == 4:
        new_lane_line = LineString([(intersection.get("left_point")[0],
                                     intersection.get("left_point")[1]),
                                    (intersection.get("right_point")[0],
                                     intersection.get("right_point")[1])])
    elif layout == "left":
        new_lane_line = LineString([(intersection.get("left_point")[0],
                                     intersection.get("left_point")[1]),
                                    (intersection.get("intersection_point")[0],
                                     intersection.get("intersection_point")[1])])
    else:
        new_lane_line = LineString([(intersection.get("intersection_point")[0],
                                     intersection.get("intersection_point")[1]),
                                    (intersection.get("right_point")[0],
                                     intersection.get("right_point")[1])])
    return new_line, new_lane_line


def calc_speed_waypoints(participants):
    """
    Calculates speed for each waypoint.
    :param participants: List of participants.
    :return: Void.
    """
    for participant in participants:
        waypoints = participant["waypoints"]
        if len(waypoints) == 0:
            continue
        i = 0
        current_index = int(waypoints[0].get("lane"))
        while i < len(waypoints):
            if 1 < i < len(waypoints) - 1:
                if int(waypoints[i].get("lane")) != current_index:
                    current_index = int(waypoints[i].get("lane"))
                    waypoints[i-1]["speed"] = 0
                    waypoints[i-2]["speed"] = 0
                    waypoints[i-3]["speed"] = 0
                    if i - 5 >= 0:
                        waypoints[i - 4]["speed"] = 0
                        waypoints[i - 5]["speed"] = 0
                    if i - 8 >= 0:
                        waypoints[i - 6]["speed"] = 0
                        waypoints[i - 7]["speed"] = 0
                        waypoints[i - 8]["speed"] = 0
                p1 = waypoints[i - 1].get("position")
                p3 = waypoints[i + 1].get("position")
                angle = get_angle(p1, waypoints[i].get("position"), p3)
                if 170 <= angle <= 190:
                    speed = 13.8
                elif 150 <= angle < 170 or 190 > angle >= 210:
                    speed = 10
                elif 130 <= angle < 150 or 210 > angle >= 230:
                    speed = 8.5
                elif 110 <= angle < 130 or 230 > angle >= 250:
                    speed = 6
                elif 90 <= angle < 110 or 250 > angle >= 270:
                    speed = 3
                elif angle < 90 or angle > 270:
                    speed = 2
                else:
                    speed = 1
                waypoints[i-1]["speed"] = 0 if waypoints[i-1].get("speed") == 0 else \
                        (speed + float(waypoints[i-1].get("speed"))) / 2
                waypoints[i]["speed"] = speed
            else:
                speed = 0 if i == len(waypoints) - 1 else 13.8
                waypoints[i]["speed"] = speed
            i += 1

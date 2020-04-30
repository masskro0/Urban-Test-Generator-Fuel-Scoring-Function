from shapely.geometry import LineString


def convert_points_to_lines(lanes):
    """Turns a list of points into a list of LineStrings.
    :param control_points: List of dicts containing points.
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

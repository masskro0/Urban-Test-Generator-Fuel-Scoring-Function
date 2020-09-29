"""This class builds an environment XML file."""

import xml.etree.ElementTree as ElementTree
from os import path, remove, mkdir
from os.path import abspath, dirname, join
from pathlib import Path


def indent(elem, level=0):
    """Pretty prints a xml file.
    :param elem: XML tag.
    :param level: Number of empty spaces, initially zero (meaning it starts only a new line).
    :return: Void.
    """
    i = "\n" + level * "    "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "    "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def save_xml(name, root, xml_type, destination_path=None):
    """Creates and saves the XML file, and moves it to the scenario folder or a given path.
    :param destination_path: Path where the XML file should be written to.
    :param xml_type: Environment or criteria.
    :param root: Root of the XML file.
    :param name: Desired name of this file.
    :return: Void.
    """
    # Wrap it in an ElementTree instance, and save as XML.
    tree = ElementTree.ElementTree(root)
    indent(root)
    full_name = name + '.dbe.xml' if xml_type == "environment" else name + '.dbc.xml'

    if destination_path is None:
        destination_path = join(Path(dirname(abspath(__file__))).parent, "scenario")

    if not path.exists(destination_path):
        mkdir(destination_path)

    if path.exists(join(destination_path, full_name)):
        remove(join(destination_path, full_name))

    tree.write(join(destination_path, full_name), encoding="utf-8", xml_declaration=True)


class DBEBuilder:

    def __init__(self):
        # Build a tree structure.
        self.root = ElementTree.Element("environment")

        self.author = ElementTree.SubElement(self.root, "author")
        self.author.text = "Michael Heine"

        self.timeOfDay = ElementTree.SubElement(self.root, "timeOfDay")

        self.roads = ElementTree.SubElement(self.root, "roads")

    def add_obstacles(self, obstacle_list):
        """Adds obstacles to the XML files.
        :param obstacle_list: List of obstacles. Each obstacle is a dict and must contain a position, name as well as
         obstacle specific attributes.
        :return: Void.
        """
        obstacles = ElementTree.SubElement(self.root, "obstacles")
        for obstacle in obstacle_list:
            name = obstacle.get("name")
            pos = obstacle.get("position")
            assert len(pos) >= 2, "Obstacles must contain a x and y coordinate."
            x = pos[0]
            y = pos[1]
            z = 0 if len(pos) == 2 else pos[2]
            x_rot = obstacle.get("xRot")
            y_rot = obstacle.get("yRot")
            z_rot = obstacle.get("zRot")
            width = obstacle.get("width")
            length = obstacle.get("length")
            height = obstacle.get("height")
            radius = obstacle.get("radius")
            base_radius = obstacle.get("baseRadius")
            upper_width = obstacle.get("upperWidth")
            upper_length = obstacle.get("upperLength")
            color = obstacle.get("color")
            mode = obstacle.get("mode")
            sign = obstacle.get("sign")
            oid = obstacle.get("oid")
            facing_ego = obstacle.get("facingEgo")
            full_string = '' + name + ' x="' + str(x) + '" y="' + str(y) + '"'
            if z:
                full_string += ' z="' + str(z) + '"'
            if x_rot:
                full_string += ' xRot="' + str(x_rot) + '"'
            if y_rot:
                full_string += ' yRot="' + str(y_rot) + '"'
            if z_rot:
                full_string += ' zRot="' + str(z_rot) + '"'
            if width:
                full_string += ' width="' + str(width) + '"'
            if length:
                full_string += ' length="' + str(length) + '"'
            if height:
                full_string += ' height="' + str(height) + '"'
            if radius:
                full_string += ' radius="' + str(radius) + '"'
            if base_radius:
                full_string += ' baseRadius="' + str(base_radius) + '"'
            if upper_width:
                full_string += ' upperWidth="' + str(upper_width) + '"'
            if upper_length:
                full_string += ' upperLength="' + str(upper_length) + '"'
            if color:
                full_string += ' color="' + str(color) + '"'
            if mode:
                full_string += ' mode="' + str(mode) + '"'
            if sign:
                full_string += ' sign="' + sign + '"'
            if oid:
                full_string += ' oid="' + oid + '"'
            if facing_ego:
                full_string += ' facingEgo="' + str(facing_ego) + '"'
            ElementTree.SubElement(obstacles, full_string)

    def set_tod(self, tod):
        """Sets the time of day.
        :param tod: Time of day as int or string type.
        :return: Void.
        """
        self.timeOfDay.text = str(tod)

    def add_roads(self, roads):
        """Adds new roads to the environment file.
        :param roads: List of roads as dict type. Must contain:
                 control_points: List of points with x and y coordinates,
                 width: Width for whole road as int,
                 left_lanes: Number of left lanes as int,
                 right_lanes: Number of right lanes as int.
        :return: Void.
        """
        for road in roads:
            self.add_road(control_points=road.get("control_points"),
                          width=road.get("width"),
                          left_lanes=road.get("left_lanes"),
                          right_lanes=road.get("right_lanes")
                          )

    def add_road(self, control_points, width, markings=True, left_lanes=0, right_lanes=0):
        """Adds a road and road segments.
        :param control_points: List of tuples containing x-coordinate and y-coordinate.
        :param width: Width of the whole road as a Integer.
        :param markings: {@code True} Enables road markings, {@code False} makes them invisible.
        :param left_lanes: Number of left lanes.
        :param right_lanes: Number of right lanes.
        :return: Void.
        """
        road = ElementTree.SubElement(self.roads, "road")
        if markings:
            road.set("markings", "true")
        if left_lanes != 0 and left_lanes is not None:
            road.set("leftLanes", str(left_lanes))
        if right_lanes != 0 and right_lanes is not None:
            road.set("rightLanes", str(right_lanes))
        for segment in control_points:
            ElementTree.SubElement(road, 'roadSegment x="{}" y="{}" width="{}"'
                                   .format('{0:.10f}'.format(round(segment[0], 2)),
                                           '{0:.10f}'.format(round(segment[1], 2)), str(width)))

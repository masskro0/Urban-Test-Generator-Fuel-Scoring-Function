"""This class builds an environment XML file."""

import xml.etree.ElementTree as ElementTree
from os import path, remove, getcwd, mkdir
from pathlib import Path
from shutil import move


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


def save_xml(name, root, xml_type):
    """Creates and saves the XML file, and moves it to the scenario folder.
    :param xml_type: Environment or criteria.
    :param root: Root of the XML file.
    :param name: Desired name of this file.
    :return: Void.
    """
    # Wrap it in an ElementTree instance, and save as XML.
    tree = ElementTree.ElementTree(root)
    indent(root)
    full_name = name + '.dbe.xml' if xml_type == "environment" else name + '.dbc.xml'

    current_path_of_file = Path(getcwd())
    current_path_of_file = path.realpath(current_path_of_file) + "\\" + full_name

    destination_path = Path(getcwd())
    destination_path = path.realpath(destination_path) + "\\scenario"

    tree.write(full_name, encoding="utf-8", xml_declaration=True)

    if not path.exists(destination_path):
        mkdir(destination_path)

    # Delete old files with the same name.
    if path.exists(destination_path + "\\" + full_name):
        remove(destination_path + "\\" + full_name)

    # Move created file to scenario folder.
    move(current_path_of_file, destination_path)


class DBEBuilder:

    def __init__(self):
        # Build a tree structure.
        self.root = ElementTree.Element("environment")

        self.author = ElementTree.SubElement(self.root, "author")
        self.author.text = "Michael Heine"

        self.timeOfDay = ElementTree.SubElement(self.root, "timeOfDay")

        self.lanes = ElementTree.SubElement(self.root, "lanes")

    def add_obstacles(self, obstacle_list):
        """Adds obstacles to the XML files.
        :param obstacle_list: List of obstacles. Each obstacle is a dict and must contain x and y position as well as
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
            ElementTree.SubElement(obstacles, full_string)

    def set_tod(self, tod):
        """Sets the time of day.
        :param tod: Time of day as int or string type.
        :return: Void.
        """
        self.timeOfDay.text = str(tod)

    def add_lanes(self, lanes):
        """Adds new lanes to the environment file.
        :param lanes: List of lanes as dict type. Must contain:
                 control_points: List of points with x and y coordinates,
                 width: Width for whole lane as int,
                 left_lanes: Number of left lanes as int,
                 right_lanes: Number of right lanes as int.
        :return: Void.
        """
        for lane in lanes:
            self.add_lane(control_points=lane.get("control_points"),
                          width=lane.get("width"),
                          left_lanes=lane.get("left_lanes"),
                          right_lanes=lane.get("right_lanes")
                          )

    def add_lane(self, control_points, width, markings=True, left_lanes=0, right_lanes=0):
        """Adds a lane and road segments.
        :param control_points: List of tuples containing x-coordinate and y-coordinate.
        :param width: Width of the whole lane as a Integer.
        :param markings: {@code True} Enables road markings, {@code False} makes them invisible.
        :param left_lanes: Number of left lanes.
        :param right_lanes: Number of right lanes.
        :return: Void.
        """
        lane = ElementTree.SubElement(self.lanes, "lane")
        if markings:
            lane.set("markings", "true")
        if left_lanes != 0 and left_lanes is not None:
            lane.set("leftLanes", str(left_lanes))
        if right_lanes != 0 and right_lanes is not None:
            lane.set("rightLanes", str(right_lanes))
        for segment in control_points:
            ElementTree.SubElement(lane, 'laneSegment x="{}" y="{}" width="{}"'
                                   .format('{0:.10f}'.format(round(segment[0], 2)),
                                           '{0:.10f}'.format(round(segment[1], 2)), str(width)))

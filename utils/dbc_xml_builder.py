"""This class builds a criteria XML file for DriveBuild in the required format. Note that not
    every criteria or items can be added to the XML files, I chose only the ones that I need.
"""

import xml.etree.ElementTree as ElementTree
from os import path, remove, getcwd, mkdir
from pathlib import Path
from shutil import move


class DBCBuilder:

    def __init__(self):
        # Build a tree structure.
        self.root = ElementTree.Element("criteria")

        self.environment = ElementTree.SubElement(self.root, "environment")

        self.author = ElementTree.SubElement(self.root, "author")
        self.author.text = "Michael Heine"

        self.version = ElementTree.SubElement(self.root, "version")
        self.version.text = "1"

        self.participants = ElementTree.SubElement(self.root, "participants")

        self.preconditions = ElementTree.SubElement(self.root, "precondition")

        self.success = ElementTree.SubElement(self.root, "success")

        self.failure = ElementTree.SubElement(self.root, "failure")

    def define_name(self, file_name):
        """Defines the name of the test (not the file name). Required tag.
        :param file_name: Name of this test.
        return: Void
        """
        name = ElementTree.SubElement(self.root, "name")
        name.text = file_name

    def environment_name(self, dbe_file_name):
        """Add the corresponding environment XML file to this criteria XML file. Required tag.
        :param dbe_file_name: File name of the environment file as a String.
        :return: Void.
        """
        if not dbe_file_name.endswith(".dbe.xml"):
            dbe_file_name += ".dbe.xml"
        self.environment.text = dbe_file_name

    def steps_per_second(self, fps="60"):
        """Sets the steps per second. Required tag.
        :param fps: FPS as an integer.
        :return: Void
        """
        steps = ElementTree.SubElement(self.root, "stepsPerSecond")
        steps.text = str(fps)

    def ai_freq(self, frequency="6"):
        """Sets the AI frequency. Required tag.
        :param frequency: Frequency as an integer.
        :return: Void
        """
        aifreq = ElementTree.SubElement(self.root, "aiFrequency")
        aifreq.text = str(frequency)

    def add_car(self, participant):
        """Adds a car to this test case. At least one car (the ego car) should be added.
        :param participant: Dict which contains init_state, waypoints, participant_id and model. See the lines below
                            for more information:
                init_state: Dict with initial states. Contains: x-coordinate (int), y-coordinate (int),
                     orientation (int), movementMode (MANUAL, _BEAMNG, AUTONOMOUS, TRAINING),
                     speed (int)
                waypoints: Array with waypoints. One waypoint contains: x-coordinate (int),
                     y-coordinate (int), tolerance (int), movementMode (see above),
                     speedLimit (int) (optional)
                participant_id: unique ID of this participant as String.
                model: BeamNG model car as String. See beamngpy documentation for more models.
        :return: Void
        """
        participant_id = participant.get("id")
        init_state = participant.get("init_state")
        waypoints = participant.get("waypoints")
        model = participant.get("model")
        participant = ElementTree.SubElement(self.participants, "participant")
        participant.set("id", participant_id)
        participant.set("model", model)
        ElementTree.SubElement(participant, 'initialState x="{}" y="{}"'
                                            ' orientation="{}" movementMode="{}"'
                                            ' speed="{}"'
                               .format(str(init_state.get("position")[0]), str(init_state.get("position")[1]),
                                       str(init_state.get("orientation")), init_state.get("movementMode"),
                                       str(init_state.get("speed"))))

        ai = ElementTree.SubElement(participant, "ai")
        ElementTree.SubElement(ai, 'roadCenterDistance id="{}"'.format("egoLaneDist"))

        movement = ElementTree.SubElement(participant, "movement")
        for waypoint in waypoints:
            position = waypoint.get("position")
            waypoint_tag = ElementTree.SubElement(movement, 'waypoint x="{}" y="{}" tolerance="{}"'
                                                            ' movementMode="{}"'
                                                  .format('{0:.10f}'.format(round(position[0], 2)),
                                                          '{0:.10f}'.format(round(position[1], 2)),
                                                          str(waypoint.get("tolerance")),
                                                          waypoint.get("movementMode")))
            if waypoint.get("speedLimit"):
                waypoint_tag.set("speedLimit", str(waypoint.get("speedLimit")))

    def add_precond_partic_sc_speed(self, vc_pos, sc_speed):
        """Adds a precondition for a position, which must be satisfied in order to continue the test.
        This method requires a lower speed bound, which must be reached.
        :param vc_pos: Position of the precondition. Dict contains: participant id (string),
                xPos (int), yPos (int), tolerance (int) defines a circle which must be entered.
        :param sc_speed: Lower speed bound as integer.
        :return: Void
        """
        vc_position = ElementTree.SubElement(self.preconditions, 'vcPosition')
        vc_position.set("participant", vc_pos.get("id"))
        vc_position.set("x", str(vc_pos.get("position")[0]))
        vc_position.set("y", str(vc_pos.get("position")[1]))
        vc_position.set("tolerance", str(vc_pos.get("tolerance")))

        not_tag = ElementTree.SubElement(vc_position, "not")
        ElementTree.SubElement(not_tag, 'scSpeed participant="{}" limit="{}"'
                               .format(vc_pos.get("id"), str(sc_speed)))

    def add_success_point(self, participant_id, success_point):
        """Point when reached a test was successfully finished.
        :param participant_id: ID of the participant as a string.
        :param success_point: Dict of a success state. Contains: x (int), y (int),
               tolerance (int) which defines a circle.
        :return: Void.
        """
        tolerance = 3
        ElementTree.SubElement(self.success, 'scPosition participant="{}" x="{}" y="{}" tolerance="{}"'
                               .format(participant_id, str(success_point[0]), str(success_point[1]),
                                       str(tolerance)))

    def add_failure_damage(self, participant_id):
        """Adds damage observation as a test failure condition.
        :param participant_id: participant id (string)
        :return: Void
        """
        ElementTree.SubElement(self.failure, 'scDamage participant="{}"'.format(participant_id))

    def add_failure_lane(self, participant_id, lane="offroad"):
        """Adds lane following observation as a test failure condition.
        :param participant_id: participant id (string)
        :param lane: on which lane should the test fail? (markings, leftLanes, rightLanes, offroad)
        :return: Void
        """
        ElementTree.SubElement(self.failure, 'scLane participant="{}" onLane="{}"'
                               .format(participant_id, lane))

    def add_failure_conditions(self, participant_id, lane="offroad"):
        """Adds both lane following and damage observation as a test failure condition.
        :param participant_id: participant id (string)
        :param lane: on which lane should the test fail? (markings, leftLanes, rightLanes, offroad)
        :return: Void
        """
        or_tag = ElementTree.SubElement(self.failure, "or")
        ElementTree.SubElement(or_tag, 'scDamage participant="{}"'.format(participant_id))
        ElementTree.SubElement(or_tag, 'scLane participant="{}" onLane="{}"'.format(participant_id, lane))

    def indent(self, elem, level=0):
        """Pretty prints the xml file.
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
                self.indent(elem, level + 1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    def save_xml(self, name):
        """Creates and saves the XML file, and moves it to the scenario folder.
        :param name: Desired name of this file.
        :return: Void, but it creates a XML file.
        """
        # Wrap it in an ElementTree instance, and save as XML.
        tree = ElementTree.ElementTree(self.root)
        self.indent(self.root)
        full_name = name + '.dbc.xml'

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

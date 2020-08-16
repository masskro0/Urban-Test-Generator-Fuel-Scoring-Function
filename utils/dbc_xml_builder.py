"""This class builds a criteria XML file."""

import xml.etree.ElementTree as ElementTree


class DBCBuilder:

    def __init__(self):
        # Build a tree structure.
        self.root = ElementTree.Element("criteria")

        self.environment = ElementTree.SubElement(self.root, "environment")

        self.author = ElementTree.SubElement(self.root, "author")
        self.author.text = "Michael Heine"

        self.participants = ElementTree.SubElement(self.root, "participants")

        self.success = ElementTree.SubElement(self.root, "success")

        self.failure = ElementTree.SubElement(self.root, "failure")

    def define_name(self, file_name):
        """Defines the name of the test (not the file name).
        :param file_name: Name of this test.
        return: Void
        """
        name = ElementTree.SubElement(self.root, "name")
        name.text = file_name

    def environment_name(self, dbe_file_name):
        """Adds the corresponding environment XML file to this criteria XML file.
        :param dbe_file_name: File name of the environment file as a String.
        :return: Void.
        """
        if not dbe_file_name.endswith(".dbe.xml"):
            dbe_file_name += ".dbe.xml"
        self.environment.text = dbe_file_name

    def add_trigger_points(self, triggers):
        """Adds trigger points to the criteria XML file.
        :param triggers: List of dict types. Must contain:
                 position: Tuple with x and y coordinate,
                 tolerance: As int type,
                 action: String type telling which task should be performed when entered,
                 triggeredBy: As string type defines which vid triggers this point,
                 triggers: as string type defines who should be triggered (can be object id or vehicle id).
               Supported actions:
                 spawnAndStart: Spawns another vehicle and gives him a polyline that he should drive along. Needs
                  following elements:
                    spawnPoint: Element containing,
                      position: x and y coordinate,
                      orientation: Float type.
                  switchLights: Switches the lights of a traffic light when entered. This action needs the attributes:
                    initState: String type to tell the initial color,
                    switchTo: String type to define to which color should be switched to.
        :return: Void.
        """
        trigger_root = ElementTree.SubElement(self.root, "triggerPoints")
        for trigger_dict in triggers:
            trigger_point = trigger_dict.get("triggerPoint")
            trigger = ElementTree.SubElement(trigger_root, 'triggerPoint')
            trigger.set("x", str(trigger_point.get("position")[0]))
            trigger.set("y", str(trigger_point.get("position")[1]))
            trigger.set("tolerance", str(trigger_point.get("tolerance")))
            trigger.set("action", trigger_point.get("action"))
            trigger.set("triggeredBy", trigger_point.get("triggeredBy"))
            if trigger_point.get("triggers") is not None:
                trigger.set("triggers", trigger_point.get("triggers"))
            if trigger_point.get("action") == "spawnAndStart":
                spawn_point = trigger_dict.get("spawnPoint")
                ElementTree.SubElement(trigger, 'spawnPoint x="{}" y="{}" orientation="{}"'
                                       .format(str(spawn_point.get("position")[0]),
                                               str(spawn_point.get("position")[1]),
                                               str(spawn_point.get("orientation"))))
            elif trigger_point.get("action") == "switchLights":
                trigger.set("initState", trigger_point.get("initState"))
                trigger.set("switchTo", trigger_point.get("switchTo"))
            elif trigger_point.get("action") == "stop":
                trigger.set("duration", str(trigger_point.get("duration")))

    def add_car(self, participant):
        """Adds a car to this test case. At least one car (the ego car) should be added.
        :param participant: Dict which contains init_state, waypoints, participant_id and model. See the lines below
                            for more information:
                init_state: Dict with initial states. Contains:
                  x-coordinate: Int,
                  y-coordinate Int,
                  orientation Int.
                waypoints: Array with waypoints. One waypoint contains:
                  x-coordinate: Int,
                  y-coordinate: Int,
                  tolerance: Int.
                participant_id: unique ID of this participant as String.
                model: BeamNG model car as String.
                color: Color of the car as string type.
        :return: Void
        """
        participant_id = participant.get("id")
        init_state = participant.get("init_state")
        waypoints = participant.get("waypoints")
        model = participant.get("model")
        color = participant.get("color")
        participant = ElementTree.SubElement(self.participants, "participant")
        participant.set("id", participant_id)
        participant.set("model", model)
        participant.set("color", color)
        ElementTree.SubElement(participant, 'initialState x="{}" y="{}"'
                                            ' orientation="{}"'
                               .format(str(init_state.get("position")[0]), str(init_state.get("position")[1]),
                                       str(init_state.get("orientation"))))

        if waypoints is not None:
            movement = ElementTree.SubElement(participant, "movement")
            for waypoint in waypoints:
                position = waypoint.get("position")
                ElementTree.SubElement(movement, 'waypoint x="{}" y="{}" tolerance="{}" lane="{}" speed="{}"'
                                                  .format('{0:.10f}'.format(round(position[0], 2)),
                                                          '{0:.10f}'.format(round(position[1], 2)),
                                                          str(waypoint.get("tolerance")), waypoint.get("lane"),
                                                          str(waypoint.get("speed"))))

    def add_success_point(self, participant_id, success_point):
        """Point when reached a test was successfully finished.
        :param participant_id: ID of the participant as a string.
        :param success_point: Dict of a success state. Contains:
                 x: Int,
                 y: Int,
                 tolerance: Int type which defines a circle.
        :return: Void.
        """
        position = success_point.get("position")
        ElementTree.SubElement(self.success, 'scPosition participant="{}" x="{}" y="{}" tolerance="{}"'
                               .format(participant_id, str(position[0]), str(position[1]),
                                       str(success_point.get("tolerance"))))

    def add_failure_damage(self, participant_id):
        """Adds damage observation as a test failure condition.
        :param participant_id: Participant id (string).
        :return: Void.
        """
        ElementTree.SubElement(self.failure, 'scDamage participant="{}"'.format(participant_id))

    def add_failure_lane(self, participant_id, lane="offroad"):
        """Adds lane following observation as a test failure condition.
        :param participant_id: participant id (string)
        :param lane: on which lane should the test fail? (markings, leftLanes, rightLanes, offroad).
        :return: Void.
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

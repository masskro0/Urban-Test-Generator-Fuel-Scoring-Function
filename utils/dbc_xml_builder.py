"""This class builds a criteria XML file for DriveBuild in the required format. Note that not
    every criteria or items can be added to the XML files, I chose only the ones that I need.
"""

import xml.etree.ElementTree as ElementTree


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

    def add_trigger_points(self, triggers):
        """
        :param triggers:
        :return:
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
            trigger.set("triggers", trigger_point.get("triggers"))
            if trigger_point.get("action") == "spawn_and_start":
                spawn_point = trigger_dict.get("spawnPoint")
                ElementTree.SubElement(trigger, 'spawnPoint x="{}" y="{}" orientation="{}"'
                                       .format(str(spawn_point.get("position")[0]),
                                               str(spawn_point.get("position")[1]),
                                               str(spawn_point.get("orientation"))))
            elif trigger_point.get("action") == "switchLights":
                trigger.set("initState", trigger_point.get("initState"))
                trigger.set("switchTo", trigger_point.get("switchTo"))

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
        color = participant.get("color")
        triggers = participant.get("triggerPoints")
        participant = ElementTree.SubElement(self.participants, "participant")
        participant.set("id", participant_id)
        participant.set("model", model)
        participant.set("color", color)
        ElementTree.SubElement(participant, 'initialState x="{}" y="{}"'
                                            ' orientation="{}" movementMode="{}"'
                                            ' speed="{}"'
                               .format(str(init_state.get("position")[0]), str(init_state.get("position")[1]),
                                       str(init_state.get("orientation")), init_state.get("movementMode"),
                                       str(init_state.get("speed"))))

        if waypoints is not None:
            movement = ElementTree.SubElement(participant, "movement")
            for waypoint in waypoints:
                position = waypoint.get("position")
                waypoint_tag = ElementTree.SubElement(movement, 'waypoint x="{}" y="{}" tolerance="{}"'
                                                                ' movementMode="{}" lane="{}"'
                                                      .format('{0:.10f}'.format(round(position[0], 2)),
                                                              '{0:.10f}'.format(round(position[1], 2)),
                                                              str(waypoint.get("tolerance")),
                                                              waypoint.get("movementMode"),
                                                              waypoint.get("lane")))
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

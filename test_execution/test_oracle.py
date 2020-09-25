from enum import Enum
from math import floor
from xml.etree.ElementTree import parse
from scipy.spatial.distance import euclidean
from termcolor import colored

from utils.utility_functions import get_angle, get_magnitude_of_3d_vector


class TrafficLightLabel:
    """Class to get the state of the next traffic light."""
    def __init__(self, traffic_light_list, traffic_triggers):
        """Initializes and declares a set of flags, time variables and lists.
        :param traffic_light_list: List of traffic light positions, e.g. by calling
         converter.get_traffic_lights_position().
        :param traffic_triggers: List of trigger points causing a light switch, e.g. by calling
         converter.traffic_triggers.
        """
        self.entered = False
        self.red_entered = False
        self.at_intersection = False                    # Flag whether the ego-car entered an intersection.
        self.trigger_entered = False                    # Bool whether the ego-car entered a trigger point or not.
        self.time_entry = 0                             # Simulation time when entered a trigger point.
        self.prev_time = 0                              # Previous recorded simluation time.
        self.traffic_light_list = traffic_light_list    # List with traffic lights.
        self.traffic_triggers = traffic_triggers        # List with trigger points for traffic lights.
        self.trigger_index = 0                          # Index for iterating the trigger point list.

        # Position of the trigger point.
        self.traffic_triggers_pos = None if len(traffic_triggers) == 0 \
            else (float(traffic_triggers[self.trigger_index]["x"]), float(traffic_triggers[self.trigger_index]["y"]))
        self.traffic_index = 0                          # Index for iterating the traffic light list.

        # Position of the next traffic light.
        self.traffic_light_pos = None if len(traffic_light_list) == 0 else \
            (float(traffic_light_list[self.traffic_index]["x"]),
             float(traffic_light_list[self.traffic_index]["y"]))

        # Initial state of the current traffic light.
        self.init_state = None if len(traffic_triggers) == 0 else traffic_triggers[self.trigger_index].get("initState")

        # Tolerance of the trigger point.
        self.tolerance = None if len(traffic_triggers) == 0 \
            else float(traffic_triggers[self.trigger_index].get("tolerance"))

        # The distance to the trigger point gets multiplied with a factor to switch the lights. Same rules as in
        # converter.py.
        self.multiplicator = 15 if self.init_state == "green" else 9
        self.label = self.init_state                     # State of the current traffic light.

    def get_traffic_light_label(self, time, ego_position, ego_direction):
        """Determines the traffic light state. Synced with the rules inside converter.py. Method might return None
           because LUA and Python are not synchronized, which specifies an undefined state.
        :param time: Simulation time.
        :param ego_position: 3D coordinates of the ego-car position.
        :param ego_direction: 3D coordinates of the ego-car direction vector.
        :return: Traffic light state.
        """
        if self.traffic_light_pos is not None:
            distance_light = euclidean(self.traffic_light_pos, (ego_position[0], ego_position[1]))
            if distance_light <= 10:
                # Did the ego-car entered an intersection?
                self.at_intersection = True
            elif distance_light > 10 and self.at_intersection:
                # Did the ego-car left an intersection?
                self.at_intersection = False
                self.traffic_index += 1
                if self.traffic_index >= len(self.traffic_light_list):
                    self.traffic_light_pos = None
                else:
                    # Get the next traffic light position.
                    self.traffic_light_pos = (float(self.traffic_light_list[self.traffic_index]["x"]),
                                              float(self.traffic_light_list[self.traffic_index]["y"]))
                # Label is the initial state of the next traffic light.
                self.label = self.init_state
            if self.traffic_index >= len(self.traffic_light_list):
                # Are there any traffic lights left?
                return None
            if self.traffic_light_list[self.traffic_index].get("mode") == "off":
                self.label = "off"
            elif self.traffic_light_list[self.traffic_index].get("mode") == "flashing":
                if floor(time) % 2 == 0:
                    # Every even second, the traffic light is off.
                    self.label = "off"
                else:
                    # Every odd second, the traffic light is yellow.
                    self.label = "yellow"
            elif self.traffic_triggers_pos is not None:
                # Manual mode.
                p0 = (ego_position[0] + ego_direction[0], ego_position[1] + ego_direction[1])
                p1 = (ego_position[0], ego_position[1])

                # Angle between ego-car and traffic light.
                angle = get_angle(self.traffic_light_pos, p1, p0)
                distance_trigger = euclidean(self.traffic_triggers_pos, (ego_position[0], ego_position[1]))
                if distance_trigger > self.tolerance * self.multiplicator and self.time_entry == 0:
                    # The traffic light state stays the same until a trigger point is entered.
                    self.label = self.init_state
                else:
                    if self.init_state == "red":
                        if self.tolerance * self.multiplicator - 4.5 < distance_trigger \
                                < self.tolerance * self.multiplicator and self.time_entry < 3:
                            # Create a tolerance range where the state can be undefined while switching to yellow-red.
                            if not self.entered:
                                self.label = "yellow-red"
                                self.entered = True
                                self.prev_time = time
                            else:
                                self.time_entry += time - self.prev_time
                                self.prev_time = time
                            return None
                        if not self.entered:
                            self.label = "yellow-red"
                            self.entered = True
                            self.prev_time = time
                        else:
                            self.time_entry += time - self.prev_time
                            self.prev_time = time
                            if 18 >= distance_trigger >= 16.1 or 1.1 <= self.time_entry < 1.5:
                                # Create undefined distance and time range.
                                return None
                            if self.trigger_entered and angle > 110:
                                # Get next traffic light position because ego-car left the intersection.
                                self.prev_time = 0
                                self.time_entry = 0
                                self.trigger_index += 1
                                self.entered = False
                                self.trigger_entered = False
                                self.traffic_triggers_pos = None if self.trigger_index >= len(
                                    self.traffic_triggers) else \
                                    (float(self.traffic_triggers[self.trigger_index]["x"]),
                                     float(self.traffic_triggers[self.trigger_index]["y"]))
                                self.init_state = None if self.trigger_index >= len(self.traffic_triggers) else \
                                    self.traffic_triggers[self.trigger_index].get("initState")
                                self.tolerance = None if self.trigger_index >= len(self.traffic_triggers) else \
                                    float(self.traffic_triggers[self.trigger_index].get("tolerance"))
                                self.multiplicator = 15 if self.init_state is None or self.init_state == "green" \
                                    else 9
                                self.label = self.init_state
                            elif self.time_entry >= 1.4:
                                self.trigger_entered = True
                                self.label = "green"
                    else:
                        # Initial state is green.
                        if 26 <= distance_trigger <= 34 and not self.trigger_entered:
                            # In LUA, light switches to yellow at distance 30. Again, I create here an undefined range.
                            if distance_trigger <= 30:
                                if not self.entered:
                                    self.label = "yellow"
                                    self.entered = True
                                    self.prev_time = time
                                else:
                                    self.time_entry += time - self.prev_time
                                    self.prev_time = time
                            return None
                        elif 7 < distance_trigger <= 12:
                            # At distance ten, the light should be red in LUA.
                            self.trigger_entered = True
                            if distance_trigger <= 10:
                                if not self.red_entered:
                                    self.label = "red"
                                    self.red_entered = True
                                    self.time_entry = 0
                                    self.prev_time = time
                                else:
                                    self.time_entry += time - self.prev_time
                                    self.prev_time = time
                                    if 6.6 <= self.time_entry <= 7.4 or 7.70 <= self.time_entry <= 8.3:
                                        return None
                                    if self.time_entry >= 8:
                                        self.label = "green"
                                        self.prev_time = 0
                                        self.time_entry = 0
                                        self.trigger_index += 1
                                        self.entered = False
                                        self.red_entered = False
                                        self.trigger_entered = False
                                        self.traffic_triggers_pos = None if self.trigger_index >= len(
                                            self.traffic_triggers) else \
                                            (float(self.traffic_triggers[self.trigger_index]["x"]),
                                             float(self.traffic_triggers[self.trigger_index]["y"]))
                                        self.init_state = None if self.trigger_index >= len(self.traffic_triggers) \
                                            else self.traffic_triggers[self.trigger_index].get("initState")
                                        self.tolerance = None if self.trigger_index >= len(self.traffic_triggers) \
                                            else float(self.traffic_triggers[self.trigger_index].get("tolerance"))
                                        self.multiplicator = 15 if self.init_state == "green" else 9
                                    elif self.time_entry >= 7:
                                        self.label = "yellow-red"
                            return None
                        elif distance_trigger <= 30 or self.trigger_entered:
                            if not self.entered:
                                self.label = "yellow"
                                self.entered = True
                                self.prev_time = time
                            else:
                                self.time_entry += time - self.prev_time
                                self.prev_time = time
                                if 6.6 <= self.time_entry <= 7.4 or 7.70 <= self.time_entry <= 8.3:
                                    return None
                                if self.time_entry >= 8:
                                    self.label = "green"
                                    self.prev_time = 0
                                    self.time_entry = 0
                                    self.trigger_index += 1
                                    self.entered = False
                                    self.red_entered = False
                                    self.trigger_entered = False
                                    self.traffic_triggers_pos = None if self.trigger_index >= len(
                                        self.traffic_triggers) else \
                                        (float(self.traffic_triggers[self.trigger_index]["x"]),
                                         float(self.traffic_triggers[self.trigger_index]["y"]))
                                    self.init_state = None if self.trigger_index >= len(self.traffic_triggers) \
                                        else self.traffic_triggers[self.trigger_index].get("initState")
                                    self.tolerance = None if self.trigger_index >= len(self.traffic_triggers) \
                                        else float(self.traffic_triggers[self.trigger_index].get("tolerance"))
                                    self.multiplicator = 15 if self.init_state == "green" else 9
                                elif self.time_entry >= 7:
                                    self.label = "yellow-red"
            return self.label
        return None


class TestCaseState(Enum):
    """Enums for test case states. SUCCESS means that the test case is over and the ego-car entered a success point,
       FAILED means that the test case is over and the ego-car violates a test oracle, OK means that the test case is
       still running and the ego-car did not violate any test oracles yet.
    """
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    OK = "OK"


class TestOracle:
    """Class for verifying test specific oracles. They determine whether a test case has failed or not."""
    def __init__(self, dbc, dbe):
        self.state = TestCaseState.OK           # Current test case state.
        self.dbc_root = parse(dbc).getroot()    # Root of the criteria XML file.
        self.dbe_root = parse(dbe).getroot()    # Root of the environment XML file.
        self.success_states = list()            # List of success states.
        self._get_success_states()
        self._get_timeout()                     # Sets the timeout variable.
        self.damage_requests = list()           # List of vehicle IDs which are not allowed to take damage.
        self._get_damage_states()
        self.ego_signs = list()                 # List of traffic lights/signs which the ego-car must pass by.
        self._get_ego_signs()
        self.sign_index = 0                     # Index to iterate over the ego_signs list.
        self.still_standing = 0                 # Time variable to determine how long the ego-car has been not moving..
        self.prev_time = 0                      # Previous recorded simulation time.

    def _get_success_states(self):
        """Gets all success positions.
        :return: Void.
        """
        success_points = self.dbc_root.findall("success/scPosition")
        for sp in success_points:
            attr = sp.attrib
            self.success_states.append(attr)
        assert len(self.success_states) != 0, "You need at least one success point."

    def _get_timeout(self):
        """Sets the timeout variable.
        :return: Void.
        """
        timeout = self.dbc_root.find("failure/timeout")
        self.timeout = 180 if timeout is None else int(timeout.attrib.get("timeout"))

    def _get_damage_states(self):
        """Gets all damage requests.
        :return: Void.
        """
        damages = self.dbc_root.findall("failure/scDamage")
        for damage in damages:
            self.damage_requests.append(damage.attrib)

    def _get_ego_signs(self):
        """Gets all traffic lights and signs which the ego-car must pass by.
        :return: Void.
        """
        obstacles = self.dbe_root.find("obstacles")
        if obstacles is None:
            obstacles = list()
        for obstacle in obstacles:
            o_attr = obstacle.attrib
            if o_attr.get("facingEgo") is not None and o_attr.get("facingEgo"):
                self.ego_signs.append(o_attr)
                self.ego_signs[-1]["kind"] = obstacle.tag
                if o_attr.get("oid") is not None:
                    trigger_points = self.dbc_root.findall("triggerPoints/triggerPoint")
                    for trigger in trigger_points:
                        t_attr = trigger.attrib
                        if t_attr.get("action") == "switchLights" and t_attr.get("triggers") == o_attr.get("oid"):
                            self.ego_signs[-1]["init_state"] = t_attr.get("initState")
                            self.ego_signs[-1]["switchTo"] = t_attr.get("switchTo")

    def _validate_success_state(self, states):
        """Validates all success states.
        :param states: List of dicts containing vehicle ID and its position.
        :return: Void.
        """
        for sstate in self.success_states:
            break_flag = False
            for state in states:
                if sstate.get("participant") == state.get("id"):
                    pos = state.get("pos")
                    if euclidean((float(sstate.get("x")), float(sstate.get("y"))), (float(pos[0]), float(pos[1]))) \
                            < float(sstate.get("tolerance")):
                        self.state = TestCaseState.SUCCESS
                        break_flag = True
                        print(colored("TEST SUCCEEDED. ENTERED SUCCESS POINT.", "green", attrs=['bold']))
                        break
            if break_flag:
                break

    def _validate_timeout(self, timer):
        """Validates simulation timeout.
        :param timer: Simulation time.
        :return: Void.
        """
        if timer >= self.timeout:
            self.state = TestCaseState.FAILED
            print(colored("TEST FAILED. TIMEOUT", "red", attrs=['bold']))

    def _validate_damage(self, damage_states):
        """Validates damage requests.
        :param damage_states: Damage states of all participants.
        :return: Void.
        """
        for request in self.damage_requests:
            break_flag = False
            for state in damage_states:
                if state.get("id") == request.get("participant") and float(state.get("damage")) > 0:
                    self.state = TestCaseState.FAILED
                    break_flag = True
                    print(colored("TEST FAILED. \"{}\" CRASHED INTO ANOTHER CAR".format(state.get("id")),
                                  "red", attrs=['bold']))
                    break
            if break_flag:
                break

    def _validate_traffic_rules(self, ego_position, ego_direction, ego_velocity, timer, label):
        """Validates traffic rules.
        :param ego_position: 3D ego-car position.
        :param ego_direction: 3D ego-car direction vector.
        :param ego_velocity: 3D ego-car velocity vector.
        :param timer: Simulation time.
        :param label: Traffic light state.
        :return: Void.
        """
        # NOTE: Yield sign rules and traffic are not respected at the moment. This would be very complex.
        if len(self.ego_signs) != 0 and self.sign_index < len(self.ego_signs):
            traffic_sign_pos = (
                float(self.ego_signs[self.sign_index]["x"]), float(self.ego_signs[self.sign_index]["y"]))
            p0 = (ego_position[0] + ego_direction[0], ego_position[1] + ego_direction[1])
            p1 = (ego_position[0], ego_position[1])
            angle = get_angle(traffic_sign_pos, p1, p0)
            distance_sign = euclidean(traffic_sign_pos, p1)
            vel = get_magnitude_of_3d_vector(ego_velocity) * 3.6
            sign = self.ego_signs[self.sign_index]
            if sign.get("kind").startswith("stop"):
                # Stop sign traffic rule validation.
                if distance_sign < 10 and vel < 0.5:
                    if self.prev_time == 0:
                        self.prev_time = timer
                    else:
                        self.still_standing += timer - self.prev_time
                        self.prev_time = timer
                if distance_sign < 10 and angle > 80:
                    if self.still_standing < 3:
                        self.state = TestCaseState.FAILED
                        print(colored("TEST FAILED. \"ego\" DIDN'T STOP AT A STOP SIGN.", "red", attrs=['bold']))
                    else:
                        self.still_standing = 0
                        self.prev_time = 0
                        self.sign_index += 1
            elif sign.get("kind").startswith("priority") or (sign.get("kind").startswith("trafficlight")
                                                             and sign.get("mode") != "manual"
                                                             and sign.get("sign") == "priority"):
                # Validate priority sign rules.
                distance_limit = 20 if sign.get("kind").startswith("trafficlight") else 10
                if distance_sign < distance_limit:
                    if vel < 0.5:
                        self.state = TestCaseState.FAILED
                        print(colored("TEST FAILED. \"ego\" STOPPED AT A PRIORITY SIGN.", "red", attrs=['bold']))
                    elif angle > 80:
                        self.sign_index += 1
            elif sign.get("kind").startswith("trafficlight") and sign.get("mode") == "manual":
                # Validate traffic light rules.
                if distance_sign < 20:
                    if angle > 80:
                        if label == "red" and vel >= 0.5:
                            self.state = TestCaseState.FAILED
                            print(colored("TEST FAILED. \"ego\" DIDN'T STOP AT A RED TRAFFIC LIGHT.", "red",
                                          attrs=['bold']))
                        else:
                            self.sign_index += 1
                            self.still_standing = 0
                            self.prev_time = 0
                    elif label == "green" and vel < 0.5:
                        if self.prev_time == 0:
                            self.prev_time = timer
                        else:
                            self.still_standing += timer - self.prev_time
                            self.prev_time = timer
                        if self.still_standing > 2:
                            self.state = TestCaseState.FAILED
                            print(colored("TEST FAILED. \"ego\" STOPPED AT A GREEN TRAFFIC LIGHT.", "red",
                                      attrs=['bold']))

    def validate_test_case(self, states, ego_position, ego_direction, ego_velocity, timer, label, damage_states):
        """Validate test oracles.
        :param states: List of dicts containing vehicle ID and position.
        :param ego_position: 3D ego-car position.
        :param ego_direction: 3D ego-car direction vector.
        :param ego_velocity: 3D ego-car velocity vector.
        :param timer: Simulation time.
        :param label: Traffic light state.
        :param damage_states: List of damage sensor values.
        :return: Void.
        """
        self._validate_traffic_rules(ego_position, ego_direction, ego_velocity, timer, label)
        self._validate_success_state(states)
        self._validate_damage(damage_states)
        self._validate_timeout(timer)

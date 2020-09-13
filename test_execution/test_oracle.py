from enum import Enum
from math import floor
from xml.etree.ElementTree import parse
from scipy.spatial.distance import euclidean
from termcolor import colored

from utils.utility_functions import get_angle, get_magnitude_of_3d_vector


class TrafficLightLabel:
    def __init__(self, traffic_light_list, traffic_triggers):
        self.entered = False
        self.red_entered = False
        self.at_intersection = False
        self.trigger_entered = False
        self.time_entry = 0
        self.prev_time = 0
        self.traffic_light_list = traffic_light_list  # converter.get_traffic_lights_position()
        self.traffic_triggers = traffic_triggers  # converter.traffic_triggers
        self.trigger_index = 0
        self.traffic_triggers_pos = None if len(traffic_triggers) == 0 \
            else (float(traffic_triggers[self.trigger_index]["x"]), float(traffic_triggers[self.trigger_index]["y"]))
        self.traffic_index = 0
        self.traffic_light_pos = None if len(traffic_light_list) == 0 else \
            (float(traffic_light_list[self.traffic_index]["x"]),
             float(traffic_light_list[self.traffic_index]["y"]))
        self.init_state = None if len(traffic_triggers) == 0 else traffic_triggers[self.trigger_index].get("initState")
        self.tolerance = None if len(traffic_triggers) == 0 \
            else float(traffic_triggers[self.trigger_index].get("tolerance"))
        self.multiplicator = 15 if self.init_state == "green" else 9
        self.label = self.init_state

    def get_traffic_light_label(self, time, ego_state):
        if self.traffic_light_pos is not None:
            distance_light = euclidean(self.traffic_light_pos, (ego_state["pos"][0], ego_state["pos"][1]))
            if distance_light <= 10:
                self.at_intersection = True
            elif distance_light > 10 and self.at_intersection:
                self.at_intersection = False
                self.traffic_index += 1
                if self.traffic_index >= len(self.traffic_light_list):
                    self.traffic_light_pos = None
                else:
                    self.traffic_light_pos = (float(self.traffic_light_list[self.traffic_index]["x"]),
                                              float(self.traffic_light_list[self.traffic_index]["y"]))
                self.label = self.init_state
            if self.traffic_index >= len(self.traffic_light_list):
                return None
            if self.traffic_light_list[self.traffic_index].get("mode") == "off":
                self.label = "off"
            elif self.traffic_light_list[self.traffic_index].get("mode") == "blinking":
                if floor(time) % 2 == 0:
                    self.label = "off"
                else:
                    self.label = "yellow"
            elif self.traffic_triggers_pos is not None:
                p0 = (ego_state["pos"][0] + ego_state["dir"][0], ego_state["pos"][1] + ego_state["dir"][1])
                p1 = (ego_state["pos"][0], ego_state["pos"][1])
                angle = get_angle(self.traffic_light_pos, p1, p0)
                distance_trigger = euclidean(self.traffic_triggers_pos, (ego_state["pos"][0], ego_state["pos"][1]))
                if distance_trigger > self.tolerance * self.multiplicator and self.time_entry == 0:
                    self.label = self.init_state
                else:
                    if self.init_state == "red":
                        if self.tolerance * self.multiplicator - 4.5 < distance_trigger \
                                < self.tolerance * self.multiplicator and self.time_entry < 3:
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
                                return None
                            if self.trigger_entered and angle > 110:
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
                        if 26 <= distance_trigger <= 34 and not self.trigger_entered:
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
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    OK = "OK"


class TestOracle:
    # TODO Test oracle: out of street
    def __init__(self, scenario, dbc, dbe):
        self.scenario = scenario
        self.dbc = dbc
        self.dbe = dbe
        self.state = TestCaseState.OK
        self.dbc_root = parse(dbc).getroot()
        self.dbe_root = parse(dbe).getroot()
        self.success_states = list()
        self._get_success_states()
        self._get_timeout()
        self.damage_requests = list()
        self._get_damage_states()
        self.ego_signs = list()
        self._get_ego_signs()
        self.sign_index = 0
        self.still_standing = 0
        self.prev_time = 0

    def _get_success_states(self):
        success_points = self.dbc_root.findall("success/scPosition")
        for sp in success_points:
            attr = sp.attrib
            self.success_states.append(attr)
        assert len(self.success_states) != 0, "You need at least one success point."

    def _get_timeout(self):
        timeout = self.dbc_root.find("failure/timeout")
        self.timeout = 180 if timeout is None else int(timeout.attrib.get("timeout"))

    def _get_damage_states(self):
        damages = self.dbc_root.findall("failure/scDamage")
        for damage in damages:
            self.damage_requests.append(damage.attrib)

    def _get_ego_signs(self):
        obstacles = self.dbe_root.find("obstacles")
        if obstacles is None:
            obstacles = list()
        for obstacle in obstacles:
            attr = obstacle.attrib
            if attr.get("facingEgo") is not None and attr.get("facingEgo"):
                self.ego_signs.append(attr)
                self.ego_signs[-1]["kind"] = obstacle.tag
                if attr.get("oid") is not None:
                    trigger_points = self.dbc_root.findall("triggerPoints/triggerPoint")
                    for trigger in trigger_points:
                        tattr = trigger.attrib
                        if tattr.get("action") == "switchLights" and tattr.get("triggers") == attr.get("oid"):
                            self.ego_signs[-1]["init_state"] = tattr.get("initState")
                            self.ego_signs[-1]["switchTo"] = tattr.get("switchTo")

    def _validate_success_state(self, states):
        for sstate in self.success_states:
            break_flag = False
            for state in states:
                if sstate.get("participant") == state.get("id"):
                    pos = state.get("state").get("pos")
                    if euclidean((float(sstate.get("x")), float(sstate.get("y"))), (float(pos[0]), float(pos[1]))) \
                            < float(sstate.get("tolerance")):
                        self.state = TestCaseState.SUCCESS
                        break_flag = True
                        print(colored("TEST SUCCEEDED. ENTERED SUCCESS POINT.", "green", attrs=['bold']))
                        break
            if break_flag:
                break

    def _validate_timeout(self, timer):
        if timer >= self.timeout:
            self.state = TestCaseState.FAILED
            print(colored("TEST FAILED. TIMEOUT", "red", attrs=['bold']))

    def _validate_damage(self, damage_states):
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

    def _validate_traffic_rules(self, ego_state, timer, label):
        # NOTE: Yield sign rules and traffic are not respected at the moment. This would be very complex.
        if len(self.ego_signs) != 0 and self.sign_index < len(self.ego_signs):
            traffic_sign_pos = (
                float(self.ego_signs[self.sign_index]["x"]), float(self.ego_signs[self.sign_index]["y"]))
            p0 = (ego_state["pos"][0] + ego_state["dir"][0], ego_state["pos"][1] + ego_state["dir"][1])
            p1 = (ego_state["pos"][0], ego_state["pos"][1])
            angle = get_angle(traffic_sign_pos, p1, p0)
            distance_sign = euclidean(traffic_sign_pos, p1)
            vel = get_magnitude_of_3d_vector(ego_state["vel"]) * 3.6
            sign = self.ego_signs[self.sign_index]
            if sign.get("kind").startswith("stop"):
                # Stop sign traffic rule validation.
                if distance_sign < 10 and vel < 1:
                    self.still_standing += timer - self.prev_time
                    self.prev_time = timer
                if distance_sign < 10 and angle > 65:
                    if self.still_standing < 3:
                        self.state = TestCaseState.FAILED
                        print(colored("TEST FAILED. \"ego\" DIDN'T STOP AT A STOP SIGN.", "red", attrs=['bold']))
                    else:
                        self.still_standing = 0
                        self.sign_index += 1
            elif sign.get("kind").startswith("priority") or (sign.get("kind").startswith("trafficlight")
                                                             and sign.get("mode") != "manual"
                                                             and sign.get("sign") == "priority"):
                distance_limit = 20 if sign.get("kind").startswith("trafficlight") else 10
                if distance_sign < distance_limit:
                    if vel < 4:
                        self.state = TestCaseState.FAILED
                        print(colored("TEST FAILED. \"ego\" STOPPED AT A PRIORITY SIGN.", "red", attrs=['bold']))
                    elif angle > 65:
                        self.sign_index += 1
            elif sign.get("kind").startswith("trafficlight") and sign.get("mode") == "manual":
                if distance_sign < 20:
                    if angle > 65:
                        if label == "red" and vel >= 1: #TODO geht auch 0 irgendwie?
                            self.state = TestCaseState.FAILED
                            print(colored("TEST FAILED. \"ego\" DIDN'T STOP AT A RED TRAFFIC LIGHT.", "red",
                                          attrs=['bold']))
                        else:
                            self.sign_index += 1
                    elif label == "green" and vel < 4:
                        self.state = TestCaseState.FAILED
                        print(colored("TEST FAILED. \"ego\" STOPPED AT A GREEN TRAFFIC LIGHT.", "red",
                                      attrs=['bold']))

    # Done.
    def validate_test_case(self, states, ego_state, timer, label, damage_states):
        self._validate_traffic_rules(ego_state, timer, label)
        self._validate_success_state(states)
        self._validate_damage(damage_states)
        self._validate_timeout(timer)

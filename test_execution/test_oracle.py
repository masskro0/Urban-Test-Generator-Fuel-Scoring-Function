from enum import Enum
from xml.etree.ElementTree import parse
from scipy.spatial.distance import euclidean


class TestCaseState(Enum):
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    OK = "OK"


class TestOracle():
    # TODO Test oracle: out of street, traffic rules
    def __init__(self, scenario, dbc, dbe):
        self.scenario = scenario
        self.dbc = dbc
        self.dbe = dbe
        self.state = TestCaseState.OK
        self.dbc_root = parse(dbc).getroot()
        self.dbe_root = parse(dbe).getroot()
        self.success_states = list()
        self._get_success_states()
        self.time = 0
        self._get_timeout()
        self.damage_requests = list()
        self._get_damage_states()

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

    def _check_success_state(self, states):
        for sstate in self.success_states:
            break_flag = False
            for state in states:
                if sstate.get("participant") == state.get("participant"):
                    if euclidean((sstate.get("x"), sstate.get("y")), (state[0], state[1])) < sstate.get("tolerance"):
                        self.state = TestCaseState.SUCCESS
                        break_flag = True
                        break
            if break_flag:
                break

    def _check_timeout(self, timer):
        self.time += timer
        if self.time >= self.timeout:
            self.state = TestCaseState.FAILED

    def _check_damage(self, damage_states):
        for request in self.damage_requests:
            break_flag = False
            for state in damage_states:
                if state.get("participant") == request.get("participant") and state.damage > 0:
                    self.state = TestCaseState.FAILED
                    break_flag = True
                    break
            if break_flag:
                break


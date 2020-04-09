from beamngpy import Vehicle
from beamngpy.sensors import Electrics


class MisbehaviourObserver:
    def __init__(self, vehicle: Vehicle):
        self.vehicle = vehicle
        self.log = dict()
        self.throttle_misbehavior = list()              
        self.rpm_misbehavior = list()
        self.accelerate_and_stop_misbehavior = list()
        self.left_turn_misbehavior = list()
        self.right_turn_misbehavior = list()
        self.brake_misbehavior = list()
        self.safety_distance_misbehavior = list()
        self.braking_distance_misbehavior = list()
        self.engine_idle_misbehavior = list()
        self.log = {
            "throttle_misbehavior": self.throttle_misbehavior,
            "rpm_misbehavior": self.rpm_misbehavior,
            "fuel_consumption": None,
            "accelerate_and_stop_misbehavior": self.accelerate_and_stop_misbehavior,
            "left_turn_misbehavior": self.left_turn_misbehavior,
            "right_turn_misbehavior": self.right_turn_misbehavior,
            "brake_misbehavior": self.brake_misbehavior,
            "safety_distance_misbehavior": self.safety_distance_misbehavior,
            "braking_distance_misbehavior": self.braking_distance_misbehavior,
            "engine_idle_misbehavior": self.engine_idle_misbehavior
        }

    def get_results(self) -> dict:
        return self.log

    def calculate_fuel_consumption(self, fuel_state):
        pass

    def _check_throttle_misbehavior(self) -> None:
        pass

    def _check_rpm_misbehavior(self) -> None:
        pass

    def _check_accelerate_and_stop_misbehavior(self) -> None:
        pass

    def _check_left_turn_misbehavior(self) -> None:
        pass

    def _check_right_turn_misbehavior(self) -> None:
        pass

    def _check_brake_misbehavior(self) -> None:
        pass

    def _check_safety_distance_misbehavior(self) -> None:
        pass

    def _check_braking_distance_misbehavior(self) -> None:
        pass

    def _check_engine_idle_misbehavior(self) -> None:
        pass

    def check_misbehavior(self) -> None:
        self._check_accelerate_and_stop_misbehavior()
        self._check_brake_misbehavior()
        self._check_braking_distance_misbehavior()
        self._check_engine_idle_misbehavior()
        self._check_left_turn_misbehavior()
        self._check_right_turn_misbehavior()
        self._check_rpm_misbehavior()
        self._check_safety_distance_misbehavior()
        self._check_throttle_misbehavior()

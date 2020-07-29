from engine_types import EngineType


class MisbehaviourObserver:
    def __init__(self, engine_type: EngineType = EngineType.PETROL):
        """Class for observing faulty behavior of a car concerning fuel efficiency.
        :param engine_type: Engine/Fuel type of a car. Check the file engine_types.py for more information.
        """
        self.engine_type = engine_type
        self.log = dict()
        self.throttle_misbehavior = 0
        self.rpm_infractions = 0
        self.accelerate_and_stop_misbehavior = 0
        self.left_turn_misbehavior = 0
        self.right_turn_misbehavior = 0
        self.brake_misbehavior = 0
        self.safety_distance_misbehavior = 0
        self.engine_idle_misbehavior = 0
        self.score = 0
        self.log = {
            "throttle_misbehavior": self.throttle_misbehavior,
            "rpm_misbehavior": self.rpm_infractions,
            "fuel_consumption": None,
            "accelerate_and_stop_misbehavior": self.accelerate_and_stop_misbehavior,
            "left_turn_misbehavior": self.left_turn_misbehavior,
            "right_turn_misbehavior": self.right_turn_misbehavior,
            "brake_misbehavior": self.brake_misbehavior,
            "safety_distance_misbehavior": self.safety_distance_misbehavior,
            "engine_idle_misbehavior": self.engine_idle_misbehavior
        }

    def get_results(self):
        """Returns the log file with all logged misbehavior actions.
        :return: log file as a dict.
        """
        return self.log

    def calculate_fuel_consumption(self, fuel_state):
        pass

    def _check_throttle_misbehavior(self):
        pass

    def _check_rpm_infraction(self, rpm):
        """Checks whether the rpm is above the allowed limit and logs the global position of the car.
        :param rpm: Current rpm of the vehicle.
        :return: None.
        """
        upper_limit = self.engine_type.get_rpm_shifting_sweetspots()['upper_limit']
        if rpm > upper_limit:
            self.rpm_infractions += 1
            increase = 180
            if rpm >= upper_limit + increase * 9:
                self.score += 9
            else:
                i = 1
                while i < 10:
                    if rpm < upper_limit + increase * i:
                        self.score += i
                        break
                    i += 1

    def _check_accelerate_and_stop_misbehavior(self):
        pass

    def _check_left_turn_misbehavior(self):
        pass

    def _check_right_turn_misbehavior(self):
        pass

    def _check_brake_misbehavior(self):
        pass

    def _check_safety_distance_misbehavior(self):
        pass

    def _check_engine_idle_misbehavior(self):
        pass

    def check_misbehavior(self, sensors, vehicle_state):
        """Checks for faulty behaviors of a car at the current state and logs it.
        :param sensors: Sensors of a car. Only the electrics sensor is needed.
        :param vehicle_state: Vehicle state information in the simulation.
        :return: None.
        """
        #print(sensors)
        #print(vehicle_state)
        self._check_accelerate_and_stop_misbehavior()
        self._check_brake_misbehavior()
        self._check_engine_idle_misbehavior()
        self._check_left_turn_misbehavior()
        self._check_right_turn_misbehavior()
        self._check_rpm_infraction(sensors['electrics']['values']['rpmTacho'])
        self._check_safety_distance_misbehavior()
        self._check_throttle_misbehavior()

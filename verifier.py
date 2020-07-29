from engine_types import EngineType


class MisbehaviourObserver:
    def __init__(self, engine_type: EngineType = EngineType.PETROL):
        """Class for observing faulty behavior of a car concerning fuel efficiency.
        :param engine_type: Engine/Fuel type of a car. Check the file engine_types.py for more information.
        """
        self.engine_type = engine_type
        self.log = dict()
        self.throttle_infractions = 0
        self.rpm_infractions = 0
        self.accelerate_and_stop_infractions = 0
        self.left_turn_infractions = 0
        self.right_turn_infractions = 0
        self.brake_infractions = 0
        self.safety_distance_infractions = 0
        self.engine_idle_infractions = 0
        self.score = 0
        self.second_passed = False
        self.log = {
            "throttle_misbehavior": self.throttle_infractions,
            "rpm_misbehavior": self.rpm_infractions,
            "fuel_consumption": None,
            "accelerate_and_stop_misbehavior": self.accelerate_and_stop_infractions,
            "left_turn_misbehavior": self.left_turn_infractions,
            "right_turn_misbehavior": self.right_turn_infractions,
            "brake_misbehavior": self.brake_infractions,
            "safety_distance_misbehavior": self.safety_distance_infractions,
            "engine_idle_misbehavior": self.engine_idle_infractions
        }

    def get_results(self):
        """Returns the log file with all logged misbehavior actions.
        :return: log file as a dict.
        """
        return self.log

    def get_score(self):
        """Returns the score.
        :return: Score as int type.
        """
        return self.score

    def _check_throttle_infraction(self, throttle):
        """Checks whether the throttle value is over the upper limit.
        :param throttle: Throttle value.
        :return: Void.
        """
        upper_limit = 0.45
        if throttle >= upper_limit:
            self.throttle_infractions += 1
            interval = 0.04
            if throttle >= upper_limit + 9 * interval:
                self.score += 9
            else:
                i = 1
                while i < 10:
                    if throttle <= upper_limit + interval * i:
                        self.score += i
                        break
                    i += 1

    def _check_rpm_infraction(self, rpm):
        """Checks whether the rpm is above the allowed limit and logs the global position of the car.
        :param rpm: Current rpm of the vehicle.
        :return: Void.
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
                    if rpm <= upper_limit + increase * i:
                        self.score += i
                        break
                    i += 1

    def _check_accelerate_and_stop_infraction(self):
        pass

    def _check_left_turn_infraction(self):
        pass

    def _check_right_turn_infraction(self):
        pass

    def _check_safety_distance_infraction(self):
        pass

    def _check_brake_infraction(self, brake):
        """Checks whether the brake value is over the upper limit.
        :param brake: Brake value.
        :return: Void.
        """
        upper_limit = 0.45
        if brake >= upper_limit:
            self.brake_infractions += 1
            interval = (1 - upper_limit) / 5
            if brake >= upper_limit + 5 * interval:
                self.score += 5
            else:
                i = 1
                while i < 6:
                    if brake <= upper_limit + interval * i:
                        self.score += i
                        break
                    i += 1

    def set_fuel_value(self, value):
        """Sets the fuel value.
        :param value: Fuel value.
        :return: Void.
        """
        self.log["fuel_consumption"] = value

    def _check_engine_idle_infraction(self, wheelspeed, running):
        """Checks if the engine is in idle mode when the speed is 0 after one second.
        :param wheelspeed: Wheel speed of the car.
        :param running: {@code True} is the engine of the car is running, else {@code False}.
        :return: Void.
        """
        velocity = wheelspeed * 3.6
        if velocity < 0.1:
            if running and self.second_passed:
                self.second_passed = False
                self.engine_idle_infractions += 1
                self.score += 3
            else:
                self.second_passed = True
        else:
            self.second_passed = False

    def check_infraction(self, sensors, vehicle_state):
        """Checks for faulty behaviors of a car at the current state and logs it.
        :param sensors: Sensors of a car. Only the electrics sensor is needed.
        :param vehicle_state: Vehicle state information in the simulation.
        :return: Void.
        """
        electrics = sensors['electrics']['values']
        print(electrics['brake'])
        self._check_accelerate_and_stop_infraction()
        self._check_brake_infraction(electrics['brake'])
        self._check_engine_idle_infraction(electrics['wheelspeed'], electrics['running'])
        self._check_left_turn_infraction()
        self._check_right_turn_infraction()
        self._check_rpm_infraction(electrics['rpmTacho'])
        self._check_safety_distance_infraction()
        self._check_throttle_infraction(electrics['throttle'])
        self.set_fuel_value(electrics['fuel'])

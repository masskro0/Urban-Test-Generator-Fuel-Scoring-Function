from test_execution.engine_types import EngineType


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
        self.brake_infractions = 0
        self.engine_idle_infractions = 0
        self.score = 0
        self.prev_time = 0
        self.log = {
            "throttle_misbehavior": self.throttle_infractions,
            "rpm_misbehavior": self.rpm_infractions,
            "fuel": None,
            "consumed_fuel": None,
            "accelerate_and_stop_misbehavior": self.accelerate_and_stop_infractions,
            "brake_misbehavior": self.brake_infractions,
            "engine_idle_misbehavior": self.engine_idle_infractions
        }
        self.cache = {"last_actions": list(), "values": list()}

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

    def _validate_throttle_infraction(self, throttle):
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

    def _validate_brake_infraction(self, brake):
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

    def _validate_rpm_infraction(self, rpm):
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

    def _validate_accelerate_and_stop_infraction(self, throttle, brake):
        """Checks for accelerate and stop behaviour of cars. Uses the local cache of this class.
        :param throttle: Throttle value.
        :param brake: Brake value.
        :return: Void.
        """
        soft_limit = 0.1
        time_boundary = 4
        score_limit = 8
        new_action = None
        new_value = None
        values = self.cache.get("values")
        if throttle >= soft_limit:
            new_action = "throttle"
            new_value = throttle
        elif brake >= soft_limit:
            new_action = "brake"
            new_value = brake
        if new_action is None:
            self.cache["last_actions"] = list()
            self.cache["values"] = list()
        elif len(self.cache.get("last_actions")) == 0 or self.cache.get("last_actions")[-1] == new_action:
            self.cache["last_actions"].append(new_action)
            self.cache["values"].append(new_value)
        else:
            if len(self.cache.get("last_actions")) <= time_boundary:
                self.accelerate_and_stop_infractions += 1
                infraction_score = 0
                i = 0
                while i < len(values):
                    if i == time_boundary:
                        break
                    infraction_score += values[-(i+1)]
                    i += 1
                infraction_score *= 10
                if infraction_score > score_limit:
                    infraction_score = score_limit
                self.score += infraction_score
            self.cache["last_actions"] = [new_action]
            self.cache["values"] = [new_value]

    def set_fuel_value(self, value):
        """Sets the fuel value and calculates the consumed fuel.
        :param value: Fuel value.
        :return: Void.
        """
        self.log["fuel"] = value
        self.log["consumed_fuel"] = 1 - value

    def _validate_engine_idle_infraction(self, wheelspeed, running, time):
        """Checks if the engine is in idle mode when the speed is 0 after one second.
        :param wheelspeed: Wheel speed of the car.
        :param running: {@code True} is the engine of the car is running, else {@code False}.
        :return: Void.
        """
        velocity = wheelspeed * 3.6
        if velocity < 0.1:
            if running and time - self.prev_time >= 1:
                self.prev_time = time
                self.engine_idle_infractions += 1
                self.score += 4
        else:
            self.prev_time = time

    def validate_infraction(self, sensors, time):
        """Checks for faulty behaviors of a car at the current state and logs it.
        :param time: Simulation time.
        :param sensors: Sensors of a car. Only the electrics sensor is needed.
        :return: Void.
        """
        electrics = sensors['electrics']['values']
        self._validate_accelerate_and_stop_infraction(electrics['throttle'], electrics['brake'])
        self._validate_brake_infraction(electrics['brake'])
        self._validate_engine_idle_infraction(electrics['wheelspeed'], electrics['running'], time)
        self._validate_rpm_infraction(electrics['rpmTacho'])
        self._validate_throttle_infraction(electrics['throttle'])
        self.set_fuel_value(electrics['fuel'])

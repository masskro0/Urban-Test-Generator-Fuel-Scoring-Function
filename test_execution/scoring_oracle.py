from test_execution.engine_types import EngineType


class ScoringOracle:
    """Class for validating fuel-specific test oracles. The scoring function assesses the fuel-(in)efficiency
       an ego-car.
    """
    def __init__(self, engine_type: EngineType = EngineType.PETROL, max_gear=6):
        """Initializes the variables.
        :param engine_type: Engine type of a car. Check the file engine_types.py for more information.
        """
        self.engine_type = engine_type                              # Engine type of the test subject
        self.rpm_infractions = 0                                    # Number of RPM (rounds per minute) infractions
        self.throttle_infractions = 0                               # Number of throttle infractions
        self.brake_infractions = 0                                  # Number of brake infractions
        self.engine_idle_infractions = 0                            # Number of engine idle infractions
        self.accelerate_and_stop_infractions = 0                    # Number of accelerate-and-stop infractions
        self.fuel = 0                                               # Current amount of fuel
        self.consumed_fuel = 0                                      # Consumed fuel
        self.score = 0                                              # Current score
        self.prev_time = 0                                          # Last time the engine idling oracle was called
        self.prev_time_called = 0                                   # Last time the scoring oracle class was called
        self.max_gear = max_gear                                    # Maximum possible gear of the ego-car
        self.cache = {"last_actions": list(), "values": list()}     # Cache of actions for a-a-s infraction

    def get_results(self):
        """Returns a dict with the number of infractions, current fuel, consumed fuel and score.
        :return: Dict file with results.
        """
        return {
            "rpm_infractions": self.rpm_infractions,
            "throttle_infractions": self.throttle_infractions,
            "brake_infractions": self.brake_infractions,
            "engine_idle_infractions": self.engine_idle_infractions,
            "accelerate_and_stop_infractions": self.accelerate_and_stop_infractions,
            "fuel": self.fuel,
            "consumed_fuel": self.consumed_fuel,
            "score": self.score
        }

    def _validate_rpm_infraction(self, rpm, gear):
        """Validates whether the RPM is above the allowed limit.
        :param rpm: Current rpm of the vehicle.
        :param gear: Current gear of the vehicle.
        :return: Void.
        """
        upper_limit = self.engine_type.get_rpm_shifting_sweetspots()['upper_limit'] + 100
        if rpm > upper_limit and gear < self.max_gear:
            self.rpm_infractions += 1
            rpm_range = 180      # Range between two penalty points.
            if rpm >= upper_limit + rpm_range * 9:
                self.score += 9
            else:
                i = 1
                while i < 10:
                    if rpm <= upper_limit + rpm_range * i:
                        self.score += i
                        break
                    i += 1

    def _validate_throttle_infraction(self, throttle):
        """Validates whether the throttle value is over the upper limit.
        :param throttle: Throttle value.
        :return: Void.
        """
        upper_limit = 0.6
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
        """Validates whether the brake value is over the upper limit.
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

    def _validate_engine_idle_infraction(self, wheelspeed, running, time):
        """Validates whether the engine is in idle mode when the speed is 0 after one second.
        :param wheelspeed: Wheel speed of the car.
        :param running: {@code True} if the engine of the car is running, else {@code False}.
        :return: Void.
        """
        velocity = wheelspeed * 3.6
        if velocity < 0.1:
            if running == 1 and time - self.prev_time >= 1:
                self.prev_time = time
                self.engine_idle_infractions += 1
                self.score += 4
        else:
            self.prev_time = time

    def _validate_accelerate_and_stop_infraction(self, throttle, brake):
        """Validates for accelerate-and-stop behaviour of cars. Uses the local cache of this class.
        :param throttle: Throttle value.
        :param brake: Brake value.
        :return: Void.
        """
        lower_limit = 0.1       # Lower limit for brake and throttle to consider as harmful for fuel-efficiency
        last_x_actions = 4      # The last x elements from the cache are considered for the penalty score
        score_limit = 8         # The maximum number of penalty points
        new_action = None       # Newest performed action
        new_value = None        # Value of the newest performed action
        values = self.cache.get("values")
        if throttle >= lower_limit:
            new_action = "throttle"
            new_value = throttle
        elif brake >= lower_limit:
            new_action = "brake"
            new_value = brake
        if new_action is None:
            # Car kept constant speed for at least one second. Empties cache.
            self.cache["last_actions"] = list()
            self.cache["values"] = list()
        elif len(self.cache.get("last_actions")) == 0 or self.cache.get("last_actions")[-1] == new_action:
            self.cache["last_actions"].append(new_action)
            self.cache["values"].append(new_value)
        else:
            # Last action =/= new action
            self.accelerate_and_stop_infractions += 1
            infraction_score = 0
            i = 0
            while i < len(values):
                if i == last_x_actions:
                    break
                infraction_score += values[-(i+1)]      # Add the last x values of the cache
                i += 1
            infraction_score /= last_x_actions          # Create average
            infraction_score *= 10                      # Values are between 0-1
            infraction_score = round(infraction_score)
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
        self.fuel = value
        self.consumed_fuel = 1 - value

    def validate_oracles(self, rpm, gear, throttle, brake, wheelspeed, running, fuel, time):
        """Validates the fuel-specific oracles and calculates the score.
        :param time: Simulation time.
        :param rpm: RPM (rounds per minute) value.
        :param gear: Current gear of the car.
        :param throttle: Throttle value between 0-1.
        :param brake: Brake value between 0-1.
        :param wheelspeed: Wheelspeed value.
        :param running: {@code True} if the engine is running, else {@code False}.
        :param fuel: Fuel value between 0-1.
        :param time: Simulation time.
        :return: Void.
        """
        if time - self.prev_time_called >= 1:
            self._validate_accelerate_and_stop_infraction(throttle, brake)
            self._validate_brake_infraction(brake)
            self._validate_engine_idle_infraction(wheelspeed, running, time)
            self._validate_rpm_infraction(rpm, gear)
            self._validate_throttle_infraction(throttle)
            self.set_fuel_value(fuel)
            self.prev_time_called = time

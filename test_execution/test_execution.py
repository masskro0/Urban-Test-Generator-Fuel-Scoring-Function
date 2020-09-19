from math import floor

from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics, Timer, Damage
from termcolor import colored

from test_execution.scoring_oracle import ScoringOracle
from test_execution.test_oracle import TestOracle, TestCaseState, TrafficLightLabel
from test_execution.engine_types import EngineType
from utils.utility_functions import get_magnitude_of_3d_vector


def setup_test_case(converter):
    print(colored("Starting test case {}.".format(converter.scenario.name), "grey", attrs=['bold']))
    vehicles = converter.scenario.vehicles
    ego = None
    for vehicle in vehicles.keys():
        if vehicle.vid == "ego":
            ego = vehicle
            break
    assert ego is not None, "At least one vehicle must have vid \"ego\"."
    beamng = BeamNGpy('localhost', 64286)
    electrics = Electrics()
    ego.attach_sensor('electrics', electrics)
    timer = Timer()
    ego.attach_sensor("timer", timer)
    damage = Damage()
    ego.attach_sensor("damage", damage)
    return beamng, ego


def run_test_case(converter, dbc, dbe):
    beamng, ego = setup_test_case(converter)
    observer = ScoringOracle()
    oracle = TestOracle(converter.scenario, dbc, dbe)
    tllabel = TrafficLightLabel(converter.get_traffic_lights_position(), converter.traffic_triggers)
    bng = beamng.open()
    bng.load_scenario(converter.scenario)
    bng.start_scenario()
    while oracle.state == TestCaseState.OK:
        ego.update_vehicle()
        sensors = bng.poll_sensors(ego)
        electrics = sensors['electrics']['values']
        observer.validate_oracles(electrics['rpmTacho'], electrics['gear'], electrics['throttle'], electrics['brake'],
                                  electrics['wheelspeed'], electrics['running'], electrics['fuel'],
                                  sensors["timer"]["time"])
        label = tllabel.get_traffic_light_label(sensors["timer"]["time"], ego.state)
        oracle.validate_test_case([{"id": "ego", "state": ego.state}], ego.state, sensors["timer"]["time"], label,
                                  [{"id": "ego", "damage": sensors["damage"]["damage"]}])
    bng.close()
    return observer.get_results()


def run_test_case_role_model(converter, dbc, dbe, engine_type=EngineType.PETROL):
    beamng, ego = setup_test_case(converter)
    observer = ScoringOracle()
    oracle = TestOracle(converter.scenario, dbc, dbe)
    tllabel = TrafficLightLabel(converter.get_traffic_lights_position(), converter.traffic_triggers)
    spots = engine_type.get_rpm_shifting_sweetspots()
    upper_limit = spots.get("upper_limit") - 100
    lower_limit = spots.get("lower_limit")
    bng = beamng.open()
    bng.load_scenario(converter.scenario)
    bng.start_scenario()
    ego.set_shift_mode('realistic_manual_auto_clutch')
    prev_time = 0
    while oracle.state == TestCaseState.OK:
        ego.update_vehicle()
        sensors = bng.poll_sensors(ego)
        time = sensors["timer"]["time"]
        if floor(time) != prev_time:
            prev_time = floor(time)
            if sensors['electrics']['values']['rpm'] >= upper_limit and sensors['electrics']['values']['gear'] < 6:
                next_gear = int(sensors['electrics']['values']['gear']) + 1
                ego.control(gear=next_gear)
            elif sensors['electrics']['values']['gear'] > 1 and \
                    sensors['electrics']['values']['rpm'] < lower_limit:
                next_gear = int(sensors['electrics']['values']['gear']) - 1
                ego.control(gear=next_gear)
            elif get_magnitude_of_3d_vector(ego.state["vel"]) * 3.6 > 15:
                ego.control(gear=2)
        electrics = sensors['electrics']['values']
        observer.validate_oracles(electrics['rpmTacho'], electrics['gear'], electrics['throttle'], electrics['brake'],
                                  electrics['wheelspeed'], electrics['running'], electrics['fuel'],
                                  sensors["timer"]["time"])
        label = tllabel.get_traffic_light_label(sensors["timer"]["time"], ego.state)
        oracle.validate_test_case([{"id": "ego", "state": ego.state}], ego.state, time, label,
                                  [{"id": "ego", "damage": sensors["damage"]["damage"]}])
    bng.close()
    return observer.get_results()

from time import sleep
from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics, Timer, Damage
from termcolor import colored

from test_execution.scoring_oracle import MisbehaviourObserver
from test_execution.test_oracle import TestOracle, TestCaseState, TrafficLightLabel


def run_test_case(converter, dbc, dbe):
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
    observer = MisbehaviourObserver()
    oracle = TestOracle(converter.scenario, dbc, dbe)
    tllabel = TrafficLightLabel(converter.get_traffic_lights_position(), converter.traffic_triggers)
    bng = beamng.open()
    bng.load_scenario(converter.scenario)
    bng.start_scenario()
    while oracle.state == TestCaseState.OK:
        ego.update_vehicle()
        sensors = bng.poll_sensors(ego)
        observer.validate_infraction(sensors, sensors["timer"]["time"])
        label = tllabel.get_traffic_light_label(sensors["timer"]["time"], ego.state)
        oracle.validate_test_case([{"id": "ego", "state": ego.state}], ego.state, sensors["timer"]["time"], label,
                                  [{"id": "ego", "damage": sensors["damage"]["damage"]}])
    bng.close()
    return observer.get_results()

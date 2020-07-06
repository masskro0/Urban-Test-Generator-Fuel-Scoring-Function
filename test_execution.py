from time import sleep
from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics
from termcolor import colored

from verifier import MisbehaviourObserver


def run_test_case(scenario, success_point, line):
    print(colored("Starting test case {}.".format(scenario.name), "grey", attrs=['bold']))
    vehicles = scenario.vehicles
    ego = None
    for vehicle in vehicles.keys():
        if vehicle.vid == "ego":
            ego = vehicle
            break
    assert ego is not None, "At least one vehicle must have vid \"ego\"."
    beamng = BeamNGpy('localhost', 64286)
    electrics = Electrics()
    ego.attach_sensor('electrics', electrics)
    bng = beamng.open()
    bng.load_scenario(scenario)
    sleep(2)
    bng.start_scenario()
    print(line)
    ego.ai_set_line(line)
    #ego.ai_set_waypoint(success_point)
    observer = MisbehaviourObserver()
    for _ in range(2000):
        # vehicle.update_vehicle()
        # sensors = bng.poll_sensors(vehicle)
        # observer.check_misbehavior(sensors, vehicle.state)
        sleep(1)

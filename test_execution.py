from time import sleep
from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics
from termcolor import colored

from verifier import MisbehaviourObserver


def run_test_case(scenario, lines):
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
    sleep(1)
    bng.start_scenario()
    observer = MisbehaviourObserver()
    for _ in range(2000):
        # vehicle.update_vehicle()
        # sensors = bng.poll_sensors(vehicle)
        # observer.check_misbehavior(sensors, vehicle.state)
        sleep(1)

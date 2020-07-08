from time import sleep
from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics
from termcolor import colored
from scipy.spatial.distance import euclidean

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
    for line in lines:
        ego.ai_set_line(line)
        while True:
            ego.update_vehicle()
            pos = ego.state.get("pos")
            if euclidean((pos[0], pos[1]), (line[-1].get("pos")[0], line[-1].get("pos")[1])) < 1:
                sleep(2)
                break
    observer = MisbehaviourObserver()
    for _ in range(2000):
        # vehicle.update_vehicle()
        # sensors = bng.poll_sensors(vehicle)
        # observer.check_misbehavior(sensors, vehicle.state)
        sleep(1)

import time
from pathlib import Path
from beamngpy import BeamNGpy, Vehicle, Scenario
from beamngpy.sensors import Electrics

from observer import MisbehaviourObserver

beamng = BeamNGpy('localhost', 64256)
vehicle = Vehicle('ego', model='etk800', licence='PYTHON', colour='Green')
electrics = Electrics()
vehicle.attach_sensor('electrics', electrics)

#scenario = Scenario('urban', 'drivebuild_45')
#scenario.add_vehicle(vehicle, pos=(0,0,0), rot=(0, 0, 0))
#scenario.path = Path("D:\\Program Files (x86)\\BeamNG\\levels\\urban\\scenarios")
scenario = Scenario('urban', 'example')
scenario.add_vehicle(vehicle, pos=(-0, 0, 0), rot=(0, 0, 0))
scenario.make(beamng)

bng = beamng.open()
bng.load_scenario(scenario)
bng.start_scenario()
vehicle.ai_set_mode('span')
observer = MisbehaviourObserver()
for _ in range(2000):
    vehicle.update_vehicle()
    sensors = bng.poll_sensors(vehicle)
    observer.check_misbehavior(sensors, vehicle.state)
    time.sleep(1)

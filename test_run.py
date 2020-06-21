import time
from pathlib import Path
from beamngpy import BeamNGpy, Vehicle, Scenario
from beamngpy.sensors import Electrics
from beamngpy.beamngcommon import ENV
from os.path import join

from observer import MisbehaviourObserver

beamng = BeamNGpy('localhost', 64256)
vehicle = Vehicle('ego', model='etk800', licence='PYTHON', colour='Green')
electrics = Electrics()
vehicle.attach_sensor('electrics', electrics)

scenario = Scenario('urban', 'urban_26')
scenario.path = Path(join(ENV["BNG_HOME"], "levels", "urban", "scenarios"))

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

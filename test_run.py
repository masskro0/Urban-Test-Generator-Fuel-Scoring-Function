import time
from pathlib import Path

import numpy as np

from beamngpy import BeamNGpy, Vehicle, Scenario
from beamngpy.sensors import Electrics

from observer import MisbehaviourObserver

beamng = BeamNGpy('localhost', 64256)
vehicle = Vehicle('ego', model='etk800', licence='PYTHON', colour='Green')

electrics = Electrics()
vehicle.attach_sensor('electrics', electrics)

#scenario = Scenario('drivebuild', 'drivebuild_710')
#scenario.add_vehicle(vehicle, pos=(0,0,0), rot=(0, 0, 0))
#scenario.path = Path("D:\\Program Files (x86)\\BeamNG\\levels\\drivebuild\\scenarios")
scenario = Scenario('west_coast_usa', 'example')
vehicle = Vehicle('ego_vehicle', model='etk800', licence='PYTHON')
scenario.add_vehicle(vehicle, pos=(-717, 101, 118), rot=(0, 0, 45))
scenario.make(beamng)
bng = beamng.open()

observer = MisbehaviourObserver(bng, vehicle)

bng.load_scenario(scenario)
bng.start_scenario()
vehicle.ai_set_mode('span')
#vehicle.ai_set_speed(13.8)      # Meter pro Sekunde!!!!!

# Manual gear control
vehicle.set_shift_mode('realistic_manual_auto_clutch')
vehicle.ai_set_aggression(0.3)
# vehicle.update_vehicle()
# sensors = bng.poll_sensors(vehicle)
for _ in range(2000):
    observer.check_misbehavior()
    #vehicle.update_vehicle()
    #sensors = bng.poll_sensors(vehicle)
    """
    vehicle.control(throttle=0.2)
    print(sensors['electrics']['values']['gear'])
    print(sensors['electrics']['values']['rpm'])
    if sensors['electrics']['values']['rpm'] > 2800 and sensors['electrics']['values']['gear'] < 6:
        next_gear = int(sensors['electrics']['values']['gear']) + 1
        vehicle.control(gear=next_gear)
    elif sensors['electrics']['values']['gear'] > 1 and \
            sensors['electrics']['values']['rpm'] < 1500:
        next_gear = int(sensors['electrics']['values']['gear']) - 1
        vehicle.control(gear=next_gear)
    elif np.linalg.norm(vehicle.state["vel"]) > 20:
        vehicle.control(gear=2)
    """
    time.sleep(0.1)


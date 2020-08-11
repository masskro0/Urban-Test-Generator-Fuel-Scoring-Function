from time import sleep, time
from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics, Camera
from termcolor import colored
from os.path import join, abspath, dirname
from scipy.spatial.distance import euclidean

from fuel_consumption_test_generator import FuelConsumptionTestGenerator
from xml_converter.xml_to_bng_files import convert_test


def create_tests_and_run():
    gen = FuelConsumptionTestGenerator()
    gen.POPULATION_SIZE = 5

    for paths in gen.get_test():
        dbe = paths[0]
        dbc = paths[1]
        scenario, success_point = convert_test(dbc, dbe)
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
        direction = (0, 1, 0)
        fov = 90
        resolution = (1280, 720)
        x, y, z = -0.3, 2.1, 1
        camera = Camera((x, y, z), direction, fov, resolution, colour=True, depth=True, annotation=True)
        ego.attach_sensor("camera", camera)
        bng = beamng.open()
        bng.load_scenario(scenario)
        sleep(1)
        bng.start_scenario()
        imagedir = join(dirname(abspath(__file__)), "images")
        while True:
            ego.update_vehicle()
            sensors = bng.poll_sensors(ego)
            img = sensors["camera"]["colour"].convert("RGB")
            filename = 'green_{}.png'.format(time())
            filepath = join(imagedir, filename)
            img.save(filepath)
            sleep(0.5)
            if euclidean(success_point, (ego.state["pos"][0], ego.state["pos"][1])) < 15:
                bng.close()
                break


if __name__ == '__main__':
    create_tests_and_run()

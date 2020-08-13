from time import sleep, time
from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics, Camera
from termcolor import colored
from os.path import join, exists
from os import mkdir
from scipy.spatial.distance import euclidean

from utils.utility_functions import get_angle
from fuel_consumption_test_generator import FuelConsumptionTestGenerator
from xml_converter.xml_to_bng_files import convert_test


def create_tests_and_run():
    gen = FuelConsumptionTestGenerator()
    gen.POPULATION_SIZE = 1

    for paths in gen.get_test():
        dbe = paths[0]
        dbc = paths[1]
        converter = convert_test(dbc, dbe)
        traffic_light_pos = converter.get_traffic_lights_position()
        if len(traffic_light_pos) == 0:
            continue
        destination_path = "test_case_" + str(time())
        if not exists(destination_path):
            mkdir(destination_path)
        imagedir = destination_path + "\\images"
        if not exists(imagedir):
            mkdir(imagedir)
        if not exists(destination_path + "\\test_case"):
            mkdir(destination_path + "\\test_case")
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
        direction = (0, 1, 0)
        fov = 90
        resolution = (1280, 720)
        x, y, z = -0.3, 2.1, 1
        camera = Camera((x, y, z), direction, fov, resolution, colour=True, depth=True, annotation=True)
        ego.attach_sensor("camera", camera)
        bng = beamng.open()
        bng.load_scenario(converter.scenario)
        sleep(1)
        bng.start_scenario()
        traffic_index = 0
        traffic_pos = None if len(traffic_light_pos) == 0 \
            else (float(traffic_light_pos[traffic_index]["x"]), float(traffic_light_pos[traffic_index]["y"]))
        while True:
            ego.update_vehicle()
            sensors = bng.poll_sensors(ego)
            if traffic_pos is not None:
                p0 = (ego.state["pos"][0] + ego.state["dir"][0], ego.state["pos"][1] + ego.state["dir"][1])
                p1 = (ego.state["pos"][0], ego.state["pos"][1])
                angle = get_angle(traffic_pos, p1, p0)
                distance = euclidean(traffic_pos, (ego.state["pos"][0], ego.state["pos"][1]))
                if distance <= 7:
                    traffic_index += 1
                    if traffic_index >= len(traffic_light_pos):
                        traffic_pos = None
                    else:
                        traffic_pos = (float(traffic_light_pos[traffic_index]["x"]),
                                       float(traffic_light_pos[traffic_index]["y"]))
                elif distance <= 60 and (0 <= angle <= fov/2 or 360 - fov/2 <= angle <= 360):
                    img = sensors["camera"]["colour"].convert("RGB")
                    filename = 'green_{}.png'.format(time())
                    filepath = join(imagedir, filename)
                    img.save(filepath)
            sleep(0.5)
            if euclidean(converter.success_point, (ego.state["pos"][0], ego.state["pos"][1])) < 15:
                bng.close()
                break


if __name__ == '__main__':
    create_tests_and_run()

from time import sleep, time
from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics, Camera, Timer
from termcolor import colored
from os.path import join, exists, abspath
from os import mkdir, getcwd
from shutil import move
from scipy.spatial.distance import euclidean
from glob import glob
from math import floor

from utils.utility_functions import get_angle
from fuel_consumption_test_generator import FuelConsumptionTestGenerator
from xml_converter.xml_to_bng_files import convert_test


def create_tests(num_tests, collect):
    gen = FuelConsumptionTestGenerator()
    gen.POPULATION_SIZE = num_tests
    for paths in gen.get_test():
        dbe = paths[0]
        dbc = paths[1]
        destination_path = "test_case_" + str(time())
        if not exists(destination_path):
            mkdir(destination_path)
        image_dir = destination_path + "\\images"
        if not exists(image_dir):
            mkdir(image_dir)
        test_case_dir = destination_path + "\\test_case"
        if not exists(test_case_dir):
            mkdir(test_case_dir)
        move(str(dbe), join(getcwd(), test_case_dir))
        move(str(dbc), join(getcwd(), test_case_dir))
        if collect:
            collect_images(destination_path)


def collect_images(destination_path):
    test_case_dir = join(destination_path, "test_case")
    image_dir = join(destination_path, "images")
    if not exists(image_dir):
        mkdir(image_dir)
    matches = glob(test_case_dir + "\\*")
    converter = convert_test(matches[0], matches[1])
    traffic_light_list = converter.get_traffic_lights_position()
    if len(traffic_light_list) == 0:
        print(colored("Test case {} has not traffic lights. Skipping...".format(converter.scenario.name),
                      "grey", attrs=['bold']))
        return

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
    direction = (0, 1, 0)
    fov = 90
    resolution = (1280, 720)
    x, y, z = -0.3, 2.1, 1
    camera = Camera((x, y, z), direction, fov, resolution, colour=True, depth=True, annotation=True)
    ego.attach_sensor("camera", camera)
    bng = beamng.open()
    bng.load_scenario(converter.scenario)
    bng.start_scenario()
    traffic_triggers = converter.traffic_triggers
    trigger_index = 0
    traffic_triggers_pos = (float(traffic_triggers[trigger_index]["x"]), float(traffic_triggers[trigger_index]["y"]))
    traffic_index = 0
    traffic_light_pos = (float(traffic_light_list[traffic_index]["x"]), float(traffic_light_list[traffic_index]["y"]))
    entered = False
    red_entered = False
    time_entry = 0
    prev_time = 0
    label = None
    init_state = traffic_triggers[trigger_index].get("initState")
    tolerance = float(traffic_triggers[trigger_index].get("tolerance"))
    multiplicator = 15 if init_state == "green" else 9
    while True:
        ego.update_vehicle()
        sensors = bng.poll_sensors(ego)
        if traffic_light_pos is not None and traffic_triggers_pos is not None:
            timer = sensors["timer"]["time"]
            p0 = (ego.state["pos"][0] + ego.state["dir"][0], ego.state["pos"][1] + ego.state["dir"][1])
            p1 = (ego.state["pos"][0], ego.state["pos"][1])
            angle = get_angle(traffic_light_pos, p1, p0)
            distance_light = euclidean(traffic_light_pos, (ego.state["pos"][0], ego.state["pos"][1]))
            if distance_light <= 7:
                traffic_index += 1
                if traffic_index >= len(traffic_light_list):
                    traffic_light_pos = None
                else:
                    traffic_light_pos = (float(traffic_light_list[traffic_index]["x"]),
                                   float(traffic_light_list[traffic_index]["y"]))
            elif distance_light <= 60 and (0 <= angle <= fov / 2 or 360 - fov / 2 <= angle <= 360):
                if traffic_light_list[traffic_index].get("mode") == "off":
                    label = "off"
                elif traffic_light_list[traffic_index].get("mode") == "blinking":
                    if floor(timer) % 2 == 0:
                        label = "off"
                    else:
                        label = "yellow"
                else:
                    distance_trigger = euclidean(traffic_triggers_pos, (ego.state["pos"][0], ego.state["pos"][1]))
                    if distance_trigger > tolerance * multiplicator:
                        label = init_state
                    else:
                        if init_state == "red":
                            if not entered:
                                label = "yellow_red"
                                entered = True
                                prev_time = timer
                            else:
                                time_entry += timer - prev_time
                                prev_time = timer
                                if time_entry >= 1.45:
                                    label = "green"
                                    prev_time = 0
                                    time_entry = 0
                                    trigger_index += 1
                                    entered = False
                                    traffic_triggers_pos = None if trigger_index >= len(traffic_triggers) else \
                                        (float(traffic_triggers[trigger_index]["x"]),
                                         float(traffic_triggers[trigger_index]["y"]))
                                    init_state = traffic_triggers[trigger_index].get("initState")
                                    tolerance = float(traffic_triggers[trigger_index].get("tolerance"))
                                    multiplicator = 15 if init_state == "green" else 9
                        else:
                            if distance_trigger <= 9 and not red_entered:
                                label = "red"
                                red_entered = True
                                time_entry += timer - prev_time
                                prev_time = timer
                            else:
                                if not entered:
                                    label = "yellow"
                                    entered = True
                                    prev_time = timer
                                else:
                                    time_entry += timer - prev_time
                                    prev_time = timer
                                    if time_entry >= 8:
                                        label = "green"
                                        prev_time = 0
                                        time_entry = 0
                                        trigger_index += 1
                                        entered = False
                                        red_entered = False
                                        traffic_triggers_pos = None if trigger_index >= len(traffic_triggers) else \
                                            (float(traffic_triggers[trigger_index]["x"]),
                                             float(traffic_triggers[trigger_index]["y"]))
                                        init_state = traffic_triggers[trigger_index].get("initState")
                                        tolerance = float(traffic_triggers[trigger_index].get("tolerance"))
                                        multiplicator = 15 if init_state == "green" else 9
                                    elif time_entry >= 7:
                                        label = "yellow_red"
                img = sensors["camera"]["colour"].convert("RGB")
                filename = label + '_{}.png'.format(time())
                file_path = join(image_dir, filename)
                img.save(file_path)
            elif entered:
                prev_time = 0
                time_entry = 0
                trigger_index += 1
                entered = False
                red_entered = False
                traffic_triggers_pos = None if trigger_index >= len(traffic_triggers) else \
                    (float(traffic_triggers[trigger_index]["x"]),
                     float(traffic_triggers[trigger_index]["y"]))
                init_state = None if trigger_index >= len(traffic_triggers) else \
                    traffic_triggers[trigger_index].get("initState")
                tolerance = None if trigger_index >= len(traffic_triggers) else \
                    float(traffic_triggers[trigger_index].get("tolerance"))
                multiplicator = 15 if init_state is not None and init_state == "green" else 9
        sleep(0.4)
        if euclidean(converter.success_point, (ego.state["pos"][0], ego.state["pos"][1])) < 15:
            bng.close()
            break


def collect_images_existing_tests():
    folders = glob("test_case_*")
    for folder in folders:
        collect_images(abspath(folder))


if __name__ == '__main__':
    collect_images_existing_tests()

# TODO Visualize results

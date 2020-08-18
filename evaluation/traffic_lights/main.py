from time import time, sleep
from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics, Camera, Timer, Damage
from termcolor import colored
from os.path import join, exists, abspath
from os import mkdir, getcwd
from shutil import move
from scipy.spatial.distance import euclidean
from glob import glob
import matplotlib.pyplot as plt
from numpy import arange

from test_execution.test_oracle import TrafficLightLabel, TestOracle, TestCaseState
from utils.utility_functions import get_angle
from fuel_consumption_test_generator import FuelConsumptionTestGenerator
from xml_converter.xml_to_bng_files import convert_test


def create_tests(num_tests, collect):
    gen = FuelConsumptionTestGenerator()
    gen.POPULATION_SIZE = num_tests
    gen.traffic = False
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
    damage = Damage()
    ego.attach_sensor("damage", damage)
    direction = (0, 1, 0)
    fov = 90
    resolution = (1280, 720)
    x, y, z = -0.3, 2.1, 1
    camera = Camera((x, y, z), direction, fov, resolution, colour=True, depth=True, annotation=True)
    ego.attach_sensor("camera", camera)
    print(colored("Starting test case \"{}\".".format(destination_path), "grey", attrs=['bold']))
    traffic_index = 0
    traffic_light_pos = (float(traffic_light_list[traffic_index]["x"]),
                         float(traffic_light_list[traffic_index]["y"]))
    at_intersection = False
    tllabel = TrafficLightLabel(converter.get_traffic_lights_position(), converter.traffic_triggers)
    oracle = TestOracle(converter.scenario, matches[0], matches[1])
    bng = beamng.open()
    bng.load_scenario(converter.scenario)
    bng.start_scenario()
    sleep(2)
    while oracle.state == TestCaseState.OK:
        sensors = bng.poll_sensors(ego)
        ego.update_vehicle()
        label = tllabel.get_traffic_light_label(sensors["timer"]["time"], ego.state)
        if traffic_light_pos is not None:
            p1 = (ego.state["pos"][0], ego.state["pos"][1])
            distance_light = euclidean(traffic_light_pos, p1)
            if distance_light <= 10:
                at_intersection = True
            elif distance_light > 10 and at_intersection:
                at_intersection = False
                traffic_index += 1
                if traffic_index >= len(traffic_light_list):
                    traffic_light_pos = None
                else:
                    traffic_light_pos = (float(traffic_light_list[traffic_index]["x"]),
                                         float(traffic_light_list[traffic_index]["y"]))
                continue
            p0 = (ego.state["pos"][0] + ego.state["dir"][0], ego.state["pos"][1] + ego.state["dir"][1])
            angle = get_angle(traffic_light_pos, p1, p0)
            if 8 <= distance_light <= 60 and (0 <= angle <= fov / 2 or 360 - fov / 2 <= angle <= 360) \
                    and label is not None:
                img = sensors["camera"]["colour"].convert("RGB")
                filename = label + '_{}.png'.format(time())
                file_path = join(image_dir, filename)
                img.save(file_path)
        oracle.validate_test_case([{"id": "ego", "state": ego.state}], ego.state, sensors["timer"]["time"], label,
                                  [{"id": "ego", "damage": sensors["damage"]["damage"]}])
    bng.close()


def collect_images_existing_tests():
    folders = glob("test_case_*")
    for folder in folders:
        collect_images(abspath(folder))


def predict_all_images():
    from evaluation.traffic_lights.detector.main import main as predict, init_function
    yolo, config = init_function()
    correct = false = 0
    green_false = yellow_false = red_false = yellow_red_false = off_false = 0
    green_images = yellow_images = red_images = yellow_red_images = off_images = 0

    folders = glob("test_case_*")
    for folder in folders:
        image_folder = join(folder, "images", "*")
        images = glob(image_folder)
        for image in images:
            prediction = predict(abspath(image), config, yolo.model)
            ground_truth = image.split("\\")[-1].split("_")[0]
            if prediction == ground_truth:
                correct += 1
            else:
                false += 1
            if ground_truth == "green":
                green_images += 1
                if prediction != ground_truth:
                    green_false += 1
            elif ground_truth == "yellow":
                yellow_images += 1
                if prediction != ground_truth:
                    yellow_false += 1
            elif ground_truth == "red":
                red_images += 1
                if prediction != ground_truth:
                    red_false += 1
            elif ground_truth == "yellow-red":
                yellow_red_images += 1
                if prediction != ground_truth:
                    yellow_red_false += 1
            elif ground_truth == "off":
                off_images += 1
                if prediction != ground_truth:
                    off_false += 1
    print(colored("\nCorrect predictions: {}".format(correct), "green", attrs=['bold']))
    print(colored("False predictions: {}\n".format(false), "red", attrs=['bold']))
    print(colored("False green lights predictions: {}".format(green_false), "red", attrs=['bold']))
    print(colored("False yellow lights predictions: {}".format(yellow_false), "red", attrs=['bold']))
    print(colored("False red lights predictions: {}".format(red_false), "red", attrs=['bold']))
    print(colored("False yellow-red lights predictions: {}".format(yellow_red_false), "red", attrs=['bold']))
    print(colored("False disabled traffic lights predictions: {}\n".format(off_false), "red", attrs=['bold']))
    print(colored("Images with green traffic lights: {}".format(green_images), "grey", attrs=['bold']))
    print(colored("Images with yellow traffic lights: {}".format(yellow_images), "grey", attrs=['bold']))
    print(colored("Images with red traffic lights: {}".format(red_images), "grey", attrs=['bold']))
    print(colored("Images with yellow-red traffic lights: {}".format(yellow_red_images), "grey", attrs=['bold']))
    print(colored("Images with disabled traffic lights: {}".format(off_images), "grey", attrs=['bold']))

    predictions = [correct, false]
    false_predictions = [green_false, yellow_false, red_false, yellow_red_false, off_false]
    images = [green_images, yellow_images, red_images, yellow_red_images, off_images]
    visualize_results(predictions, false_predictions, images)


def visualize_results(predictions, false_predictions, images):
    if not exists("result_pictures"):
        mkdir("result_pictures")
    possibilities = ["Correct predictions", "False predictions"]
    y_pos = arange(len(possibilities))
    barlist = plt.bar(y_pos, predictions, width=0.8)
    barlist[0].set_color("g")
    barlist[1].set_color("r")
    plt.xticks(y_pos, possibilities)
    ax = plt.gca()
    ax.set_ylabel('Number predictions')
    ax.set_title('Prediction of traffic light color')
    plt.savefig(join("result_pictures", 'predictions_bar.png'), bbox_inches='tight')
    plt.show()
    explode = (0, 0)
    fig1, ax1 = plt.subplots()
    ax1.pie(predictions, explode=explode, labels=possibilities, autopct='%1.1f%%',
            shadow=True, startangle=90)
    ax1.axis('equal')
    ax1.set_title('Prediction of traffic light color')
    plt.savefig(join("result_pictures", 'predictions_pie.png'), bbox_inches='tight')
    plt.show()

    possibilities = ["Green", "Yellow", "Red", "Yellow-Red", "Off"]
    y_pos = arange(len(possibilities))
    plt.bar(y_pos, false_predictions, width=0.4)
    plt.xticks(y_pos, possibilities)
    ax = plt.gca()
    ax.set_ylabel('Number false predictions')
    ax.set_title('Number of false predictions for each traffic light color')
    plt.savefig(join("result_pictures", 'number_false_predictions_bar.png'), bbox_inches='tight')
    plt.show()
    explode = (0, 0, 0, 0, 0)
    fig1, ax1 = plt.subplots()
    ax1.pie(false_predictions, explode=explode, labels=possibilities, autopct='%1.1f%%',
            shadow=True, startangle=90)
    ax1.axis('equal')
    ax1.set_title('Number of false predictions for each traffic light color')
    plt.savefig(join("result_pictures", 'number_false_predictions_pie.png'), bbox_inches='tight')
    plt.show()

    possibilities = ["Green", "Yellow", "Red", "Yellow-Red", "Off"]
    y_pos = arange(len(possibilities))
    plt.bar(y_pos, images, width=0.4)
    plt.xticks(y_pos, possibilities)
    ax = plt.gca()
    ax.set_ylabel('Number of images')
    ax.set_title('Number of images for each traffic light color')
    plt.savefig(join("result_pictures", 'number_images_bar.png'), bbox_inches='tight')
    plt.show()
    explode = (0, 0, 0, 0, 0)
    fig1, ax1 = plt.subplots()
    ax1.pie(images, explode=explode, labels=possibilities, autopct='%1.1f%%',
            shadow=True, startangle=90)
    ax1.axis('equal')
    ax1.set_title('Number of images for each traffic light color')
    plt.savefig(join("result_pictures", 'number_images_pie.png'), bbox_inches='tight')
    plt.show()


if __name__ == '__main__':
    # create_tests(1, True)
    collect_images_existing_tests()
    predict_all_images()


from time import time, sleep
from beamngpy.sensors import Camera
from termcolor import colored
from os.path import join, exists, abspath
from os import mkdir, getcwd
from shutil import move
from scipy.spatial.distance import euclidean
from glob import glob
import matplotlib.pyplot as plt
from numpy import arange, array
from sklearn.metrics import confusion_matrix
from seaborn import heatmap
from numpy import trace, asarray
from numpy import sum as npsum
from copy import deepcopy

from test_execution.test_oracle import TrafficLightLabel, TestOracle, TestCaseState
from test_execution.test_execution import setup_test_case
from utils.utility_functions import get_angle
from utils.plotter import plot_road_traffic_light
from fuel_consumption_test_generator import FuelConsumptionTestGenerator
from xml_converter.xml_to_bng_files import convert_test
from evaluation.traffic_lights.detector.main import main as predict, init_function


def create_tests(num_tests, collect, experiments):
    """Generates new test cases and creates an images and a test case folder. XML files are moved to test case folder.
    :param num_tests: Number of tests to generate.
    :param collect: {@code True} if images should be collected for each generated test case.
    :param experiments: List of experiment names.
    :return: Void.
    """
    gen = FuelConsumptionTestGenerator()
    gen.POPULATION_SIZE = num_tests
    gen.traffic = False
    for paths in gen.get_test():
        dbe = paths[0]
        dbc = paths[1]
        temp = deepcopy(experiments)
        temp.append("default")
        for exp in temp:
            destination_path = join(exp, "test_case_" + str(time()))
            if not exists(destination_path):
                mkdir(destination_path)
            test_case_dir = join(destination_path, "\\test_case")
            if not exists(test_case_dir):
                mkdir(test_case_dir)
            if exp != "default":
                image_dir = join(destination_path, "images")
                if not exists(image_dir):
                    mkdir(image_dir)
            move(str(dbe), join(getcwd(), test_case_dir))
            move(str(dbc), join(getcwd(), test_case_dir))
            if collect:
                collect_images(destination_path)


def collect_images(destination_path, min_distance=None, max_distance=None):
    """Runs test case in simulator and stores images in the images folder of the concurrent test case.
    :param destination_path: Path to the test case folder (not to the XML folder)
    :param min_distance: Minimum distance to traffic light to allow collecting images.
    :param max_distance: Maximum distance to traffic lights to allow collecting images.
    :return: Void.
    """
    max_distance = 60 if max_distance is None else max_distance
    min_distance = 8 if min_distance is None else min_distance
    test_case_dir = join(destination_path, "test_case")
    image_dir = join(destination_path, "images")
    if not exists(image_dir):
        mkdir(image_dir)
    xml_files = glob(test_case_dir + "\\*")
    converter = convert_test(xml_files[0], xml_files[1])
    traffic_light_list = converter.get_traffic_lights_position()
    if len(traffic_light_list) == 0:
        print(colored("Test case {} has no traffic lights. Skipping...".format(converter.scenario.name),
                      "grey", attrs=['bold']))
        return
    beamng, ego = setup_test_case(converter)

    # Setup camera.
    direction = (0, 1, 0)
    fov = 90
    resolution = (1280, 720)
    x, y, z = -0.3, 2.1, 1
    camera = Camera((x, y, z), direction, fov, resolution, colour=True, depth=True, annotation=True)
    ego.attach_sensor("camera", camera)
    print(colored("Starting test case \"{}\".".format(destination_path), "grey", attrs=['bold']))
    traffic_index = 0

    # Get first traffic light position.
    traffic_light_pos = (float(traffic_light_list[traffic_index]["x"]),
                         float(traffic_light_list[traffic_index]["y"]))

    # True if the ego-car enters an intersection.
    at_intersection = False

    # Class to get traffic light state.
    tllabel = TrafficLightLabel(converter.get_traffic_lights_position(), converter.traffic_triggers)
    oracle = TestOracle(xml_files[0], xml_files[1])
    bng = beamng.open()
    bng.load_scenario(converter.scenario)
    if converter.weather is not None:
        bng.set_weather_preset(converter.weather)
    bng.start_scenario()
    sleep(2)
    while oracle.state == TestCaseState.OK:
        sensors = bng.poll_sensors(ego)
        ego.update_vehicle()
        label = tllabel.get_traffic_light_label(sensors["timer"]["time"], ego.state["pos"], ego.state["dir"])
        if traffic_light_pos is not None:
            p0 = (ego.state["pos"][0], ego.state["pos"][1])
            distance_light = euclidean(traffic_light_pos, p0)
            if distance_light <= 10:
                # Did ego-car entered an intersection?
                at_intersection = True
            elif distance_light > 10 and at_intersection:
                # Did ego-car leaves an intersection?
                at_intersection = False
                traffic_index += 1
                if traffic_index >= len(traffic_light_list):
                    traffic_light_pos = None
                else:
                    # Get next traffic light position.
                    traffic_light_pos = (float(traffic_light_list[traffic_index]["x"]),
                                         float(traffic_light_list[traffic_index]["y"]))
                continue
            # Endpoint of ego-car's directional vector.
            p1 = (ego.state["pos"][0] + ego.state["dir"][0], ego.state["pos"][1] + ego.state["dir"][1])

            # Angle between ego-car direction vector and traffic light.
            angle = get_angle(traffic_light_pos, p0, p1)
            print(distance_light, angle, label)
            if min_distance <= distance_light <= max_distance \
                    and (0 <= angle <= fov / 2 or 360 - fov / 2 <= angle <= 360) and label is not None:
                # Is distance between ego-car and traffic light within the allowed range, the traffic light in the
                # field-of-view of the camera and the traffic light in a defined state?
                # Get image of camera and save it with the traffic light state.
                img = sensors["camera"]["colour"].convert("RGB")
                filename = label + '_{}.png'.format(time())
                file_path = join(image_dir, filename)
                img.save(file_path)
        oracle.validate_test_case([{"id": "ego", "pos": ego.state["pos"]}], ego.state["pos"], ego.state["dir"],
                                  ego.state["vel"], sensors["timer"]["time"], label,
                                  [{"id": "ego", "damage": sensors["damage"]["damage"]}])
    bng.close()


def collect_images_existing_tests(test_cases, min_distance=None, max_distance=None):
    """Collects images of all test cases.
    :param test_cases: List of test cases.
    :param min_distance: Minimum distance to traffic light to take an image.
    :param max_distance: Maximum distance to traffic light to take an image.
    :return:
    """
    folders = glob(test_cases)
    for folder in folders:
        collect_images(abspath(folder), min_distance, max_distance)


def predict_all_images(experiment_folder):
    """Setup a traffic light detection system and compare predicted results with the ground truth labels.
    :param experiment_folder: Path to a experiment folder.
    :return: Void.
    """
    # Initialize model.
    yolo, config = init_function()
    correct = false = 0
    green_false = yellow_false = red_false = yellow_red_false = off_false = 0
    green_images = yellow_images = red_images = yellow_red_images = off_images = 0
    test_cases_status = list()

    # Maximum allowed wrong predictions before a test case fails.
    limit_false_predictions = 6

    # Get all test case folders of an experiment.
    folders = glob(experiment_folder)
    for folder in folders:
        test_cases = glob(join(folder, "test_case", "*"))
        road_network_dir = join(folder, "road_network")
        if not exists(road_network_dir):
            mkdir(road_network_dir)
        # Plot the road network.
        plot_road_traffic_light(test_cases[0], test_cases[1], show=False, save_path=road_network_dir)
        false_predictions = 0
        bbox_path = abspath(join(folder, "bounding_boxes"))
        if not exists(bbox_path):
            mkdir(bbox_path)
        image_folder = join(folder, "images", "*")
        images = glob(image_folder)
        for image in images:
            # Make a prediction for each image.
            if not image.endswith(".png") and not image.endswith(".jpg"):
                continue
            prediction = predict(abspath(image), config, yolo.model, bbox_path)
            ground_truth = image.split("\\")[-1].split("_")[0]

            # Compare the prediction with the ground truth label. Add to results.
            if prediction == ground_truth:
                correct += 1
            else:
                false += 1
                false_predictions += 1
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
        if false_predictions < limit_false_predictions:
            test_cases_status.append({"name": folder, "status": "success"})
        else:
            test_cases_status.append({"name": folder, "status": "fail"})
    # Print results.
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
    print(colored("Images with disabled traffic lights: {}\n".format(off_images), "grey", attrs=['bold']))
    for tc in test_cases_status:
        if tc.get("status") == "success":
            print(colored("Test case \"{}\" succeeded.".format(tc.get("name")), "green", attrs=['bold']))
        else:
            print(colored("Test case \"{}\" failed.".format(tc.get("name")), "red", attrs=['bold']))

    false_predictions = [green_false, yellow_false, red_false, yellow_red_false, off_false]
    images = [green_images, yellow_images, red_images, yellow_red_images, off_images]

    # Create plots.
    visualize_results(experiment_folder, false_predictions, images)


def write_text(ind, np_cp, np_fp):
    """Writes text inside the bars of a barplot.
    :param ind:
    :param np_cp: Numpy array of correct predictions.
    :param np_fp: Numpy array of false predicitons.
    :return: Void.
    """
    for xpos, ypos, yval in zip(ind, np_cp / 2, np_cp):
        if yval == 0:
            continue
        plt.text(xpos, ypos, "%.1f" % yval, ha="center", va="center")
    for xpos, ypos, yval in zip(ind, np_cp + np_fp / 2, np_fp):
        if yval == 0:
            continue
        plt.text(xpos, ypos, "%.1f" % yval, ha="center", va="center")


def visualize_results(experiment_folder, false_predictions, images):
    """Create plots of traffic light experiment.
    :param experiment_folder: Path to experiment folder.
    :param false_predictions: List containing false predictions per color.
    :param images: List containing the number of images per color.
    :return: Void.
    """
    test_case_folder = experiment_folder.split("\\")[0]
    plt.clf()
    if not exists(join(test_case_folder, "result_pictures")):
        mkdir(join(test_case_folder, "result_pictures"))
    correct_predictions = list()
    i = 0
    while i < len(images):
        # Create a list of correct predictions.
        correct_predictions.append(images[i] - false_predictions[i])
        i += 1
    np_cp = array(correct_predictions)
    np_fp = array(false_predictions)

    labels = ["Green", "Yellow", "Red", "Yellow-Red", "Off"]
    ind = arange(len(images))
    width = 0.4

    # Create stacked bar plot of correct and wrong predictions.
    plt.figure(figsize=(10, 5.3))
    plt.subplot(1, 2, 1)

    # Plot bars.
    p1 = plt.bar(ind, correct_predictions, width)
    p2 = plt.bar(ind, false_predictions, width, bottom=correct_predictions)

    # Write text into the bars.
    write_text(ind, np_cp, np_fp)
    for xpos, ypos, yval in zip(ind, np_cp + np_fp, np_cp + np_fp):
        plt.text(xpos, ypos, "N=%d" % yval, ha="center", va="bottom")
    for ax in p1:
        ax.set_color("g")
    for ax in p2:
        ax.set_color("r")
    plt.ylabel('Predictions')
    plt.title('Correct and False Predictions for each Traffic Light Color')
    plt.xticks(ind, labels)
    plt.legend((p1[0], p2[0]), ('Correct', 'False'))
    plt.tight_layout()

    # Create normalized stacked bar plot correct and wrong predictions.
    snum = np_cp + np_fp

    # Normalize values.
    cp_norm = np_cp / snum * 100
    fp_norm = np_fp / snum * 100
    plt.subplot(1, 2, 2)
    p1 = plt.bar(ind, cp_norm, width=width, label='Correct')
    p2 = plt.bar(ind, fp_norm, width=width, bottom=cp_norm, label='False')
    write_text(ind, cp_norm, fp_norm)
    for xpos, ypos, yval in zip(ind, cp_norm + fp_norm, snum):
        plt.text(xpos, ypos, "N=%d" % yval, ha="center", va="bottom")
    for ax in p1:
        ax.set_color("g")
    for ax in p2:
        ax.set_color("r")
    plt.ylabel('Relation in Percentage')
    plt.title('Relations between Correct and False Predictions')
    plt.xticks(ind, labels)
    plt.legend((p1[0], p2[0]), ('Correct', 'False'), loc="upper left", bbox_to_anchor=(0, 0.95), framealpha=0.9)
    plt.tight_layout()
    plt.subplots_adjust(top=0.88)
    plt.suptitle("Test case \"{}\"".format(test_case_folder), fontsize=14)
    plt.savefig(join(test_case_folder, "result_pictures", test_case_folder + '_predictions_per_light_bar.png'),
                dpi=200)


def plot_confusion_matrix(prediction, expectation, experiment_name, save_path):
    """Plots a confusion matrix of which test cases passed or failed.
    :param prediction: List of states per test case with the predicted outcome.
    :param expectation: List of states per test case with the expected outcome.
    :param experiment_name: Name of the experiment.
    :param save_path: Path to store the image.
    :return: Void.
    """
    labels = ['Success', 'Fail']
    cm = confusion_matrix(expectation, prediction, labels)
    accuracy = trace(cm) / float(npsum(cm))
    precision = cm[1, 1] / sum(cm[:, 1])
    recall = cm[1, 1] / sum(cm[1, :])
    f1_score = 2 * precision * recall / (precision + recall)
    stats_text = "Accuracy={:0.3f}\nPrecision={:0.3f}\nRecall={:0.3f}\nF1 Score={:0.3f}".format(
        accuracy, precision, recall, f1_score)
    group_counts = ["{0:0.0f}".format(value) for value in cm.flatten()]
    group_percentages = ["{0:.2%}".format(value) for value in cm.flatten() / npsum(cm)]
    box_labels = [f"{v1}\n{v2}" for v1, v2 in zip(group_counts, group_percentages)]
    box_labels = asarray(box_labels).reshape(2, 2)
    heatmap(cm, fmt="", annot=box_labels, xticklabels=["Success", "Fail"], yticklabels=["Success", "Fail"])
    plt.title("Confusion Matrix of TLDS in Test Case \"{}\"".format(experiment_name))
    plt.ylabel('True label')
    plt.xlabel('Predicted label')
    plt.gcf().subplots_adjust(bottom=0.28)
    plt.gcf().subplots_adjust(right=1.05)
    plt.figtext(0.49, 0.01, stats_text, ha="center", fontsize=12,
                bbox={"facecolor": "gray", "alpha": 0.5, "pad": 5})
    plt.savefig(join(save_path, "cm_" + experiment_name + ".png"), dpi=200)


if __name__ == '__main__':
    experiment_names = ["15m", "45m", "cloudy", "daylight", "nighttime", "random"]
    # create_tests(1, True, experiment_names)
    for experiment in experiment_names:
        if experiment != "random":
            continue
        collect_images_existing_tests(join(experiment, "test_case_*"))
        predict_all_images(join(experiment, "test_case_*"))

    """Example of how to generate a confusion matrix. You need to check manually, which test cases should have passed
     and which not. The actual results are printed in the predict_all_images() method."""
    # expected = ['Success', 'Success', 'Success', 'Success', 'Success', 'Fail', 'Success', 'Fail', 'Fail',
    #           'Fail', 'Success']
    # prediction = ['Success', 'Success', 'Fail', 'Fail', 'Fail', 'Fail', 'Fail', 'Fail', 'Fail', 'Fail', 'Fail']
    # experiment = "45m"
    # plot_confusion_matrix(prediction, expected, experiment, join(experiment, "result_pictures"))

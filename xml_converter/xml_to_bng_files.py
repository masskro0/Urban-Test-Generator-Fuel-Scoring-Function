"""This class converts xml files to beamng json and prefab files."""

from pathlib import Path
from glob import glob
from termcolor import colored
from os import getcwd, path, mkdir, environ
from os.path import exists
from shutil import move
from beamngpy.beamngcommon import ENV
import json


def get_next_test(files_name):
    """Returns the next test files.
    :param files_name: File name series.
    :return: dbc and dbe file paths in a list.
    """
    destination_path = Path(getcwd())
    destination_path = path.realpath(destination_path) + "\\scenario"
    xml_names = destination_path + "\\" + files_name + "*"
    matches = glob(xml_names)
    if len(matches) > 1:
        return [matches[0], matches[1]]


def init_scenario_folder():
    """Creates the urban and scenarios directory in the BeamNG trunk directory, if is doesn't exists.
    :return: Void.
    """
    assert environ.get("BNG_HOME") is not None, "Please set the BNG_HOME environment variable."
    scenario_folder = ENV['BNG_HOME']
    scenario_folder += "\\levels\\urban"
    if not exists(scenario_folder):
        mkdir(scenario_folder)
    scenario_folder += "\\scenarios"
    if not exists(scenario_folder):
        mkdir(scenario_folder)


def get_index():
    """Gets the unique index which reflects the number of executions.
    :return: Index (Integer).
    """
    if exists("xml_converter\\index.txt"):
        with open("xml_converter\\index.txt", "r") as text_file:
            index = int(text_file.read())
    else:
        with open("xml_converter\\index.txt", "w") as text_file:
            print("{}".format(0), file=text_file)
            index = 0
    return index


def update_index(index):
    """Increments the index in the text file.
    :param index: Old index (Integer).
    :return: Void.
    """
    assert exists("xml_converter\\index.txt"), "Missing index.txt file in the xml_converter directory."
    index += 1
    with open("xml_converter\\index.txt", "w") as text_file:
        print("{}".format(index), file=text_file)


def create_json_file(index, participants, multiple_prefabs=False, start_index=None):
    data = {'authors': "Michael Heine", "description": None, "difficulty": 0, "name": "urban_" + str(index)}
    data["prefabs"] = ["levels/urban/scenarios/{}.prefab".format(data["name"])]
    if multiple_prefabs:
        assert start_index is not None
        temp = index - 1
        while temp >= start_index:
            data["prefabs"].append("levels/urban/scenarios/urban_{}.prefab".format(temp))
            temp -= 1
    data["vehicles"] = {}
    for participant in participants:
        bool_value = True if participant.get("id") == "ego" else False
        data["vehicles"]["{}".format(participant.get("id"))] = {"playerUsable": bool_value, "startFocus": bool_value}

    data = [data]
    with open('{}.json'.format(data[0]["name"]), 'w') as outfile:
        json.dump(data, outfile, indent=4)


def get_participants(dbc):
    pass


def convert_test(dbc, dbe):
    """Converts the XML files into BeamNG files.
    :param dbc: Path to the criteria XML file.
    :param dbe: Path to the environment XML file.
    :return: Void.
    """
    print(colored("Converting XML files to BNG files. Moving to scenarios folder...", "grey"))
    init_scenario_folder()
    index = get_index()
    participants = [{"id": "ego"}, {"id": "other"}]
    # participants = get_participants(dbc)
    create_json_file(index, participants)
    matches = glob("urban_*")
    for match in matches:
        move(match, ENV['BNG_HOME'] + "\\levels\\urban\\scenarios")
    update_index(index)
    """
    # Change it to YOUR DriveBuild user path.
    destination_path = "C:\\BeamNG.research_userpath\\drivebuild_*"
    matches = glob(destination_path)
    if len(matches) > 0:
        latest_folder = max(matches, key=path.getmtime)
        latest_folder = latest_folder + "\\levels\\drivebuild\\scenarios\\*"
        matches = glob(latest_folder)
        if len(matches) != 0:
            latest_file = max(matches, key=path.getmtime)
            elements = latest_file.split("\\")
            filename = elements[-1].split(".")[0]
            destination_path = latest_folder[:-1] + filename + "*"
            matches = glob(destination_path)
            for match in matches:
                # Change it to YOUR DriveBuild scenario folder in the BNG trunk folder.
                move(match, "D:\\Program Files (x86)\\BeamNG\\levels\\drivebuild\\scenarios")
    """

"""This class converts xml files to beamng json and prefab files."""

from pathlib import Path
from glob import glob
from termcolor import colored
from os import getcwd, path, mkdir, environ
from os.path import exists, join
from shutil import move
from beamngpy.beamngcommon import ENV
from json import dump
import xml.etree.ElementTree as Etree

from test_execution import run_test_case
from xml_converter.converter import Converter


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


def create_json_file(index, participants, author, tod, multiple_prefabs=False, start_index=None):
    """Creates a basic JSON file. If desired, it can add multiple prefab files so you don't have to load all by one.
    :param index: Index of this execution/file.
    :param participants: Array containing dict type information about each participant.
    :param author: Author of the XML file.
    :param tod: Time of the day. Must be between 0 and 8.
    :param multiple_prefabs: Option to add multiple prefab files. Needs starting index.
    :param start_index: Index where prefab inclusion stops.
    :return: Void.
    """
    data = {'authors': author, "description": "Urban scenario", "difficulty": 0, "name": "urban_" + str(index)}
    data["prefabs"] = ["levels/urban/scenarios/{}.prefab".format(data["name"])]
    if multiple_prefabs:
        assert start_index is not None
        temp = index - 1
        while temp >= start_index:
            data["prefabs"].append("levels/urban/scenarios/urban_{}.prefab".format(temp))
            temp -= 1
    data["vehicles"] = {}
    for participant in participants:
        assert participant.get("id") is not None, "Please include an id for each participant. Check the example" \
                                                  " for reference."
        bool_value = True if participant.get("id") == "ego" else False
        data["vehicles"]["{}".format(participant.get("id"))] = {"playerUsable": bool_value, "startFocus": bool_value,
                                                                "player": bool_value}
    data["levelObjects"] = {"tod": {"time": tod, "dayLength": 120, "play": False}}

    data = [data]
    with open('{}.json'.format(data[0]["name"]), 'w') as outfile:
        dump(data, outfile, indent=4)


def create_lua_file():
    # Cars can stop at red light, needs waypoint.
    raise NotImplementedError()


def convert_test(dbc, dbe):
    """Converts the XML files into BeamNG files.
    :param dbc: Path to the criteria XML file.
    :param dbe: Path to the environment XML file.
    :return: Void.
    """
    print(colored("Converting XML files to BNG files...", "grey", attrs=['bold']))
    init_scenario_folder()
    index = get_index()
    dbc_root = Etree.parse(dbc).getroot()
    dbe_root = Etree.parse(dbe).getroot()
    author = dbe_root.findtext("author")
    tod = dbe_root.findtext("timeOfDay")
    assert 0 <= float(tod) <= 1, "Time of Day must be between 0 and 1."
    participants = []
    for participant in dbc_root.findall('participants/participant'):
        participants.append(participant.attrib)
    assert len(participants) > 0, "Please add participants to your test case. Check the example for reference."
    create_json_file(index, participants, author, tod)
    converter = Converter(dbc_root, dbe_root, index)
    converter.add_to_prefab()
    matches = glob("urban_*")
    print(colored("Moving files to folder {}...".format(join(ENV['BNG_HOME'], "levels", "urban", "scenarios")),
                  "grey", attrs=['bold']))
    for match in matches:
        move(join(getcwd(), match), join(ENV['BNG_HOME'], "levels", "urban", "scenarios", match))
    update_index(index)
    run_test_case(converter.scenario, converter.success_point)

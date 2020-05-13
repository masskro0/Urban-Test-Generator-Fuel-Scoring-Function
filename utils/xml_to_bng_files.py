"""This class converts xml files to beamng json and prefab files."""

from drivebuildclient.AIExchangeService import AIExchangeService
from pathlib import Path
from glob import glob
from subprocess import call
from termcolor import colored
from os import getcwd, path
from shutil import move


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


def add_prefab_files_to_json(json_path, start_index, end_index):
    """Add all generated Prefab files to the last JSON file and save it.
    :param json_path: Path to the JSON file.
    :param start_index: Start index of the generated files.
    :param end_index: End index of the generated files.
    :return: Void.
    """
    # TODO
    pass


def convert_test(dbc, dbe):
    """Starts a test in DriveBuild to convert xml files to prefab, json and lua files. Moves them automatically to
    the scenario folder in the BeamNG trunk folder.
    :return: Void.
    """
    service = AIExchangeService("localhost", 8383)
    service.run_tests("test", "test", Path(dbe), Path(dbc))
    print(colored("Converting XML files to BNG files. Moving to scenarios folder...", "blue"))

    # Close BeamNG after converting. If you don't close it, BeamNG will load.
    call("C:\\Windows\\System32\\taskkill.exe /f /im BeamNG.research.x64.exe", shell=True)

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

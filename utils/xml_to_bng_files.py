"""This class converts xml files to beamng json and prefab files."""

from drivebuildclient.AIExchangeService import AIExchangeService
from pathlib import Path
from glob import glob
from subprocess import call
from termcolor import colored
import os
import shutil


def get_next_test(files_name):
    """Returns the next test files.
    :param files_name: File name series.
    :return: dbc and dbe file paths in a list.
    """
    destination_path = Path(os.getcwd())
    destination_path = os.path.realpath(destination_path) + "\\scenario"
    xml_names = destination_path + "\\" + files_name + "*"
    matches = glob(xml_names)
    if len(matches) > 1:
        return [matches[0], matches[1]]


def convert_test(dbc, dbe):
    """Starts a test in DriveBuild to convert xml files to prefab, json and lua files.
    :return: Void.
    """
    # next_test = get_next_test(files_name)
    # dbc = next_test[0]
    # dbe = next_test[1]
    service = AIExchangeService("localhost", 8383)
    service.run_tests("test", "test", Path(dbe), Path(dbc))
    print(colored("Converting XML files to BNG files. Moving to scenarios folder...", "blue"))

    # Close BeamNG after converting. If you don't close it, BeamNG will load.
    call("C:\\Windows\\System32\\taskkill.exe /f /im BeamNG.research.x64.exe", shell=True)

    # Change it to YOUR DriveBuild user path.
    destination_path = "C:\\BeamNG.research_userpath\\drivebuild_*"
    matches = glob(destination_path)
    if len(matches) > 0:
        latest_folder = max(matches, key=os.path.getctime)
        latest_folder = latest_folder + "\\levels\\drivebuild\\scenarios\\*"
        matches = glob(latest_folder)
        latest_file = max(matches, key=os.path.getctime)
        elements = latest_file.split("\\")
        filename = elements[-1].split(".")[0]
        destination_path = latest_folder[:-1] + filename + "*"
        matches = glob(destination_path)
        for match in matches:
            # Change it to YOUR DriveBuild scenario folder in the BNG trunk folder.
            shutil.move(match, "D:\\Program Files (x86)\\BeamNG\\levels\\drivebuild\\scenarios")


from os import environ, mkdir
from os.path import exists
from beamngpy.beamngcommon import ENV
from shutil import move


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


def move_files_bng_folder():
    assert environ.get("BNG_HOME") is not None, "Please set the BNG_HOME environment variable."
    bng_folder = ENV['BNG_HOME']
    vehicle_folder = bng_folder + "\\vehicles"
    move("87Golf", vehicle_folder)
    levels_folder = bng_folder + "\\levels"
    move("urban", levels_folder)


if __name__ == '__main__':
    move_files_bng_folder()
    init_scenario_folder()
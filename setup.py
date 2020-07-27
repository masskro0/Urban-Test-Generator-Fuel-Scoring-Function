"""Module to setup the correct folders. Run this module first before you run anything else."""

from os import environ, mkdir
from os.path import exists, join
from beamngpy.beamngcommon import ENV
from shutil import move


def init_scenario_folder():
    """Creates the urban and scenarios directory in the BeamNG trunk directory, if is doesn't exists.
    :return: Void.
    """
    assert environ.get("BNG_HOME") is not None, "Please set the BNG_HOME environment variable."
    urban_folder = join(ENV['BNG_HOME'], "levels", "urban")
    if not exists(urban_folder):
        mkdir(urban_folder)
    scenario_folder = join(ENV['BNG_HOME'], "levels", "urban", "scenarios")
    if not exists(scenario_folder):
        mkdir(scenario_folder)


def move_files_to_bng_folder():
    """Moves files from this directory to the bng trunk directory.
    :return: Void.
    """
    assert environ.get("BNG_HOME") is not None, "Please set the BNG_HOME environment variable."
    vehicle_folder = join(ENV['BNG_HOME'], "vehicles")
    move("87Golf", vehicle_folder)
    levels_folder = join(ENV['BNG_HOME'], "levels")
    move("urban", levels_folder)
    lua_folder = join(ENV['BNG_HOME'], "lua", "ge", "extensions", "scenario")
    move("scenariohelper.lua", lua_folder)


if __name__ == '__main__':
    move_files_to_bng_folder()
    init_scenario_folder()
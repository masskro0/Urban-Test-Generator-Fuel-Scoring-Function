"""This file offers methods to create xml files."""

from utils.dbe_xml_builder import DBEBuilder
from utils.dbc_xml_builder import DBCBuilder
from utils.dbe_xml_builder import save_xml


def build_environment_xml(lanes, tod=0, file_name="fuelTesting", obstacles=None):
    """Creates a environment (dbe) xml file.
    :param tod: Time of day as integer.
    :param lanes: List of lanes.
    :param file_name: Name of this dbe file.
    :param obstacles: List of dicts containing obstacles.
    """
    if obstacles is None:
        obstacles = list()
    dbe = DBEBuilder()
    dbe.add_lanes(lanes)
    if len(obstacles) > 0:
        dbe.add_obstacles(obstacles)
    dbe.set_tod(tod)
    save_xml(file_name, dbe.root, "environment")


def build_criteria_xml(participants, ego_car, success_points, triggers=None, file_name="fuelTesting",
                       name="Fuel Efficiency Test"):
    """Creates a dbc xml file. Failure, success and preconditions are controlled
    manually for this test generation.
    :param triggers: List of trigger points. Check the dbc_xml_builder.py to see how the list should look like.
    :param participants: List of dicts of car states. See the add_car method in dbc_xml_builder.py for more
        information.
    :param ego_car: The test subject as dict. Contains the same information as any other participant.
    :param success_points: List with points of success. Each one is a dict with x, y and tolerance.
    :param file_name: Name of this dbc file. Should be the same as the environment file (laziness).
    :param name: Self defined description name of this file.
    :return: Void.
    """
    dbc = DBCBuilder()
    dbc.define_name(name)
    dbc.environment_name(file_name)
    for participant in participants:
        dbc.add_car(participant)
    for success_point in success_points:
        dbc.add_success_point(ego_car.get("id"), success_point)
    dbc.add_failure_conditions(ego_car.get("id"))
    if triggers is not None:
        dbc.add_trigger_points(triggers)
    save_xml(file_name, dbc.root, "criteria")


def build_xml(individual, iterator: int = 0):
    """Builds an environment and criteria xml file out of a list of control points.
    :param individual: Individual containing obstacles (list), file_name (str), lanes (list), participants (list),
        success_point (dict)
    :param iterator: Unique index of a population.
    :return: Void.
    """
    obstacles = individual.get("obstacles")
    file_name = individual.get("file_name")
    lanes = individual.get("lanes")
    participants = individual.get("participants")
    file_name = file_name + str(iterator)
    success_point = individual.get("success_point")
    triggers = individual.get("triggers")
    success_points = [success_point]
    ego = None
    tod = individual.get("tod")
    for participant in participants:
        if participant.get("id") == "ego":
            ego = participant
            break
    build_environment_xml(lanes=lanes, file_name=file_name, obstacles=obstacles, tod=tod)
    build_criteria_xml(participants=participants, ego_car=ego, success_points=success_points,
                       file_name=file_name, triggers=triggers)


def build_all_xml(population):
    """Calls the build_xml method for each individual.
    :param population: List of individuals.
    :return: Void.
    """
    iterator = 0
    while iterator < len(population):
        build_xml(population[iterator], iterator)
        iterator += 1

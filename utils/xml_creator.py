"""This file creates XML files by using the output provided by the test generator."""

from utils.dbe_xml_builder import DBEBuilder
from utils.dbc_xml_builder import DBCBuilder
from utils.dbe_xml_builder import save_xml


def build_environment_xml(roads, tod=0, file_name="fuelTesting", obstacles=None):
    """Creates a environment (dbe) XML file.
    :param tod: Time of day as integer.
    :param roads: List of roads.
    :param file_name: Name of this dbe file.
    :param obstacles: List of dicts containing obstacles.
    :return: Void.
    """
    if obstacles is None:
        obstacles = list()
    dbe = DBEBuilder()
    dbe.add_roads(roads)
    if len(obstacles) > 0:
        dbe.add_obstacles(obstacles)
    dbe.set_tod(tod)
    save_xml(file_name, dbe.root, "environment")


def build_criteria_xml(participants, ego_car, success_points, triggers=None, file_name="fuelTesting",
                       name="Fuel Efficiency Test", damage_requests=None):
    """Creates a dbc XML file. Failure, success and preconditions are controlled manually for this test generation.
    :param triggers: List of trigger points. Check the dbc_xml_builder.py to see how the list should look like.
    :param participants: List of dicts of car states. See the add_car method in dbc_xml_builder.py for more
        information.
    :param ego_car: The test subject as dict. Contains the same information as any other participant.
    :param success_points: List with points of success. Each one is a dict with x, y and tolerance.
    :param file_name: Name of this dbc file. Should be the same as the environment file.
    :param name: Self defined description name of this file.
    :param damage_requests: List of vehicle IDs which are not allowed to take damage.
    :return: Void.
    """
    dbc = DBCBuilder()
    dbc.define_name(name)
    dbc.environment_name(file_name)
    for participant in participants:
        dbc.add_car(participant)
    for success_point in success_points:
        dbc.add_success_point(ego_car.get("id"), success_point)
    for vid in damage_requests:
        dbc.add_failure_damage(vid)
    if triggers is not None:
        dbc.add_trigger_points(triggers)
    save_xml(file_name, dbc.root, "criteria")


def build_xml(individual, iterator: int = 0):
    """Builds an environment and criteria XML file of the output of the test generator.
    :param individual: Individual containing obstacles (list), file_name (str), roads (list), participants (list),
        success_point (dict)
    :param iterator: Unique index of a population.
    :return: Void.
    """
    obstacles = individual.get("obstacles")
    file_name = individual.get("file_name")
    roads = individual.get("roads")
    participants = individual.get("participants")
    file_name = file_name + str(iterator)
    success_point = individual.get("success_point")
    triggers = individual.get("triggers")
    success_points = [success_point]
    damage_requests = individual.get("damage_requests")
    ego = None
    tod = individual.get("tod")
    for participant in participants:
        if participant.get("id") == "ego":
            ego = participant
            break
    build_environment_xml(roads=roads, file_name=file_name, obstacles=obstacles, tod=tod)
    build_criteria_xml(participants=participants, ego_car=ego, success_points=success_points,
                       file_name=file_name, triggers=triggers, damage_requests=damage_requests)


def build_all_xml(population):
    """Calls the build_xml method for each individual.
    :param population: List of individuals.
    :return: Void.
    """
    iterator = 0
    while iterator < len(population):
        build_xml(population[iterator], iterator)
        iterator += 1

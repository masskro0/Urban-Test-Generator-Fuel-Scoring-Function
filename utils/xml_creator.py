"""This file offers methods to create xml files of a list of control points.
   Methods are preconfigured to meet the requirements of the road_generator.py
   class.
"""

from utils.dbe_xml_builder import DBEBuilder
from utils.dbc_xml_builder import DBCBuilder


def build_environment_xml(lanes, file_name="fuelTesting", obstacles=None):
    """Creates a dbe xml file.
    :param lanes: List of lanes.
    :param file_name: Name of this dbe file.
    :param obstacles: List of dicts containing obstacles.
    """
    if obstacles is None:
        obstacles = []
    dbe = DBEBuilder()
    dbe.add_lanes(lanes)
    if len(obstacles) > 0:
        dbe.add_obstacles(obstacles)
    dbe.save_xml(file_name)


def build_criteria_xml(participants, ego_car, success_points, vc_pos, sc_speed, file_name="fuelTesting",
                       name="Fuel Efficiency Test", fps="60", frequency="6"):
    """Creates a dbc xml file. Failure, success and preconditions are controlled
    manually for this test generation.
    :param participants: List of dicts of car states. See the add_car method in dbc_xml_builder.py for more
        information.
    :param ego_car: The test subject as dict. Contains the same information as any other participant.
    :param success_points: List with points of success. Each one is a dict with x, y and tolerance.
    :param vc_pos: Position which must be entered at a specific speed by a specific participant.
    :param sc_speed: Speed condition that has to be met at vc_pos.
    :param file_name: Name of this dbc file. Should be the same as the environment file (laziness).
    :param name: Self defined description name of this file.
    :param fps: Frames per second.
    :param frequency: Frequency of the AI to compute the next step.
    :return: Void.
    """
    dbc = DBCBuilder()
    dbc.define_name(name)
    dbc.environment_name(file_name)
    dbc.steps_per_second(fps)
    dbc.ai_freq(frequency)
    for participant in participants:
        dbc.add_car(participant)
    for success_point in success_points:
        dbc.add_success_point(ego_car.get("id"), success_point)
    dbc.add_failure_conditions(ego_car.get("id"), "offroad")
    dbc.add_precond_partic_sc_speed(vc_pos, sc_speed)
    dbc.save_xml(file_name)


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
    success_points = [success_point]
    ego = None
    for participant in participants:
        if participant.get("id") == "ego":
            ego = participant
            break
    vc_pos = {"id": ego.get("id"),
              "tolerance": 3,
              "x": '{0:.10f}'.format(round(lanes[0].get("control_points")[1].get("x"))),
              "y": '{0:.10f}'.format(round(lanes[0].get("control_points")[1].get("y")))}
    sc_speed = 10
    build_environment_xml(lanes=lanes, file_name=file_name, obstacles=obstacles)
    build_criteria_xml(participants=participants, ego_car=ego, success_points=success_points,
                       file_name=file_name, vc_pos=vc_pos, sc_speed=sc_speed)


def build_all_xml(population):
    """Calls the build_xml method for each individual.
    :param population: List of individuals.
    :return: Void.
    """
    iterator = 0
    while iterator < len(population):
        build_xml(population[iterator], iterator)
        iterator += 1

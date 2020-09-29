from utils.dbe_xml_builder import DBEBuilder, save_xml
from utils.dbc_xml_builder import DBCBuilder
from xml_converter.bng_converter import b_spline
from os.path import dirname, abspath
from urban_test_generator import _add_ego_car
from utils.utility_functions import calc_speed_waypoints


def main():
    """Script for generating XML files for the curvy test case by using my XML interface.
    :return: Void.
    """
    dbe = DBEBuilder()
    dbe.set_tod(0)
    roads = [{"width": 10, "left_lanes": 1, "right_lanes": 1}]
    control_points = [(1, 0), (35, 60), (70, -20), (115, 30), (160, -70), (185, 40)]
    control_points = b_spline(control_points, samples=150)
    roads[0]["control_points"] = control_points
    dbe.add_roads(roads)
    file_name = "urban0"
    save_xml(file_name, dbe.root, "environment", abspath(dirname(__file__)))

    dbc = DBCBuilder()
    dbc.define_name("Fuel Efficiency Evaluation")
    dbc.environment_name(file_name)
    ego = _add_ego_car(roads, [0], list(), list(), True)
    calc_speed_waypoints([ego])
    del ego["waypoints"][:2]
    ego = {"id": "ego", "init_state": {"position": (4, 2), "orientation": 60}, "model": "etk800", "color": "White",
           "waypoints": ego.get("waypoints")}
    dbc.add_car(ego)
    dbc.add_success_point("ego", {"position": ego["waypoints"][-2].get("position"), "tolerance": 6})
    dbc.add_failure_conditions("ego")
    save_xml(file_name, dbc.root, "criteria", abspath(dirname(__file__)))


if __name__ == '__main__':
    main()

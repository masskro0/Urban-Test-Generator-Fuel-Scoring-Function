from utils.dbe_xml_builder import DBEBuilder, save_xml
from utils.dbc_xml_builder import DBCBuilder
from xml_converter.converter import b_spline
from os.path import dirname, abspath
from fuel_consumption_test_generator import _add_ego_car
from utils.utility_functions import calc_speed_waypoints


def main():
    dbe = DBEBuilder()
    dbe.set_tod(0)
    lanes = [{"width": 10, "left_lanes": 1, "right_lanes": 1}]
    control_points = [(1, 0), (20, 30), (40, 60), (60, 0), (80, -60), (100, 0), (120, 60), (140, 0), (160, -60),
                      (180, 0), (200, 60), (220, 0), (240, -60), (260, 0), (280, 60), (300, 0), (320, -60)]
    control_points = b_spline(control_points, samples=150)
    lanes[0]["control_points"] = control_points
    dbe.add_lanes(lanes)
    file_name = "urban0"
    save_xml(file_name, dbe.root, "environment", abspath(dirname(__file__)))

    dbc = DBCBuilder()
    dbc.define_name("Fuel Efficiency Evaluation")
    dbc.environment_name(file_name)
    temp = {"lanes": lanes, "ego_lanes": [0], "directions": list(), "actions": list()}
    _add_ego_car(temp)
    calc_speed_waypoints(temp.get("participants"))
    ego = temp.get("participants")[0]
    del ego["waypoints"][:2]
    ego = {"id": "ego", "init_state": {"position": (4, 2), "orientation": 60}, "model": "etk800", "color": "White",
           "waypoints": ego.get("waypoints")}
    dbc.add_car(ego)
    dbc.add_success_point("ego", {"position": control_points[-1], "tolerance": 6})
    dbc.add_failure_conditions("ego")
    save_xml(file_name, dbc.root, "criteria", abspath(dirname(__file__)))


if __name__ == '__main__':
    main()

from copy import deepcopy

from numpy.ma import array
from shapely.geometry import LineString

from safety_test_generator import TestGenerator
from utils.validity_checks import intersection_check_last, intersection_check_width
from utils.xml_to_bng_files import convert_test
from fuel_consumption_test_generator import FuelConsumptionTestGenerator
from utils.xml_creator import build_all_xml
from utils.plotter import plotter, plot_splined_list, plot_splines_and_width
from utils.utility_functions import convert_points_to_lines, convert_splines_to_lines
import time

if __name__ == '__main__':
    """
    gen = TestGenerator("easy")      # Parameter is optional, you can also call gen.set_difficulty(str)
    gen.set_files_name("test_case")  # This method call is optional
    
    while True:
        for paths in gen.getTest():
            dbe = paths[0]
            dbc = paths[1]
            #convert_test(dbc, dbe)  # Use this function only when DriveBuild doesn't work for you.
            # All your code goes here
            # Be sure to call onTestFinished(sid, vid)
    
    #dbe = "D:\\Bachelorarbeit\\code\\scenario\\intersection.dbe.xml"
    #dbc = "D:\\Bachelorarbeit\\code\\scenario\\intersection.dbc.xml"
    #convert_test(dbc, dbe)
    """

    gen = FuelConsumptionTestGenerator()
    """
    while True:
        start = time.time()
        list = gen.genetic_algorithm()
        end = time.time()
        print(end - start)
    """
    list = [{'control_points': [{'x': 1, 'y': 0, 'type': 'segment'}, {'x': 50, 'y': 0, 'type': 'segment'}, {'x': 65, 'y': 0, 'type': 'segment'}, {'x': 135, 'y': 14, 'type': 'segment'}, {'x': 145, 'y': -47, 'type': 'segment'}, {'x': 157.94200216531846, 'y': -125.94621320844269, 'type': 'intersection'}], 'width': 8, 'left_lanes': 1, 'right_lanes': 1}, {'control_points': [{'x': -49.29180750684367, 'y': -159.91896889240374, 'type': 'intersection'}, {'x': 157.94200216531846, 'y': -125.94621320844269, 'type': 'intersection'}], 'width': 8, 'left_lanes': 1, 'right_lanes': 1}, {'control_points': [{'x': 157.94200216531846, 'y': -125.94621320844269, 'type': 'intersection'}, {'x': 169.26625405997214, 'y': -195.02414976583003, 'type': 'intersection'}], 'width': 8, 'left_lanes': 1, 'right_lanes': 1}, {'control_points': [{'x': 157.94200216531846, 'y': -125.94621320844269, 'type': 'intersection'}, {'x': 365.1758118374805, 'y': -91.9734575244816, 'type': 'intersection'}, {'x': 370, 'y': -168, 'type': 'segment'}], 'width': 8, 'left_lanes': 1, 'right_lanes': 1}]
    temp_list = gen._bspline(list)
    control_points_lines = convert_splines_to_lines(temp_list)
    width_lines = gen._get_width_lines(temp_list)
    print(intersection_check_width(width_lines, control_points_lines))
    plot_splines_and_width(control_points_lines, width_lines)
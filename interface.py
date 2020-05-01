from copy import deepcopy

from numpy.ma import array
from shapely.geometry import LineString

from safety_test_generator import TestGenerator
from utils.validity_checks import intersection_check_last, intersection_check_width
from utils.xml_to_bng_files import convert_test
from fuel_consumption_test_generator import FuelConsumptionTestGenerator
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
            convert_test(dbc, dbe)  # Use this function only when DriveBuild doesn't work for you.
            # All your code goes here
            # Be sure to call onTestFinished(sid, vid)
    
    dbe = "D:\\Bachelorarbeit\\code\\scenario\\intersection.dbe.xml"
    dbc = "D:\\Bachelorarbeit\\code\\scenario\\intersection.dbc.xml"
    convert_test(dbc, dbe)
    """
    gen = FuelConsumptionTestGenerator()

    while True:
        start = time.time()
        list = gen._create_urban_environment()
        end = time.time()
        print(end - start)
        list = gen._bspline(list)
        widths = gen._get_width_lines(list)
        controls = convert_splines_to_lines(list)
        plot_splines_and_width(widths, controls)

    """
    list = [{'control_points': array([
       [ 3.74416974e+02,  7.72752571e+02],
       [ 3.81000000e+02,  7.41000000e+02]]), 'width': 8}, {'control_points': array([
       [380.85776278, 734.34636231],
       [370.48141606, 737.22868084],
       [360.10506934, 740.11099937],
       [349.72872262, 742.9933179 ]]), 'width': 8}]
    widths = gen._get_width_lines(list)
    controls = convert_splines_to_lines(list)
    print(intersection_check_width(widths, controls))
    plot_splines_and_width(widths, controls)
    """
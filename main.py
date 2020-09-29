"""Interface for generating test cases, setting up and running the simulator."""

from urban_test_generator import UrbanTestGenerator
from xml_converter.xml_to_bng_files import convert_test
from test_execution.test_execution import run_test_case

FILES_NAME = "urban"
TRAFFIC = True
SPLINE_DEGREE = 2
MAX_TRIES = 20
POPULATION_SIZE = 1
MIN_SEGMENT_LENGTH = 10
MAX_SEGMENT_LENGTH = 30
MIN_NODES = 6
MAX_NODES = 16
INTERSECTION_LENGTH = 50
OPPOSITE_ROAD = 30
STRAIGHT_LENGTH = 20
MAX_LEFT_LANES = 2
MAX_RIGHT_LANES = 2
MAX_WIDTH = 5
EGO_WAYPOINTS = True


if __name__ == '__main__':
    gen = UrbanTestGenerator(files_name=FILES_NAME,
                             traffic=TRAFFIC,
                             spline_degree=SPLINE_DEGREE,
                             max_tries=MAX_TRIES,
                             population_size=POPULATION_SIZE,
                             min_segment_length=MIN_SEGMENT_LENGTH,
                             max_segment_length=MAX_SEGMENT_LENGTH,
                             min_nodes=MIN_NODES,
                             max_nodes=MAX_NODES,
                             intersection_length=INTERSECTION_LENGTH,
                             opposite_road_length=OPPOSITE_ROAD,
                             straight_length=STRAIGHT_LENGTH,
                             max_left_lanes=MAX_LEFT_LANES,
                             max_right_lanes=MAX_RIGHT_LANES,
                             max_width=MAX_WIDTH,
                             ego_waypoints=EGO_WAYPOINTS)
    while True:
        for paths in gen.get_test():
            dbe = paths[0]
            dbc = paths[1]
            converter = convert_test(dbc, dbe)
            results = run_test_case(converter, dbc, dbe)
            gen.on_test_finished(float(results.get("score")))

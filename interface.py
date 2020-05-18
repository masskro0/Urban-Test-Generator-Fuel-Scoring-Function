"""Interface for generating test cases."""

from fuel_consumption_test_generator import FuelConsumptionTestGenerator
from utils.xml_to_bng_files import convert_test

if __name__ == '__main__':
    """
    while True:
        for paths in gen.getTest():
            dbe = paths[0]
            dbc = paths[1]
            #convert_test(dbc, dbe)  # Use this function only when DriveBuild doesn't work for you.
            # All your code goes here
            # Be sure to call onTestFinished(sid, vid)
    """

    gen = FuelConsumptionTestGenerator()

    while True:
        for paths in gen.get_test():
            dbe = paths[0]
            dbc = paths[1]
            #convert_test(dbc, dbe)
        var = [{'control_points': [{'x': 1, 'y': 0, 'type': 'segment'}, {'x': 50, 'y': 0, 'type': 'segment'},
                                   {'x': 65, 'y': 0, 'type': 'segment'}, {'x': 93, 'y': 0, 'type': 'intersection'}],
                'width': 10,
                'left_lanes': 1, 'right_lanes': 1, 'samples': 75},
               {'control_points': [{'x': 93, 'y': 0, 'type': 'intersection'},
                                   {'x': 113.0, 'y': 0.0, 'type': 'intersection'}],
                'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25},
               {'control_points': [{'x': 120.87328690203528, 'y': 34.102952267483225, 'type': 'intersection'},
                                   {'x': 113.0, 'y': 0.0, 'type': 'intersection'}], 'width': 15, 'left_lanes': 2,
                'right_lanes': 1, 'samples': 25},
               {'control_points': [{'x': 113.0, 'y': 0.0, 'type': 'intersection'},
                                   {'x': 114.22148238458755, 'y': -34.97867894566835,
                                    'type': 'intersection'}], 'width': 15, 'left_lanes': 2,
                'right_lanes': 1, 'samples': 25},
               {'control_points': [{'x': 113.0, 'y': 0.0, 'type': 'intersection'},
                                   {'x': 143.0, 'y': 0.0, 'type': 'intersection'}], 'width': 10, 'left_lanes': 1,
                'right_lanes': 1, 'samples': 25},
               {'control_points': [{'x': 143.0, 'y': 0.0, 'type': 'intersection'}], 'width': 10, 'left_lanes': 1,
                'right_lanes': 1, 'samples': 75}]

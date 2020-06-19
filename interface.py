"""Interface for generating test cases."""

from fuel_consumption_test_generator import FuelConsumptionTestGenerator
from xml_converter.xml_to_bng_files import convert_test

if __name__ == '__main__':
    gen = FuelConsumptionTestGenerator()

    """
    while True:
        for paths in gen.get_test():
            dbe = paths[0]
            dbc = paths[1]
            convert_test(dbc, dbe)
    """
    dbc = "D:\\Bachelorarbeit\\code\\scenario\\urban0.dbc.xml"
    dbe = "D:\\Bachelorarbeit\\code\\scenario\\urban0.dbe.xml"
    convert_test(dbc, dbe)
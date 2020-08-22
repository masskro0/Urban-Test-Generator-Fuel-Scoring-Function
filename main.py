"""Interface for generating test cases."""

from fuel_consumption_test_generator import FuelConsumptionTestGenerator
from xml_converter.xml_to_bng_files import convert_test
from test_execution.test_execution import run_test_case


if __name__ == '__main__':
    gen = FuelConsumptionTestGenerator()

    while True:
        for paths in gen.get_test():
            dbe = paths[0]
            dbc = paths[1]
            converter = convert_test(dbc, dbe)
            run_test_case(converter, dbc, dbe)

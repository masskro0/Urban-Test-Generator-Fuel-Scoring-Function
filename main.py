"""Interface for generating test cases, setting up and running the simulator."""

from urban_test_generator import UrbanTestGenerator
from xml_converter.xml_to_bng_files import convert_test
from test_execution.test_execution import run_test_case


if __name__ == '__main__':
    gen = UrbanTestGenerator(add_ego=False)
    while True:
        for paths in gen.get_test():
            dbe = paths[0]
            dbc = paths[1]
            converter = convert_test(dbc, dbe)
            results = run_test_case(converter, dbc, dbe)
            gen.on_test_finished(float(results.get("score")))

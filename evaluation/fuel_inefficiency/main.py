from glob import glob
from os import mkdir
from os.path import join, exists

from test_execution.test_execution import run_test_case
from xml_converter.xml_to_bng_files import convert_test


def visualize_results(results, name, profile):
    if not exists("result_pictures"):
        mkdir("result_pictures")


def main():
    # TODO Visualize results, create profiles, move profiles
    test_cases = glob(join("test_cases", "*"))
    for test_case in test_cases:
        results = list()
        files = glob(join(test_case, "*.xml"))
        converter = convert_test(files[0], files[1])
        for profile in profiles:
            results.append(run_test_case(converter, files[0], files[1]))
        name = test_case.split("\\")[-1]
        profile_name = None
        visualize_results(results, name, profile_name)


if __name__ == '__main__':
    main()

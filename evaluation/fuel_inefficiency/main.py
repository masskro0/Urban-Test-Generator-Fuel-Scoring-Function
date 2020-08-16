from glob import glob
from os.path import join
from test_execution import run_test_case

from xml_converter.xml_to_bng_files import convert_test


def main():
    test_cases = glob(join("test_cases", "*"))
    for test_case in test_cases:
        if test_case.endswith("curvy"):
            continue
        files = glob(join(test_case, "*"))
        converter = convert_test(files[0], files[1])
        run_test_case(converter.scenario)


if __name__ == '__main__':
    main()

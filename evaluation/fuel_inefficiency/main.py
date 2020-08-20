from glob import glob
from os import mkdir
from os.path import join, exists
from beamngpy import ENV
from shutil import copyfile
import matplotlib.pyplot as plt
from numpy import arange

from test_execution.test_execution import run_test_case, run_test_case_role_model
from xml_converter.xml_to_bng_files import convert_test


def visualize_results(results):
    print(results)
    # TODO Bar plot for consumed fuel per test case.
    if not exists("result_pictures"):
        mkdir("result_pictures")
    print(results)
    rpm_infractions = list()
    throttle_infractions = list()
    brake_infractions = list()
    accelerate_and_stop_infractions = list()
    engine_idle_infractions = list()
    profiles = list()

    if name == "curvy":
        yticks = 120
    else:
        yticks = 60

    for result in results:
        res = result.get("results")
        rpm_infractions.append(res.get("rpm_infractions"))
        throttle_infractions.append(res.get("throttle_infractions"))
        brake_infractions.append(res.get("brake_infractions"))
        accelerate_and_stop_infractions.append(res.get("accelerate_and_stop_infractions"))
        engine_idle_infractions.append(res.get("engine_idle_infractions"))
        profiles.append(result.get("profile"))

    ind = arange(len(results))  # the x locations for the groups
    width = 0.5  # the width of the bars: can also be len(x) sequence

    p1 = plt.bar(ind, rpm_infractions, width)
    p2 = plt.bar(ind, throttle_infractions, width, bottom=rpm_infractions)
    p3 = plt.bar(ind, brake_infractions, width, bottom=throttle_infractions)
    p4 = plt.bar(ind, engine_idle_infractions, width, bottom=brake_infractions)
    p5 = plt.bar(ind, accelerate_and_stop_infractions, width, bottom=engine_idle_infractions)

    plt.ylabel('Scores/Infractions')
    plt.title('Scores by profile in test case "{}"'.format(name))
    plt.xticks(ind, profiles)
    plt.yticks(arange(0, yticks, yticks/10))
    plt.legend((p1[0], p2[0], p3[0], p4[0], p5[0]), ('RPM', 'Throttle', 'Brake', 'Engine Idle', 'Accelerate-and-Stop'))
    plt.savefig(join("result_pictures", 'fuel_inefficiency_{}.png'.format(name)), bbox_inches='tight')
    plt.show()


def main():
    repeat = 10
    test_cases = glob(join("test_cases", "*"))
    assert exists(ENV["BNG_HOME"]), "Please set your BNG_HOME variable to BeamNG's trunk folder."
    ai_path = join(ENV["BNG_HOME"], "lua", "vehicle", "ai.lua")
    results = list()
    for test_case in test_cases:
        test_case_name = test_case.split("\\")[-1]
        files = glob(join(test_case, "*.xml"))
        converter = convert_test(files[0], files[1])
        profiles = glob(join("ai_profiles", "*"))
        for profile in profiles:
            i = 0
            copyfile(join(profile, "ai.lua"), ai_path)
            profile_name = profile.split("\\")[-1]
            if profile_name == "role_model":
                while i < repeat:
                    results.append({"profile": profile_name, "test_case": test_case_name,
                                    "results": run_test_case_role_model(converter, files[0], files[1])})
                    i += 1
            else:
                while i < repeat:
                    results.append({"profile": profile_name, "test_case": test_case_name,
                                    "results": run_test_case(converter, files[0], files[1])})
                    i += 1
    visualize_results(results)
    copyfile(join("ai_profiles", "default", "ai.lua"), ai_path)


if __name__ == '__main__':
    main()

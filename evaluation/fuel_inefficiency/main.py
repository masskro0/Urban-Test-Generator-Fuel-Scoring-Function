from glob import glob
from math import ceil
from os import mkdir
from os.path import join, exists
from beamngpy import ENV
from shutil import copyfile
import matplotlib.pyplot as plt
from numpy import arange

from test_execution.test_execution import run_test_case, run_test_case_role_model
from xml_converter.xml_to_bng_files import convert_test


def get_avg_value(my_list):
    temp = 0
    for value in my_list:
        temp += value
    return round(temp / len(my_list))


def visualize_results(results):
    print(results)
    if not exists("result_pictures"):
        mkdir("result_pictures")

    test_case = None
    profile = None
    rpm_all = list()
    throttle_all = list()
    brake_all = list()
    accelerate_and_stop_all = list()
    engine_idle_all = list()
    scores_all = list()
    infractions_all = list()
    consumed_fuel_all = list()
    rpm_ind = list()
    throttle_ind = list()
    brake_ind = list()
    accelerate_and_stop_ind = list()
    engine_idle_ind = list()
    scores_ind = list()
    infractions_ind = list()
    consumed_fuel_ind = list()
    profiles = ["Aggressive", "Default", "Role Model"]
    for idx, result in enumerate(results):
        if profile != result.get("profile") or idx == len(results) - 1:
            profile = result.get("profile")
            if len(rpm_ind) > 0:
                rpm_all.append(get_avg_value(rpm_ind))
                throttle_all.append(get_avg_value(throttle_ind))
                brake_all.append(get_avg_value(brake_ind))
                accelerate_and_stop_all.append(get_avg_value(accelerate_and_stop_ind))
                engine_idle_all.append(get_avg_value(engine_idle_ind))
                scores_all.append(scores_ind)
                infractions_all.append(infractions_ind)
                consumed_fuel_all.append(consumed_fuel_ind)
            rpm_ind = list()
            throttle_ind = list()
            brake_ind = list()
            accelerate_and_stop_ind = list()
            engine_idle_ind = list()
            scores_ind = list()
            infractions_ind = list()
            consumed_fuel_ind = list()
        if test_case != result.get("test_case") or idx == len(results) - 1:
            if len(rpm_all) > 0:
                yticks = 0
                i = 0
                while i < len(rpm_all):
                    temp = rpm_all[i] + throttle_all[i] + brake_all[i] + engine_idle_all[i] \
                           + accelerate_and_stop_all[i]
                    if yticks < temp:
                        yticks = temp
                    i += 1
                yticks = ceil(round(yticks * 1.2) / 10) * 10
                ind = arange(len(rpm_all))  # the x locations for the groups
                width = 0.5
                p1 = plt.bar(ind, rpm_all, width, color="b")
                p2 = plt.bar(ind, throttle_all, width, bottom=rpm_all, color="g")
                p3 = plt.bar(ind, brake_all, width, bottom=[rpm_all[j] + throttle_all[j] for j in range(len(rpm_all))],
                             color="r")
                p4 = plt.bar(ind, engine_idle_all, width, color="k",
                             bottom=[rpm_all[j] + throttle_all[j] + brake_all[j] for j in range(len(rpm_all))])
                p5 = plt.bar(ind, accelerate_and_stop_all, width, color="peru",
                             bottom=[rpm_all[j] + throttle_all[j] + brake_all[j] + engine_idle_all[j]
                                     for j in range(len(rpm_all))])
                plt.ylabel('Infractions')
                plt.title('Average Infractions by Profile in Test Case "{}"'.format(test_case))
                plt.xticks(ind, profiles)
                plt.yticks(arange(0, yticks, yticks / 10))
                plt.legend((p1[0], p2[0], p3[0], p4[0], p5[0]),
                           ('RPM', 'Throttle', 'Brake', 'Engine Idle', 'Accelerate-and-Stop'))
                plt.savefig(join("result_pictures", 'infractions_bar_{}.png'.format(test_case)),
                            bbox_inches='tight')
                plt.clf()

                plt.ylabel('Score')
                plt.title('Scores by Profile in Test Case "{}"'.format(test_case))
                plt.boxplot(scores_all, notch=False, sym="o", labels=profiles)
                plt.savefig(join("result_pictures", 'scores_boxplot_{}.png'.format(test_case)),
                            bbox_inches='tight')
                plt.clf()

                plt.ylabel('Infractions')
                plt.title('Infractions by Profile in Test Case "{}"'.format(test_case))
                plt.boxplot(infractions_all, notch=False, sym="o", labels=profiles)
                plt.savefig(join("result_pictures", 'infractions_boxplot_{}.png'.format(test_case)),
                            bbox_inches='tight')
                plt.clf()

                plt.ylabel('Consumed Fuel')
                plt.title('Consumed Fuel by Profile in Test Case "{}"'.format(test_case))
                plt.boxplot(consumed_fuel_all, notch=False, sym="o", labels=profiles)
                plt.savefig(join("result_pictures", 'consumed_fuel_boxplot_{}.png'.format(test_case)),
                            bbox_inches='tight')
                plt.clf()

            test_case = result.get("test_case")
            rpm_all = list()
            throttle_all = list()
            brake_all = list()
            accelerate_and_stop_all = list()
            engine_idle_all = list()
            scores_all = list()
            infractions_all = list()
            consumed_fuel_all = list()
        test_results = result.get("results")
        rpm_ind.append(test_results.get("rpm_infractions"))
        throttle_ind.append(test_results.get("throttle_infractions"))
        accelerate_and_stop_ind.append(test_results.get("accelerate_and_stop_infractions"))
        brake_ind.append(test_results.get("brake_infractions"))
        engine_idle_ind.append(test_results.get("engine_idle_infractions"))
        scores_ind.append(test_results.get("score"))
        infractions_ind.append(rpm_ind[-1] + throttle_ind[-1] + brake_ind[-1] + engine_idle_ind[-1]
                               + accelerate_and_stop_ind[-1])
        consumed_fuel_ind.append(test_results.get("consumed_fuel"))


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

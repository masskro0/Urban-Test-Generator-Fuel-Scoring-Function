from glob import glob
from math import ceil
from os import mkdir
from os.path import join, exists
from beamngpy import ENV
from shutil import copyfile
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from numpy import arange, array
from scipy.stats import spearmanr, kendalltau
from termcolor import colored

from test_execution.test_execution import run_test_case, run_test_case_role_model
from xml_converter.xml_to_bng_files import convert_test


def get_avg_value(my_list):
    """Returns the arithmetic mean/average value of a list.
    :param my_list: List with Integers/Floats.
    :return: Arithmetic mean/average value.
    """
    temp = 0
    for value in my_list:
        temp += value
    return round(temp / len(my_list)) if isinstance(my_list[0], int) else temp / len(my_list)


def set_box_color(bp, color):
    """Sets the color for all box properties of a boxplot.
    :param bp: Boxplot.
    :param color: Color.
    :return: Void.
    """
    plt.setp(bp['boxes'], color=color)
    plt.setp(bp['whiskers'], color=color)
    plt.setp(bp['caps'], color=color)
    plt.setp(bp['medians'], color=color)


def boxplot_settings(bpl, bpr, profiles):
    """Applies predefined settings on a boxplot. This sets the color, plots and sets the label for each test case.
     Additionally plots the legend and adjusts the x range.
    :param bpl: Left box.
    :param bpr: Right box.
    :param profiles: List of profiles.
    :return: Void.
    """
    set_box_color(bpl, '#D7191C')
    set_box_color(bpr, '#2C7BB6')
    plt.plot([], c='#D7191C', label='Curvy')
    plt.plot([], c='#2C7BB6', label='Intersection')
    plt.legend()
    plt.xticks(arange(0, len(profiles) * 2, 2), profiles)


def plot_scores(scores_all_tests, profiles):
    """Creates a boxplots for scores.
    :param scores_all_tests: List with sublists containing scores for each profile.
    :param profiles: List of profiles.
    :return: Void.
    """
    plt.subplot(1, 2, 2)
    plt.ylabel('Score')
    bpl = plt.boxplot(scores_all_tests[0], notch=False, sym="o",
                      positions=array(arange(len(scores_all_tests[0]))) * 2.0 - 0.3)
    bpr = plt.boxplot(scores_all_tests[1], notch=False, sym="o",
                      positions=array(arange(len(scores_all_tests[1]))) * 2.0 + 0.3)
    boxplot_settings(bpl, bpr, profiles)
    plt.title("Scores")


def visualize_results(results):
    """Creates all plots for this experiment and additionally prints the Kendall correlation.
    :param results: Results after running main().
    :return: Void.
    """
    if not exists("result_pictures"):
        mkdir("result_pictures")

    test_case = None    # Current test case name.
    profile = None      # Current profile name.

    # Lists containing average infractions of all profiles for one test case.
    rpm_all, throttle_all, brake_all, accelerate_and_stop_all, engine_idle_all, scores_all, infractions_all \
        = ([] for _ in range(7))

    # Lists containing number of infractions for each test run for one profile.
    rpm_ind, throttle_ind, brake_ind, accelerate_and_stop_ind, engine_idle_ind, scores_ind, infractions_ind,\
    consumed_fuel_ind = ([] for _ in range(8))

    # Lists saving various information of all profiles over one test case.
    fuel_list, score_list, curvy_list, intersection_list, scores_all_tests, fuel_list_all, score_list_all, ind_data \
        = ([] for _ in range(8))

    # Profiles which should be evaluated.
    profiles = ["Aggressive", "Default", "Role Model"]
    for idx, result in enumerate(results):
        if profile != result.get("profile") or idx == len(results) - 1:
            if len(rpm_ind) > 0:
                # Append average infraction value, scores and total number of infractions of one profile to
                # test-specific list.
                rpm_all.append(get_avg_value(rpm_ind))
                throttle_all.append(get_avg_value(throttle_ind))
                brake_all.append(get_avg_value(brake_ind))
                accelerate_and_stop_all.append(get_avg_value(accelerate_and_stop_ind))
                engine_idle_all.append(get_avg_value(engine_idle_ind))
                scores_all.append(scores_ind)
                infractions_all.append(infractions_ind)
            if test_case is not None:
                # Collect data in one list.
                ind_data.append({"test_case": test_case, "profile": profile,
                                 "rpm": rpm_ind, "throttle": throttle_ind, "brake": brake_ind,
                                 "engine_idle": engine_idle_ind,
                                 "accelerate_and_stop": accelerate_and_stop_ind,
                                 "infractions": infractions_ind,
                                 "consumed_fuel": consumed_fuel_ind})
            profile = result.get("profile")
            score_list.extend(scores_ind)
            fuel_list.extend(consumed_fuel_ind)
            rpm_ind = list()
            throttle_ind = list()
            brake_ind = list()
            accelerate_and_stop_ind = list()
            engine_idle_ind = list()
            scores_ind = list()
            infractions_ind = list()
            consumed_fuel_ind = list()

        if test_case != result.get("test_case") or idx == len(results) - 1:
            if test_case is not None:
                spearman = spearmanr(score_list, fuel_list)
                print(colored("Spearman correlation for test case \"{}\": {}.".format(test_case, spearman), "grey",
                              attrs=['bold']))
                kendall = kendalltau(score_list, fuel_list)
                print(colored("Kendall correlation for test case \"{}\": {}.".format(test_case, kendall), "grey",
                              attrs=['bold']))
                fuel_list_all.extend(fuel_list)
                score_list_all.extend(score_list)
                score_list = list()
                fuel_list = list()
                scores_all_tests.append(scores_all)
            if test_case == "curvy":
                curvy_list.extend([rpm_all, throttle_all, brake_all, engine_idle_all, accelerate_and_stop_all])
            elif test_case == "intersection":
                intersection_list.extend([rpm_all, throttle_all, brake_all, engine_idle_all, accelerate_and_stop_all])
                kendall = kendalltau(score_list_all, fuel_list_all)
                print(colored("Kendall correlation for all test cases: {}.".format(kendall), "grey",
                              attrs=['bold']))

            test_case = result.get("test_case")
            rpm_all = list()
            throttle_all = list()
            brake_all = list()
            accelerate_and_stop_all = list()
            engine_idle_all = list()
            scores_all = list()
            infractions_all = list()
        if idx == len(results) - 1:
            # Create plots.
            yticks = 0
            test_cases = [curvy_list, intersection_list]

            # Get y range.
            for test in test_cases:
                i = 0
                while i < len(test[0]):
                    current_value = test[0][i] + test[1][i] + test[2][i] + test[3][i] + test[4][i]
                    if yticks < current_value:
                        yticks = current_value
                    i += 1
            yticks = ceil(round(yticks * 1.1) / 10) * 10
            ind = arange(len(curvy_list[0]))  # the x locations for the groups
            width = 0.35

            # Create grouped stacked bar plot for infractions and grouped boxplots for scores side by side.
            plt.figure(figsize=(11, 5))
            plt.subplot(1, 2, 1)

            # RPM Bar.
            p1 = plt.bar(ind - width / 2, curvy_list[0], width, color="b")

            # Throttle Bar.
            p2 = plt.bar(ind - width / 2, curvy_list[1], width, bottom=curvy_list[0], color="g")

            # Brake Bar.
            p3 = plt.bar(ind - width / 2, curvy_list[2], width,
                         bottom=[curvy_list[0][j] + curvy_list[1][j] for j in range(len(curvy_list[0]))], color="r")

            # Engine Idle Bar.
            p4 = plt.bar(ind - width / 2, curvy_list[3], width, color="grey",
                         bottom=[curvy_list[0][j] + curvy_list[1][j] + curvy_list[2][j]
                                 for j in range(len(curvy_list[0]))])

            # Accelerate-and-Stop Bar.
            p5 = plt.bar(ind - width / 2, curvy_list[4], width, color="peru",
                         bottom=[curvy_list[0][j] + curvy_list[1][j] + curvy_list[2][j] + curvy_list[3][j]
                                 for j in range(len(curvy_list[0]))])

            # Same for test case intersection.
            plt.bar(ind + width / 2 + 0.05, intersection_list[0], width, color="b", hatch="/")
            plt.bar(ind + width / 2 + 0.05, intersection_list[1], width, bottom=intersection_list[0], color="g",
                    hatch="/")
            plt.bar(ind + width / 2 + 0.05, intersection_list[2], width,
                    bottom=[intersection_list[0][j] + intersection_list[1][j]
                            for j in range(len(intersection_list[0]))], color="r", hatch="/")
            plt.bar(ind + width / 2 + 0.05, intersection_list[3], width, color="grey",
                    bottom=[intersection_list[0][j] + intersection_list[1][j] + intersection_list[2][j]
                            for j in range(len(intersection_list[0]))], hatch="/")
            plt.bar(ind + width / 2 + 0.05, intersection_list[4], width, color="peru",
                    bottom=[intersection_list[0][j] + intersection_list[1][j] + intersection_list[2][j]
                            + intersection_list[3][j]
                            for j in range(len(intersection_list[0]))], hatch="/")
            plt.ylabel('Infractions')
            plt.title('Average Infractions for each Profile')
            plt.xticks(ind, profiles)
            plt.yticks(arange(0, yticks, yticks / 10))

            # Infraction legend.
            first_legend = plt.legend((p1[0], p2[0], p3[0], p4[0], p5[0]),
                                      ('RPM', 'Throttle', 'Brake', 'Engine Idle', 'Accelerate-and-Stop'))
            plt.gca().add_artist(first_legend)

            # Symbols for test-case specific legend.
            case1 = Patch(facecolor="lightgrey", alpha=1, label='Curvy')
            case2 = Patch(facecolor="lightgrey", alpha=1, hatch=r'\\\\\\', label='Intersection')
            plt.legend(handles=[case1, case2], loc='center right', bbox_to_anchor=(1, 0.61))

            # Create boxplot of scores.
            plot_scores(scores_all_tests, profiles)
            plt.tight_layout()

            # Save image.
            plt.savefig(join("result_pictures", 'infractions_and_scores.png'), bbox_inches='tight', dpi=200)
            plt.clf()

            # Creates boxplots of both test cases side by side for each infraction, total infractions and consumed fuel
            # individually.
            plt.figure(figsize=(8, 6))
            values = ["rpm", "throttle", "brake", "engine_idle", "accelerate_and_stop", "infractions", "consumed_fuel"]
            for val in values:
                curvy = [d[val] for d in ind_data if val in d and d['test_case'] == "curvy"]
                intersection = [d[val] for d in ind_data if val in d and d['test_case'] == "intersection"]
                if val == "consumed_fuel":
                    plt.figure(figsize=(11, 5))
                    plt.subplot(1, 2, 1)
                plt.ylabel('Infractions')
                bpl = plt.boxplot(curvy, notch=False, sym="o",
                                  positions=array(arange(len(curvy))) * 2.0 - 0.3)              # Left box
                bpr = plt.boxplot(intersection, notch=False, sym="o",
                                  positions=array(arange(len(intersection))) * 2.0 + 0.3)       # Right box
                boxplot_settings(bpl, bpr, profiles)
                file_name = val + '_infraction_boxplots.png'
                if val == "rpm":
                    title = 'Box Plots for RPM Infractions'
                elif val == "throttle":
                    title = 'Box Plots for Throttle Infractions'
                elif val == "brake":
                    title = 'Box Plots for Brake Infractions'
                elif val == "engine_idle":
                    title = 'Box Plots for Engine Idling Infractions'
                elif val == "accelerate_and_stop":
                    title = 'Box Plots for Accelerate-and-Stop Infractions'
                elif val == "infractions":
                    title = "Total Number of Infractions"
                    file_name = "infractions_boxplots.png"
                elif val == "consumed_fuel":
                    plt.ylabel('Consumed Fuel')
                    title = "Consumed Fuel"
                    file_name = "consumed_fuel_and_scores_boxplots.png"
                else:
                    raise ValueError("Unknown infraction: {}".format(val))
                plt.title(title)
                if val == "consumed_fuel":
                    plot_scores(scores_all_tests, profiles)
                plt.tight_layout()
                plt.savefig(join("result_pictures", file_name), bbox_inches='tight', dpi=200)
                plt.clf()

        test_results = result.get("results")

        # Store all infractions in separate lists for each profile of one test case.
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
    """Runs test cases x times to collect results and calls visualize_results() method to create plots.
    :return: Void.
    """
    repeat = 10     # Number of repetitions of one test case per profile.
    test_cases = glob(join("test_cases", "*"))
    assert exists(ENV["BNG_HOME"]), "Please set your BNG_HOME variable to BeamNG's trunk folder."
    ai_path = join(ENV["BNG_HOME"], "lua", "vehicle", "ai.lua")
    results = list()
    for test_case in test_cases:
        # Run each test case.
        test_case_name = test_case.split("\\")[-1]
        files = glob(join(test_case, "*.xml"))
        converter = convert_test(files[0], files[1])    # Convert XML files to simulation data.
        profiles = glob(join("ai_profiles", "*"))
        for profile in profiles:
            # Replace current BeamNG AI file with the profile.
            copyfile(join(profile, "ai.lua"), ai_path)
            i = 0
            profile_name = profile.split("\\")[-1]
            if profile_name == "role_model":
                while i < repeat:
                    # Run test case {@code repeat} times and collect result for role model.
                    results.append({"profile": profile_name, "test_case": test_case_name,
                                    "results": run_test_case_role_model(converter, files[0], files[1])})
                    i += 1
            else:
                while i < repeat:
                    # Run test case {@code repeat} times and collect result.
                    results.append({"profile": profile_name, "test_case": test_case_name,
                                    "results": run_test_case(converter, files[0], files[1])})
                    i += 1
    visualize_results(results)  # Create plots.
    copyfile(join("ai_profiles", "default", "ai.lua"), ai_path)     # Restore default AI file.


if __name__ == '__main__':
    main()

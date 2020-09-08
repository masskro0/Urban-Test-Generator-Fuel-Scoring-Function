from glob import glob
from math import ceil
from os import mkdir
from os.path import join, exists
from beamngpy import ENV
from shutil import copyfile
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from numpy import arange, array
from scipy.stats import spearmanr, kendalltau
from termcolor import colored

from test_execution.test_execution import run_test_case, run_test_case_role_model
from xml_converter.xml_to_bng_files import convert_test


def get_avg_value(my_list):
    temp = 0
    for value in my_list:
        temp += value
    return round(temp / len(my_list)) if isinstance(my_list[0], int) else temp / len(my_list)


def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color)
    plt.setp(bp['whiskers'], color=color)
    plt.setp(bp['caps'], color=color)
    plt.setp(bp['medians'], color=color)


def boxplot_settings(bpl, bpr, profiles):
    set_box_color(bpl, '#D7191C')
    set_box_color(bpr, '#2C7BB6')
    plt.plot([], c='#D7191C', label='Curvy')
    plt.plot([], c='#2C7BB6', label='Intersection')
    plt.legend()
    plt.xticks(arange(0, len(profiles) * 2, 2), profiles)


def plot_scores(scores_all_tests, profiles):
    plt.subplot(1, 2, 2)
    plt.ylabel('Score')
    bpl = plt.boxplot(scores_all_tests[0], notch=False, sym="o",
                      positions=array(arange(len(scores_all_tests[0]))) * 2.0 - 0.3)
    bpr = plt.boxplot(scores_all_tests[1], notch=False, sym="o",
                      positions=array(arange(len(scores_all_tests[1]))) * 2.0 + 0.3)
    boxplot_settings(bpl, bpr, profiles)
    plt.title("Scores")


def visualize_results(results):
    print(results)
    if not exists("result_pictures"):
        mkdir("result_pictures")

    test_case = None
    profile = None
    rpm_all, throttle_all, brake_all, accelerate_and_stop_all, engine_idle_all, scores_all, infractions_all \
        = ([] for _ in range(7))
    rpm_ind, throttle_ind, brake_ind, accelerate_and_stop_ind, engine_idle_ind, scores_ind, infractions_ind,\
    consumed_fuel_ind = ([] for _ in range(8))
    fuel_list, score_list, curvy_list, intersection_list, scores_all_tests, ind_data = ([] for _ in range(6))
    profiles = ["Aggressive", "Default", "Role Model"]
    for idx, result in enumerate(results):
        if profile != result.get("profile") or idx == len(results) - 1:
            if len(rpm_ind) > 0:
                rpm_all.append(get_avg_value(rpm_ind))
                throttle_all.append(get_avg_value(throttle_ind))
                brake_all.append(get_avg_value(brake_ind))
                accelerate_and_stop_all.append(get_avg_value(accelerate_and_stop_ind))
                engine_idle_all.append(get_avg_value(engine_idle_ind))
                scores_all.append(scores_ind)
                infractions_all.append(infractions_ind)
            if test_case is not None:
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
                score_list = list()
                fuel_list = list()
                scores_all_tests.append(scores_all)
            if test_case == "curvy":
                curvy_list.extend([rpm_all, throttle_all, brake_all, engine_idle_all, accelerate_and_stop_all])
            elif test_case == "intersection":
                intersection_list.extend([rpm_all, throttle_all, brake_all, engine_idle_all, accelerate_and_stop_all])

            test_case = result.get("test_case")
            rpm_all = list()
            throttle_all = list()
            brake_all = list()
            accelerate_and_stop_all = list()
            engine_idle_all = list()
            scores_all = list()
            infractions_all = list()
        if idx == len(results) - 1:
            yticks = 0
            test_cases = [curvy_list, intersection_list]
            for test in test_cases:
                i = 0
                while i < len(test[0]):
                    temp = test[0][i] + test[1][i] + test[2][i] + test[3][i] + test[4][i]
                    if yticks < temp:
                        yticks = temp
                    i += 1
            yticks = ceil(round(yticks * 1.1) / 10) * 10
            ind = arange(len(curvy_list[0]))  # the x locations for the groups
            width = 0.35
            plt.figure(figsize=(11, 5))
            plt.subplot(1, 2, 1)
            p1 = plt.bar(ind - width / 2, curvy_list[0], width, color="b")
            p2 = plt.bar(ind - width / 2, curvy_list[1], width, bottom=curvy_list[0], color="g")
            p3 = plt.bar(ind - width / 2, curvy_list[2], width,
                         bottom=[curvy_list[0][j] + curvy_list[1][j] for j in range(len(curvy_list[0]))], color="r")
            p4 = plt.bar(ind - width / 2, curvy_list[3], width, color="grey",
                         bottom=[curvy_list[0][j] + curvy_list[1][j] + curvy_list[2][j]
                                 for j in range(len(curvy_list[0]))])
            p5 = plt.bar(ind - width / 2, curvy_list[4], width, color="peru",
                         bottom=[curvy_list[0][j] + curvy_list[1][j] + curvy_list[2][j] + curvy_list[3][j]
                                 for j in range(len(curvy_list[0]))])
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
            first_legend = plt.legend((p1[0], p2[0], p3[0], p4[0], p5[0]),
                                      ('RPM', 'Throttle', 'Brake', 'Engine Idle', 'Accelerate-and-Stop'))
            plt.gca().add_artist(first_legend)
            case1 = mpatches.Patch(facecolor="lightgrey", alpha=1, label='Curvy')
            case2 = mpatches.Patch(facecolor="lightgrey", alpha=1, hatch=r'\\\\\\', label='Intersection')
            plt.legend(handles=[case1, case2], loc='center right', bbox_to_anchor=(1, 0.61))

            plot_scores(scores_all_tests, profiles)
            plt.tight_layout()
            plt.savefig(join("result_pictures", 'infractions_and_scores.png'), bbox_inches='tight', dpi=200)
            plt.clf()

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
                                  positions=array(arange(len(curvy))) * 2.0 - 0.3)
                bpr = plt.boxplot(intersection, notch=False, sym="o",
                                  positions=array(arange(len(intersection))) * 2.0 + 0.3)
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
    # main()
    res = [{'profile': 'aggressive', 'test_case': 'curvy', 'results': {'throttle_infractions': 8, 'rpm_infractions': 21, 'fuel': 0.9966903089941753, 'consumed_fuel': 0.0033096910058246776, 'accelerate_and_stop_infractions': 5, 'brake_infractions': 4, 'engine_idle_infractions': 0, 'score': 304}}, {'profile': 'aggressive', 'test_case': 'curvy', 'results': {'throttle_infractions': 8, 'rpm_infractions': 22, 'fuel': 0.9966928506212895, 'consumed_fuel': 0.003307149378710461, 'accelerate_and_stop_infractions': 4, 'brake_infractions': 4, 'engine_idle_infractions': 0, 'score': 304}}, {'profile': 'aggressive', 'test_case': 'curvy', 'results': {'throttle_infractions': 10, 'rpm_infractions': 22, 'fuel': 0.9966619902444449, 'consumed_fuel': 0.0033380097555550936, 'accelerate_and_stop_infractions': 5, 'brake_infractions': 3, 'engine_idle_infractions': 0, 'score': 310}}, {'profile': 'aggressive', 'test_case': 'curvy', 'results': {'throttle_infractions': 10, 'rpm_infractions': 22, 'fuel': 0.9966358602969333, 'consumed_fuel': 0.003364139703066704, 'accelerate_and_stop_infractions': 4, 'brake_infractions': 3, 'engine_idle_infractions': 0, 'score': 303}}, {'profile': 'aggressive', 'test_case': 'curvy', 'results': {'throttle_infractions': 10, 'rpm_infractions': 22, 'fuel': 0.9966239019446075, 'consumed_fuel': 0.003376098055392518, 'accelerate_and_stop_infractions': 5, 'brake_infractions': 3, 'engine_idle_infractions': 0, 'score': 314}}, {'profile': 'aggressive', 'test_case': 'curvy', 'results': {'throttle_infractions': 9, 'rpm_infractions': 22, 'fuel': 0.9966289410431068, 'consumed_fuel': 0.003371058956893158, 'accelerate_and_stop_infractions': 6, 'brake_infractions': 4, 'engine_idle_infractions': 0, 'score': 315}}, {'profile': 'aggressive', 'test_case': 'curvy', 'results': {'throttle_infractions': 9, 'rpm_infractions': 22, 'fuel': 0.9966775084136259, 'consumed_fuel': 0.003322491586374099, 'accelerate_and_stop_infractions': 6, 'brake_infractions': 4, 'engine_idle_infractions': 0, 'score': 314}}, {'profile': 'aggressive', 'test_case': 'curvy', 'results': {'throttle_infractions': 8, 'rpm_infractions': 22, 'fuel': 0.9967038548772296, 'consumed_fuel': 0.003296145122770433, 'accelerate_and_stop_infractions': 5, 'brake_infractions': 4, 'engine_idle_infractions': 0, 'score': 304}}, {'profile': 'aggressive', 'test_case': 'curvy', 'results': {'throttle_infractions': 8, 'rpm_infractions': 22, 'fuel': 0.9967069368366158, 'consumed_fuel': 0.00329306316338418, 'accelerate_and_stop_infractions': 4, 'brake_infractions': 3, 'engine_idle_infractions': 0, 'score': 299}}, {'profile': 'aggressive', 'test_case': 'curvy', 'results': {'throttle_infractions': 8, 'rpm_infractions': 22, 'fuel': 0.9967293282704659, 'consumed_fuel': 0.0032706717295341026, 'accelerate_and_stop_infractions': 4, 'brake_infractions': 3, 'engine_idle_infractions': 0, 'score': 300}}, {'profile': 'default', 'test_case': 'curvy', 'results': {'throttle_infractions': 3, 'rpm_infractions': 19, 'fuel': 0.9980753068307119, 'consumed_fuel': 0.0019246931692881164, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 0, 'score': 157}}, {'profile': 'default', 'test_case': 'curvy', 'results': {'throttle_infractions': 2, 'rpm_infractions': 19, 'fuel': 0.9980307213565213, 'consumed_fuel': 0.0019692786434787335, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 0, 'score': 164}}, {'profile': 'default', 'test_case': 'curvy', 'results': {'throttle_infractions': 2, 'rpm_infractions': 19, 'fuel': 0.9980404961267824, 'consumed_fuel': 0.0019595038732176384, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 0, 'score': 159}}, {'profile': 'default', 'test_case': 'curvy', 'results': {'throttle_infractions': 3, 'rpm_infractions': 19, 'fuel': 0.9980582290865658, 'consumed_fuel': 0.001941770913434171, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 0, 'score': 163}}, {'profile': 'default', 'test_case': 'curvy', 'results': {'throttle_infractions': 2, 'rpm_infractions': 19, 'fuel': 0.998042809287581, 'consumed_fuel': 0.0019571907124189547, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 0, 'score': 158}}, {'profile': 'default', 'test_case': 'curvy', 'results': {'throttle_infractions': 2, 'rpm_infractions': 19, 'fuel': 0.9980717064958587, 'consumed_fuel': 0.0019282935041412541, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 0, 'score': 158}}, {'profile': 'default', 'test_case': 'curvy', 'results': {'throttle_infractions': 2, 'rpm_infractions': 19, 'fuel': 0.9980608130089079, 'consumed_fuel': 0.001939186991092079, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 0, 'score': 164}}, {'profile': 'default', 'test_case': 'curvy', 'results': {'throttle_infractions': 2, 'rpm_infractions': 19, 'fuel': 0.9980692826383346, 'consumed_fuel': 0.0019307173616653905, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 0, 'score': 157}}, {'profile': 'default', 'test_case': 'curvy', 'results': {'throttle_infractions': 2, 'rpm_infractions': 19, 'fuel': 0.998051595222767, 'consumed_fuel': 0.0019484047772330149, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 0, 'score': 158}}, {'profile': 'default', 'test_case': 'curvy', 'results': {'throttle_infractions': 2, 'rpm_infractions': 19, 'fuel': 0.998075533131283, 'consumed_fuel': 0.0019244668687169453, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 0, 'score': 156}}, {'profile': 'role_model', 'test_case': 'curvy', 'results': {'throttle_infractions': 0, 'rpm_infractions': 2, 'fuel': 0.9988833590850743, 'consumed_fuel': 0.0011166409149256928, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 1, 'engine_idle_infractions': 0, 'score': 9}}, {'profile': 'role_model', 'test_case': 'curvy', 'results': {'throttle_infractions': 0, 'rpm_infractions': 2, 'fuel': 0.9988804143574568, 'consumed_fuel': 0.0011195856425432194, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 1, 'engine_idle_infractions': 0, 'score': 9}}, {'profile': 'role_model', 'test_case': 'curvy', 'results': {'throttle_infractions': 0, 'rpm_infractions': 1, 'fuel': 0.9988854157577065, 'consumed_fuel': 0.0011145842422934926, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 0, 'score': 3}}, {'profile': 'role_model', 'test_case': 'curvy', 'results': {'throttle_infractions': 0, 'rpm_infractions': 2, 'fuel': 0.9989162987778079, 'consumed_fuel': 0.001083701222192146, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 0, 'engine_idle_infractions': 0, 'score': 3}}, {'profile': 'role_model', 'test_case': 'curvy', 'results': {'throttle_infractions': 0, 'rpm_infractions': 2, 'fuel': 0.9988848277044785, 'consumed_fuel': 0.0011151722955214938, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 0, 'engine_idle_infractions': 0, 'score': 3}}, {'profile': 'role_model', 'test_case': 'curvy', 'results': {'throttle_infractions': 0, 'rpm_infractions': 1, 'fuel': 0.9988975328883372, 'consumed_fuel': 0.0011024671116628415, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 0, 'engine_idle_infractions': 0, 'score': 1}}, {'profile': 'role_model', 'test_case': 'curvy', 'results': {'throttle_infractions': 0, 'rpm_infractions': 1, 'fuel': 0.9989108679941131, 'consumed_fuel': 0.0010891320058868947, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 0, 'engine_idle_infractions': 0, 'score': 4}}, {'profile': 'role_model', 'test_case': 'curvy', 'results': {'throttle_infractions': 0, 'rpm_infractions': 1, 'fuel': 0.9988847513578023, 'consumed_fuel': 0.001115248642197697, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 0, 'engine_idle_infractions': 0, 'score': 2}}, {'profile': 'role_model', 'test_case': 'curvy', 'results': {'throttle_infractions': 0, 'rpm_infractions': 3, 'fuel': 0.9989157931379348, 'consumed_fuel': 0.001084206862065229, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 0, 'engine_idle_infractions': 0, 'score': 6}}, {'profile': 'role_model', 'test_case': 'curvy', 'results': {'throttle_infractions': 0, 'rpm_infractions': 1, 'fuel': 0.9988818591673337, 'consumed_fuel': 0.001118140832666259, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 0, 'engine_idle_infractions': 0, 'score': 4}}, {'profile': 'aggressive', 'test_case': 'intersection', 'results': {'throttle_infractions': 5, 'rpm_infractions': 14, 'fuel': 0.9979146586517686, 'consumed_fuel': 0.002085341348231351, 'accelerate_and_stop_infractions': 2, 'brake_infractions': 2, 'engine_idle_infractions': 2, 'score': 189}}, {'profile': 'aggressive', 'test_case': 'intersection', 'results': {'throttle_infractions': 5, 'rpm_infractions': 13, 'fuel': 0.9980280836595302, 'consumed_fuel': 0.0019719163404697815, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 2, 'engine_idle_infractions': 2, 'score': 182}}, {'profile': 'aggressive', 'test_case': 'intersection', 'results': {'throttle_infractions': 5, 'rpm_infractions': 13, 'fuel': 0.998016384328185, 'consumed_fuel': 0.001983615671815042, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 2, 'engine_idle_infractions': 2, 'score': 181}}, {'profile': 'aggressive', 'test_case': 'intersection', 'results': {'throttle_infractions': 5, 'rpm_infractions': 13, 'fuel': 0.9980011305385555, 'consumed_fuel': 0.0019988694614444524, 'accelerate_and_stop_infractions': 2, 'brake_infractions': 2, 'engine_idle_infractions': 2, 'score': 189}}, {'profile': 'aggressive', 'test_case': 'intersection', 'results': {'throttle_infractions': 5, 'rpm_infractions': 13, 'fuel': 0.9979998196937814, 'consumed_fuel': 0.002000180306218624, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 3, 'engine_idle_infractions': 2, 'score': 183}}, {'profile': 'aggressive', 'test_case': 'intersection', 'results': {'throttle_infractions': 5, 'rpm_infractions': 13, 'fuel': 0.9979406533374214, 'consumed_fuel': 0.002059346662578565, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 2, 'engine_idle_infractions': 3, 'score': 185}}, {'profile': 'aggressive', 'test_case': 'intersection', 'results': {'throttle_infractions': 5, 'rpm_infractions': 13, 'fuel': 0.998038329758477, 'consumed_fuel': 0.0019616702415230503, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 1, 'engine_idle_infractions': 2, 'score': 180}}, {'profile': 'aggressive', 'test_case': 'intersection', 'results': {'throttle_infractions': 5, 'rpm_infractions': 13, 'fuel': 0.9979963496072127, 'consumed_fuel': 0.0020036503927872706, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 2, 'engine_idle_infractions': 2, 'score': 181}}, {'profile': 'aggressive', 'test_case': 'intersection', 'results': {'throttle_infractions': 5, 'rpm_infractions': 13, 'fuel': 0.9980393399280959, 'consumed_fuel': 0.0019606600719040967, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 1, 'engine_idle_infractions': 2, 'score': 180}}, {'profile': 'aggressive', 'test_case': 'intersection', 'results': {'throttle_infractions': 5, 'rpm_infractions': 13, 'fuel': 0.9980210428021266, 'consumed_fuel': 0.0019789571978734433, 'accelerate_and_stop_infractions': 2, 'brake_infractions': 4, 'engine_idle_infractions': 2, 'score': 192}}, {'profile': 'default', 'test_case': 'intersection', 'results': {'throttle_infractions': 4, 'rpm_infractions': 13, 'fuel': 0.9985647777549784, 'consumed_fuel': 0.0014352222450215724, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 0, 'engine_idle_infractions': 3, 'score': 152}}, {'profile': 'default', 'test_case': 'intersection', 'results': {'throttle_infractions': 4, 'rpm_infractions': 13, 'fuel': 0.9985325108819288, 'consumed_fuel': 0.0014674891180711525, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 0, 'engine_idle_infractions': 3, 'score': 158}}, {'profile': 'default', 'test_case': 'intersection', 'results': {'throttle_infractions': 4, 'rpm_infractions': 13, 'fuel': 0.9985590208147206, 'consumed_fuel': 0.0014409791852794118, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 0, 'engine_idle_infractions': 3, 'score': 152}}, {'profile': 'default', 'test_case': 'intersection', 'results': {'throttle_infractions': 4, 'rpm_infractions': 13, 'fuel': 0.9985641979803186, 'consumed_fuel': 0.0014358020196814092, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 1, 'engine_idle_infractions': 4, 'score': 158}}, {'profile': 'default', 'test_case': 'intersection', 'results': {'throttle_infractions': 4, 'rpm_infractions': 13, 'fuel': 0.9985324100880666, 'consumed_fuel': 0.001467589911933409, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 0, 'engine_idle_infractions': 3, 'score': 158}}, {'profile': 'default', 'test_case': 'intersection', 'results': {'throttle_infractions': 4, 'rpm_infractions': 13, 'fuel': 0.9985615869063652, 'consumed_fuel': 0.0014384130936347939, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 0, 'engine_idle_infractions': 3, 'score': 152}}, {'profile': 'default', 'test_case': 'intersection', 'results': {'throttle_infractions': 4, 'rpm_infractions': 13, 'fuel': 0.9985608762553099, 'consumed_fuel': 0.0014391237446901295, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 0, 'engine_idle_infractions': 3, 'score': 153}}, {'profile': 'default', 'test_case': 'intersection', 'results': {'throttle_infractions': 4, 'rpm_infractions': 13, 'fuel': 0.9985648226569848, 'consumed_fuel': 0.001435177343015237, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 0, 'engine_idle_infractions': 3, 'score': 151}}, {'profile': 'default', 'test_case': 'intersection', 'results': {'throttle_infractions': 4, 'rpm_infractions': 13, 'fuel': 0.9985631565108956, 'consumed_fuel': 0.0014368434891044446, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 1, 'engine_idle_infractions': 3, 'score': 156}}, {'profile': 'default', 'test_case': 'intersection', 'results': {'throttle_infractions': 4, 'rpm_infractions': 13, 'fuel': 0.9985475083896972, 'consumed_fuel': 0.0014524916103028085, 'accelerate_and_stop_infractions': 1, 'brake_infractions': 0, 'engine_idle_infractions': 3, 'score': 160}}, {'profile': 'role_model', 'test_case': 'intersection', 'results': {'throttle_infractions': 0, 'rpm_infractions': 3, 'fuel': 0.9990454937146304, 'consumed_fuel': 0.0009545062853696384, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 2, 'score': 18}}, {'profile': 'role_model', 'test_case': 'intersection', 'results': {'throttle_infractions': 0, 'rpm_infractions': 2, 'fuel': 0.999026141967491, 'consumed_fuel': 0.0009738580325090451, 'accelerate_and_stop_infractions': 2, 'brake_infractions': 2, 'engine_idle_infractions': 3, 'score': 36}}, {'profile': 'role_model', 'test_case': 'intersection', 'results': {'throttle_infractions': 0, 'rpm_infractions': 3, 'fuel': 0.9990296961260898, 'consumed_fuel': 0.0009703038739101721, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 0, 'engine_idle_infractions': 2, 'score': 12}}, {'profile': 'role_model', 'test_case': 'intersection', 'results': {'throttle_infractions': 0, 'rpm_infractions': 3, 'fuel': 0.9990459883424689, 'consumed_fuel': 0.000954011657531062, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 2, 'score': 18}}, {'profile': 'role_model', 'test_case': 'intersection', 'results': {'throttle_infractions': 0, 'rpm_infractions': 3, 'fuel': 0.999046321738993, 'consumed_fuel': 0.0009536782610070071, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 1, 'score': 14}}, {'profile': 'role_model', 'test_case': 'intersection', 'results': {'throttle_infractions': 0, 'rpm_infractions': 5, 'fuel': 0.9990516488614013, 'consumed_fuel': 0.000948351138598702, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 1, 'score': 15}}, {'profile': 'role_model', 'test_case': 'intersection', 'results': {'throttle_infractions': 0, 'rpm_infractions': 3, 'fuel': 0.9990476048005302, 'consumed_fuel': 0.0009523951994697688, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 1, 'score': 14}}, {'profile': 'role_model', 'test_case': 'intersection', 'results': {'throttle_infractions': 0, 'rpm_infractions': 5, 'fuel': 0.9990514671874186, 'consumed_fuel': 0.0009485328125814485, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 2, 'score': 19}}, {'profile': 'role_model', 'test_case': 'intersection', 'results': {'throttle_infractions': 0, 'rpm_infractions': 5, 'fuel': 0.9990510156810033, 'consumed_fuel': 0.000948984318996704, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 2, 'score': 19}}, {'profile': 'role_model', 'test_case': 'intersection', 'results': {'throttle_infractions': 0, 'rpm_infractions': 3, 'fuel': 0.9990450151106745, 'consumed_fuel': 0.0009549848893255497, 'accelerate_and_stop_infractions': 0, 'brake_infractions': 1, 'engine_idle_infractions': 2, 'score': 18}}]
    visualize_results(res)

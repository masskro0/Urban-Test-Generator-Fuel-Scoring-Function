"""This file offers several plotting methods to visualize functions or roads."""

import matplotlib.pyplot as plt
import xml.etree.ElementTree as Etree
from os.path import join


def plotter(lanes):
    """Plots every point and lines between them. Used to visualize a road network.
    :param lanes: List of dicts containing multiple lanes.
    :return: Void.
    """
    for lane in lanes:
        x = []
        y = []
        control_points = lane.get("control_points")
        for point in control_points:
            x.append(point[0])
            y.append(point[1])
        plt.plot(x, y, '-og', markersize=8, linewidth=lane.get("width") / 6)
    plt.axis('scaled')
    plt.title('Road overview')
    plt.show()


def plot_all(population):
    """Plots a whole population. Method starts a new figure for every individual.
    :param population: Population with individuals in dict form containing another dict type called control_points.
    :return: Void.
    """
    for individual in population:
        plotter(individual.get("lanes"))


def plot_lines(lines):
    """Plots LineStrings of the package Shapely. Can be also used to plot other geometries. Show function
    must be called manually.
    :param lines: List of lines, e.g. LineStrings.
    :return: Void.
    """
    i = 0
    while i < len(lines):
        x, y = lines[i].xy
        plt.plot(x, y, '-og', markersize=3)
        i += 1
    # plt.show()


def plot_splines_and_width(width_lines, control_point_lines):
    """Plots connected control points with their width lines.
    :param width_lines: List of lines (e.g. LineStrings) which represent the width of a road segment.
    :param control_point_lines: List of connected control points (e.g. LineStrings).
    :return: Void.
    """
    for lane in width_lines:
        plot_lines(lane)
    for lane in control_point_lines:
        plot_lines(lane)
    plt.axis('scaled')
    plt.title('Road overview with width lines')
    plt.show()


def plot_splined_list(splined_list):
    """Plots a splined control point list of an individual.
    :param splined_list: List containing tuples of points.
    :return: Void.
    """
    for lane in splined_list:
        x = []
        y = []
        for point in lane:
            x.append(point[0])
            y.append(point[1])
        plt.plot(x, y, '-og', markersize=8, linewidth=lane.get("width") / 2)
    plt.axis('scaled')
    plt.title('Road overview')
    plt.show()


def plot_road_traffic_light(dbc, dbe, save_path=None, show=False):
    dbe_root = Etree.parse(dbe).getroot()
    dbc_root = Etree.parse(dbc).getroot()
    lanes = dbe_root.findall("lanes/lane")
    for lane in lanes:
        x = list()
        y = list()
        segments = lane.findall("laneSegment")
        width = 10
        for seg in segments:
            width = seg.attrib.get("width")
            x.append(float(seg.attrib.get("x")))
            y.append(float(seg.attrib.get("y")))
        plt.plot(x, y, '-og', markersize=1, linewidth=float(float(width)), label="Road")
    obstacles = dbe_root.find("obstacles")
    for obs in obstacles:
        if obs.tag.startswith("trafficlight") or obs.tag.endswith("sign"):
            plt.plot(float(obs.attrib.get("x")), float(obs.attrib.get("y")), markersize=12, marker='.', color="peru",
                     label="Traffic Sign/Light")
    participants = dbc_root.findall("participants/participant")
    for par in participants:
        if par.attrib.get("id") == "ego":
            init_state = par.find("initialState").attrib
            plt.plot(float(init_state.get("x")), float(init_state.get("y")), markersize=12, marker='>', color="r",
                     label="Ego Car")
    success_points = dbc_root.findall("success/scPosition")
    for sp in success_points:
        if sp.attrib.get("participant") == "ego":
            plt.plot(float(sp.attrib.get("x")), float(sp.attrib.get("y")), markersize=12, marker='.', color="b",
                     label="Success Point")
    tod = abs((float(dbe_root.find("timeOfDay").text) - 0.5) * 2)
    plt.gca().set_facecolor(str(tod))
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc="best")
    plt.title('Road overview')
    if show:
        plt.show()
    if save_path is not None:
        plt.savefig(join(save_path, "road_network.png"), bbox_inches='tight')
    plt.clf()

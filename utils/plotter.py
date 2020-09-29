"""This file offers several plotting methods to visualize functions or roads."""

import matplotlib.pyplot as plt
import xml.etree.ElementTree as Etree
from os.path import join


def plotter(roads, markersize=8, color="-og", marker=True, linewidth=None, title="Road Overview", show=True, dpi=200,
            save_path=None):
    """Plots every point and lines between them. Used to visualize a road network.
    :param save_path: Path where the plot should be saved. If none is given, the images won't be saved.
    :param dpi: Resolution of the image.
    :param show: {@code True} shows the image.
    :param title: Title of the image.
    :param linewidth: Width of polylines.
    :param marker: {@code True} to show the points.
    :param color: Color of the polylines.
    :param markersize: Size of the points.
    :param roads: List of dicts containing multiple roads.
    :return: Void.
    """
    for road in roads:
        x = []
        y = []
        control_points = road.get("control_points")
        for point in control_points:
            x.append(point[0])
            y.append(point[1])
        plt.plot(x, y, color=color, marker="o" if marker else None, markersize=markersize,
                 linewidth=road.get("width") / 6 if linewidth is None else linewidth)
    plt.axis('scaled')
    if title is not None and title != "":
        plt.title(title)
    if save_path is not None:
        plt.tight_layout()
        plt.savefig(save_path, dpi=dpi)
    if show:
        plt.show()


def plot_all(population):
    """Plots a whole population. Method starts a new figure for every individual.
    :param population: Population with individuals in dict form containing another dict type called control_points.
    :return: Void.
    """
    for individual in population:
        plotter(individual.get("roads"))


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
    for road in width_lines:
        plot_lines(road)
    for road in control_point_lines:
        plot_lines(road)
    plt.axis('scaled')
    plt.title('Road overview with width lines')
    plt.show()


def plot_splined_list(splined_list):
    """Plots a splined control point list of an individual.
    :param splined_list: List containing tuples of points.
    :return: Void.
    """
    for road in splined_list:
        x = []
        y = []
        for point in road:
            x.append(point[0])
            y.append(point[1])
        plt.plot(x, y, '-og', markersize=8, linewidth=road.get("width") / 2)
    plt.axis('scaled')
    plt.title('Road overview')
    plt.show()


def plot_road_traffic_light(dbc, dbe, save_path=None, show=False):
    """Plots a whole road network out of XML files. This includes ego-car position, success point, traffic lights and
     signs and time of day.
    :param dbc: Path to criteria XML file.
    :param dbe: Path to environment XML file.
    :param save_path: Path where the image should be saved.
    :param show: {@code True} to show the image.
    :return: Void.
    """
    dbe_root = Etree.parse(dbe).getroot()
    dbc_root = Etree.parse(dbc).getroot()
    roads = dbe_root.findall("roads/road")
    for road in roads:
        # Plot roads.
        x = list()
        y = list()
        segments = road.findall("roadSegment")
        width = 10
        for seg in segments:
            width = seg.attrib.get("width")
            x.append(float(seg.attrib.get("x")))
            y.append(float(seg.attrib.get("y")))
        plt.plot(x, y, '-og', markersize=1, linewidth=float(float(width)), label="Road")
    obstacles = dbe_root.find("obstacles")
    if obstacles is None:
        obstacles = list()
    for obs in obstacles:
        # Plot obstacles
        if obs.tag.startswith("trafficlight") or obs.tag.endswith("sign"):
            plt.plot(float(obs.attrib.get("x")), float(obs.attrib.get("y")), markersize=12, marker='.', color="peru",
                     label="Traffic Sign/Light")
    participants = dbc_root.findall("participants/participant")
    for par in participants:
        # Plot ego-car position.
        if par.attrib.get("id") == "ego":
            init_state = par.find("initialState").attrib
            plt.plot(float(init_state.get("x")), float(init_state.get("y")), markersize=12,
                     marker=(3, 0, float(init_state.get("orientation")) - 90), color="r", label="Ego Car")
    success_points = dbc_root.findall("success/scPosition")
    for sp in success_points:
        # Plot success states.
        if sp.attrib.get("participant") == "ego":
            plt.plot(float(sp.attrib.get("x")), float(sp.attrib.get("y")), markersize=12, marker='.', color="b",
                     label="Success Point")
    tod = abs((float(dbe_root.find("timeOfDay").text) - 0.5) * 2)
    plt.gca().set_facecolor(str(tod))  # Plot time of day.
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc="best")
    plt.title('Road overview')
    if show:
        plt.show()
    if save_path is not None:
        plt.tight_layout()
        plt.savefig(join(save_path, "road_network.png"), dpi=200)
    plt.clf()

"""This file offers several plotting methods to visualize functions or roads."""

import matplotlib.pyplot as plt


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
        plt.plot(x, y, '-og', markersize=8, linewidth=lane.get("width")/6)
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
    """Plots LineStrings of the package shapely. Can be also used to plot other geometries. Show function
    must be called manually.
    :param lines: List of lines, e.g. LineStrings.
    :return: Void.
    """
    iterator = 0
    while iterator < len(lines):
        x, y = lines[iterator].xy
        plt.plot(x, y, '-og', markersize=3)
        iterator += 1
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
        plt.plot(x, y, '-og', markersize=8, linewidth=lane.get("width")/2)
    plt.axis('scaled')
    plt.title('Road overview')
    plt.show()

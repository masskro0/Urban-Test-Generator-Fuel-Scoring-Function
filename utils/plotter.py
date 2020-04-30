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
            x.append(point.get("x"))
            y.append(point.get("y"))
        plt.plot(x, y, '-og', markersize=6, linewidth=control_points[0].get("width"))
    plt.axis('scaled')
    plt.title('Road overview')
    plt.show()


def plot_all(population):
    """Plots a whole population. Method starts a new figure for every individual.
    :param population: Population with individuals in dict form containing another dict type called control_points.
    :return: Void
    """
    iterator = 0
    while iterator < len(population):
        plotter(population[iterator].get("lanes"))
        iterator += 1


def plot_lines(lines):
    """Plots LineStrings of the package shapely. Can be also used to plot other geometries.
    :param lines: List of lines, e.g. LineStrings
    :return: Void
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
    plot_lines(width_lines)
    plot_lines(control_point_lines)
    plt.show()

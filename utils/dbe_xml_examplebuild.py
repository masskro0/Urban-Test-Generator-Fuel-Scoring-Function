"""This example dbe builder class demonstrates how to use the methods of the dbe builder to create
an environment xml file.
"""

from utils.dbe_xml_builder import DBEBuilder, save_xml
from os.path import abspath, exists
from os import mkdir, getcwd

dbe = DBEBuilder()

segment1 = (0, 0)
segment2 = (50, 0)
segment3 = (80, 20)
segment4 = (100, 20)
width = 12
segments = [segment1, segment2, segment3, segment4]

dbe.add_road(segments, width=width, left_lanes=1, right_lanes=2)

cone = {"name": "cone",
        "position": (5, 5),
        "baseRadius": 5,
        "height": 5}
cylinder = {"name": "cylinder",
            "position": (10, 10),
            "radius": 2,
            "height": 2}
obstacles = [cone, cylinder]

dbe.add_obstacles(obstacles)

scenario = getcwd() + "\\scenario"
if not exists(scenario):
    mkdir(scenario)

dbe.set_tod(0)

save_xml("exampleXML", dbe.root, "environment", abspath("scenario"))

"""This example dbe builder class demonstrates how to use the methods of the dbe builder to create
an environment xml file.
"""

from utils.dbe_xml_builder import DBEBuilder
import os

dbe = DBEBuilder()

segment1 = [0, 0, 12]
segment2 = [50, 0, 12]
segment3 = [80, 20, 12]
segment4 = [100, 20, 12]
segments = [segment1, segment2, segment3, segment4]

dbe.add_lane(segments, left_lanes=1, right_lanes=2)

cone = {"name": "cone",
        "x": 5,
        "y": 5,
        "baseRadius": 5,
        "height": 5}
cylinder = {"name": "cylinder",
            "x": 10,
            "y": 10,
            "radius": 2,
            "height": 2}
obstacles = [cone, cylinder]

dbe.add_obstacles(obstacles)

scenario = os.getcwd() + "\\scenario"
if not os.path.exists(scenario):
    os.mkdir(scenario)

dbe.save_xml("exampleXML")

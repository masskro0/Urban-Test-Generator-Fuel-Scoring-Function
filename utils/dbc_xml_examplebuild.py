"""This example dbc builder class demonstrates how to use the methods of the dbc builder to create
a criteria xml file.
"""

from utils.dbc_xml_builder import DBCBuilder
from utils.dbe_xml_builder import save_xml
from os.path import exists, abspath
from os import mkdir, getcwd

dbc = DBCBuilder()

dbc.define_name("Example Test")
dbc.environment_name("exampleXML")

init_state = {"position": (0, 4),
              "orientation": 0}
waypoint1 = {"position": (15, 4),
             "tolerance": 4}
waypoint2 = {"position": (61, 4),
             "tolerance": 5}
waypoints = [waypoint1, waypoint2]
participant_id = "ego"
model = "ETK800"
color = "black"
participant = {"init_state": init_state,
               "waypoints": waypoints,
               "model": model,
               "id": participant_id,
               "color": color}
dbc.add_car(participant=participant)

success_points = [{"id": participant_id,
                   "position": waypoint2.get("position"),
                   "tolerance": waypoint2.get("tolerance")}]
dbc.add_success_point(participant_id=participant_id, success_point=success_points[0])

dbc.add_failure_conditions(participant_id)

scenario = getcwd() + "\\scenario"
if not exists(scenario):
    mkdir(scenario)

save_xml("exampleXML", dbc.root, "criteria", abspath("scenario"))

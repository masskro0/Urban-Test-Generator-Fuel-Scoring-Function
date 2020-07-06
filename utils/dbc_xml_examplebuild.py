"""This example dbc builder class demonstrates how to use the methods of the dbc builder to create
a criteria xml file.
"""

from utils.dbc_xml_builder import DBCBuilder
from utils.dbe_xml_builder import save_xml
import os

dbc = DBCBuilder()

dbc.define_name("Example Test")
dbc.environment_name("exampleXML")
dbc.steps_per_second(60)
dbc.ai_freq(6)

init_state = {"position": (0, 4),
              "orientation": 0,
              "movementMode": "MANUAL",
              "speed": 50}
waypoint1 = {"position": (15, 4),
             "tolerance": 4,
             "movementMode": "_BEAMNG",
             "speed": 40}
waypoint2 = {"position": (61, 4),
             "tolerance": 5,
             "movementMode": "AUTONOMOUS"}
waypoints = [waypoint1, waypoint2]
participant_id = "ego"
model = "ETK800"
participant = {"init_state": init_state,
               "waypoints": waypoints,
               "model": model,
               "id": participant_id}
dbc.add_car(participant=participant)

vc_pos = {"id": participant_id,
          "position": waypoints[1].get("position"),
          "tolerance": waypoint1.get("tolerance")}
sc_speed = 15
dbc.add_precond_partic_sc_speed(vc_pos, sc_speed)

success_points = [{"id": participant_id,
                   "position": waypoint2.get("position"),
                   "tolerance": waypoint2.get("tolerance")}]
dbc.add_success_point(participant_id=participant_id, success_point=success_points[0].get("position"))

dbc.add_failure_conditions(participant_id, "offroad")

scenario = os.getcwd() + "\\scenario"
if not os.path.exists(scenario):
    os.mkdir(scenario)

save_xml("exampleXML", dbc.root, "criteria")

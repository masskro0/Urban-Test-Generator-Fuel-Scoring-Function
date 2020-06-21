from beamngpy import BeamNGpy, Scenario, Road
from beamngpy.beamngcommon import ENV
from os.path import join


class EnvironmentCreator:

    def __init__(self, dbe_root, index):
        self.dbe_root = dbe_root
        self.index = index

    def _init_prefab(self):
        self.bng = BeamNGpy('localhost', 64255)
        self.scenario = Scenario('urban', 'urban_{}'.format(self.index))

    def _finalize_prefab(self):
        self.scenario.make(self.bng)
        self._change_object_options()

    def add_to_prefab(self):
        self._init_prefab()
        self._add_lanes()
        # self._add_obstacles()
        self._finalize_prefab()

    def _add_lanes(self):
        lanes = self.dbe_root.findall("lanes/lane")
        rid = 0
        for lane in lanes:
            self._add_road(lane, rid)
            self._add_lane_markings(lane, rid)
            rid += 1

    def _add_road(self, lane, rid):
        road = Road(material='road_rubber_sticky', rid='road_{}'.format(rid), interpolate=True, render_priority=10,
                    texture_length=2.5, drivability=1)
        nodes = []
        road_segments = lane.findall("laneSegment")
        for segment in road_segments:
            d = segment.attrib
            z = 0.01 if d.get("z") is None else d.get("z")
            nodes.append((d.get("x"), d.get("y"), z, d.get("width")))
        road.nodes.extend(nodes)
        self.scenario.add_road(road)

    def _add_lane_markings(self, lane, rid):
        # renderpriority = 9 , texture_length = 16, drivability = -1
        pass

    def _change_object_options(self):
        prefab_path = join(ENV["BNG_HOME"], "levels", "urban", "scenarios", "urban_{}.prefab".format(self.index))
        prefab_file = open(prefab_path, "r")
        original_content = prefab_file.readlines()
        prefab_file.close()
        new_content = list()
        for line in original_content:
            if "overObjects" in line:
                line = line.replace("0", "1")
                new_content.append("        annotation = \"STREET\";\n")
            if "distanceFade" in line:
                line = ""
            if "Material" in line:
                if "road_rubber_sticky" in line:
                    new_content.append("        distanceFade = \"1000 1000\";\n")
                else:
                    new_content.append("        distanceFade = \"0 0\";\n")
            new_content.append(line)
        prefab_file = open(prefab_path, "w")
        prefab_file.writelines(new_content)
        prefab_file.close()

    def _add_obstacles(self):
        raise NotImplementedError()

from beamngpy import BeamNGpy, Scenario, Road
from beamngpy.beamngcommon import ENV
from os.path import join

from shapely.geometry import LineString


class EnvironmentCreator:

    def __init__(self, dbe_root, index):
        self.dbe_root = dbe_root
        self.index = index

    def _init_prefab(self):
        self.bng = BeamNGpy('localhost', 64255)
        self.scenario = Scenario('urban', 'urban_{}'.format(self.index))

    def _finalize_prefab(self):
        self.bng.user = None
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
            if bool(lane.attrib.get("markings")):
                self._add_lane_markings(lane, rid)
            rid += 1

    def _add_road(self, lane, rid):
        road = Road(material='road_rubber_sticky', rid='road_{}'.format(rid), interpolate=True, texture_length=2.5,
                    drivability=1)
        nodes = []
        road_segments = lane.findall("laneSegment")
        for segment in road_segments:
            d = segment.attrib
            z = 0.01 if d.get("z") is None else d.get("z")
            nodes.append((d.get("x"), d.get("y"), z, d.get("width")))
        road.nodes.extend(nodes)
        self.scenario.add_road(road)

    def _add_lane_markings(self, lane, rid):
        linestring_nodes = []
        road_segments = lane.findall("laneSegment")
        for segment in road_segments:
            d = segment.attrib
            linestring_nodes.append((float(d.get("x")), float(d.get("y"))))
        line = LineString(linestring_nodes)
        outer_offset = int(road_segments[0].attrib.get("width")) / 2 - 0.4
        self._add_outer_marking(road_segments, rid, line, outer_offset, "left")
        self._add_outer_marking(road_segments, rid, line, outer_offset, "right")
        if lane.attrib.get("leftLanes") != 0 and lane.attrib.get("rightLanes") != 0:
            self._add_yellow_divider_line(road_segments, rid, line, int(lane.attrib.get("leftLanes")),
                                          int(lane.attrib.get("rightLanes")))

    def _add_outer_marking(self, road_segments, rid, line, offset, direction):
        nodes = []
        outer_line = line.parallel_offset(offset, direction)
        fac = len(road_segments) / len(list(outer_line.coords))
        iterator = 0
        while iterator < len(list(outer_line.coords)):
            z = road_segments[int(round(fac * iterator))].attrib.get("z")
            if z is None:
                z = 0.01
            else:
                z = (int(z))
            nodes.append((list(outer_line.coords)[iterator][0], list(outer_line.coords)[iterator][1], z, 0.2))
            iterator += 1
        nodes = nodes[::-1]
        road = Road(material='line_white', rid='road_{}_{}_line'.format(rid, direction), interpolate=False,
                    texture_length=16, drivability=-1)
        road.nodes.extend(nodes)
        self.scenario.add_road(road)

    def _add_yellow_divider_line(self, road_segments, rid, line, left_lanes, right_lanes):
        mid = int(road_segments[0].attrib.get("width")) / 2
        lane_width = int(road_segments[0].attrib.get("width")) / (left_lanes + right_lanes)
        nodes = []
        if left_lanes == right_lanes:
            for node in road_segments:
                d = node.attrib
                z = 0.01 if d.get("z") is None else d.get("z")
                nodes.append((d.get("x"), d.get("y"), z, 0.3))
        else:
            if left_lanes > right_lanes:
                offset = left_lanes * lane_width - mid
                direction = "right"
            elif left_lanes < right_lanes:
                offset = right_lanes * lane_width - mid
                direction = "left"
            else:
                raise TypeError("leftLanes and rightLanes must be Integers.")
            divider_line = line.parallel_offset(offset, direction)
            fac = len(road_segments) / len(list(divider_line.coords))
            iterator = 0
            while iterator < len(list(divider_line.coords)):
                z = road_segments[int(round(fac * iterator))].attrib.get("z")
                if z is None:
                    z = 0.01
                else:
                    z = (int(z))
                nodes.append((list(divider_line.coords)[iterator][0], list(divider_line.coords)[iterator][1], z, 0.3))
                iterator += 1
            nodes = nodes[::-1]
        road = Road(material='line_yellow_double', rid='road_{}_left_right_divider'.format(rid), interpolate=False,
                    texture_length=16, drivability=-1)
        road.nodes.extend(nodes)
        self.scenario.add_road(road)

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
            if "breakAngle" in line:
                line = "        breakAngle = \"3\";\n"
            if "renderPriority" in line:
                line = ""
            if "distanceFade" in line:
                line = ""
            if "Material" in line:
                if "road_rubber_sticky" in line:
                    new_content.append("        distanceFade = \"1000 1000\";\n")
                    new_content.append("        renderPriority = \"10\";\n")
                else:
                    new_content.append("        distanceFade = \"0 0\";\n")
                    new_content.append("        renderPriority = \"9\";\n")
            new_content.append(line)
        prefab_file = open(prefab_path, "w")
        prefab_file.writelines(new_content)
        prefab_file.close()

    def _add_obstacles(self):
        raise NotImplementedError()

    # TODO Add own interpolation
    # TODO Add input checking

class EnvironmentCreator:

    def __init__(self, dbe_root, index):
        self.dbe_root = dbe_root
        self.index = index
        self._init_prefab()

    def _init_prefab(self):
        open("urban_{}.prefab".format(str(self.index)), "w")

    def add_to_prefab(self):
        self._add_lanes()
        # self._add_obstacles()

    def _add_lanes(self):
        lanes = self.dbe_root.findall("lanes/lane")
        for lane in lanes:
            print(lane.attrib)
            print(lane.findall("laneSegment")[0].attrib)

    def _add_lane_markings(self):
        raise NotImplementedError()

    def _add_obstacles(self):
        raise NotImplementedError()

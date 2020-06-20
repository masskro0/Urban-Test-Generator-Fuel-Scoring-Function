class CriteriaConverter:

    def __init__(self, dbc_root, index):
        self.dbc_root = dbc_root
        self.index = index

    def add_to_prefab(self):
        raise NotImplementedError()

    def _add_participants(self):
        raise NotImplementedError()

    def _add_waypoints(self, participant):
        raise NotImplementedError()

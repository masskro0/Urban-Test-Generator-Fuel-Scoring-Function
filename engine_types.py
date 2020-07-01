from enum import Enum, auto


class EngineType(Enum):
    """Enum class for the engine/fuel type of a car."""
    PETROL = auto()
    DIESEL = auto()
    ELECTRIC = auto()
    GAS = auto()

    def get_rpm_shifting_sweetspots(self):
        """Returns the upper and lower rpm limits where a gear should be shifted, depending on the engine/fuel type."""
        if self.name is "PETROL":
            return {"lower_limit": 1300,
                    "upper_limit": 2500}
        elif self.name is "DIESEL":
            return {"lower_limit": 1100,
                    "upper_limit": 2000}
        elif self.name is "ELECTRIC":
            return None
        elif self.name is "GAS":
            return {"lower_limit": 1100,
                    "upper_limit": 2000}

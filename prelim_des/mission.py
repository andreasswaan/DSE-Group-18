from typing import Literal


class MissionPhase:
    loitering_time: float
    payload_mass: float
    TO_speed: float
    LD_speed: float
    distance: float

    def __init__(
        self,
        TO_str: str,
        LD_str: str,
        loitering_time: float,
        payload_mass: float,
        TO_speed: float,
        LD_speed: float,
        distance: float,
    ):
        self.TO_str = TO_str
        self.LD_str = LD_str
        self.loitering_time = loitering_time
        self.payload_mass = payload_mass
        self.TO_speed = TO_speed
        self.LD_speed = LD_speed
        self.distance = distance

    @property
    def dict(self):
        return {
            "TO": self.TO_str,
            "LD": self.LD_str,
            "cruise_distance:": self.distance,
            "loitering_time": self.loitering_time,
            "payload_mass": self.payload_mass,
            "TO_speed": self.TO_speed,
            "LD_speed": self.LD_speed,
            "cruise_h": 200,
            "cruise_speed": 15,
        }


class Mission:
    """Class for a mission."""

    phases_str: str  # Attribute type annotation
    phases: list[MissionPhase]  # Attribute type annotation
    # loitering_times only contains keys "R", "C", and "D" (all str), values are int (seconds)
    loitering_times: dict[Literal["R", "C", "D"], float]
    payload_masses: dict[Literal["DR", "RC", "CC", "CD", "CR"], float]
    TO_speed: dict[Literal["R", "C", "D"], float]
    LD_speed: dict[Literal["R", "C", "D"], float]
    distances: dict[Literal["DR", "RC", "CC", "CD"], float]

    def __init__(
        self,
        phases_str: str,
        loitering_times: dict[Literal["R", "C", "D"], float],
        payload_masses: dict[Literal["DR", "RC", "CC", "CD", "CR"], float],
        TO_speed: dict[Literal["R", "C", "D"], float],
        LD_speed: dict[Literal["R", "C", "D"], float],
        distances: dict[Literal["DR", "RC", "CC", "CD"], float],
    ):
        self.phases_str = phases_str
        self.loitering_times = loitering_times
        self.payload_masses = payload_masses
        self.TO_speed = TO_speed
        self.LD_speed = LD_speed
        self.distances = distances
        self.phases = self.__phases_obj()

    def __phases_obj(self) -> list[MissionPhase]:
        phases_str = self.phases_str
        phases = []
        for i in range(len(phases_str)):
            if i == len(phases_str) - 1:
                continue
            TO_letter = phases_str[i]
            LD_letter = phases_str[i + 1]
            phases.append(
                MissionPhase(
                    TO_letter,
                    LD_letter,
                    self.loitering_times[LD_letter],
                    self.__payload_mass(TO_letter, LD_letter),
                    self.TO_speed[TO_letter],
                    self.LD_speed[LD_letter],
                    self.__distance(TO_letter, LD_letter),
                )
            )
        return phases

    @property
    def phases_dict(self) -> list[MissionPhase.dict]:
        phases_dict = []
        for phase in self.phases:
            phases_dict.append(phase.dict)
        return phases_dict

    def __payload_mass(self, TO_str: str, LD_str: str):
        match TO_str + LD_str:
            case "DR" | "RD":
                return self.payload_masses["DR"]
            case "RC" | "CR":
                return self.payload_masses["RC"]
            case "CC":
                return self.payload_masses["CC"]
            case "CD" | "DC":
                return self.payload_masses["CD"]
            case _:
                raise ValueError("Invalid mission phase")

    def __distance(self, TO_str: str, LD_str: str):
        match TO_str + LD_str:
            case "DR" | "RD":
                return self.distances["DR"]
            case "RC" | "CR":
                return self.distances["RC"]
            case "CC":
                return self.distances["CC"]
            case "CD" | "DC":
                return self.distances["CD"]
            case _:
                raise ValueError("Invalid mission phase")

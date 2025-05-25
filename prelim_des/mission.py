from typing import Literal
from utils.import_toml import load_toml
from utils.unit_converter import ImperialConverter

toml = load_toml()


class MissionPhase:
    loitering_time: float
    payload_mass: float
    TO_speed: float
    LD_speed: float
    distance: float
    cruise_h: float
    cruise_speed: float
    TO_time: float
    LD_time: float

    def __init__(
        self,
        TO_str: str,
        LD_str: str,
        loitering_time: float,
        payload_mass: float,
        TO_speed: float,
        LD_speed: float,
        distance: float,
        cruise_h: float = ImperialConverter.len_ft_m(toml["config"]["mission"]["cruise_height"]),
        cruise_speed: float = toml["config"]["mission"]["cruise_speed"],
    ):
        self.TO_str = TO_str
        self.LD_str = LD_str
        self.loitering_time = loitering_time
        self.payload_mass = payload_mass
        self.TO_speed = TO_speed
        self.LD_speed = LD_speed
        self.distance = distance
        self.cruise_h = cruise_h
        self.cruise_speed = cruise_speed
        self.TO_time = cruise_h / TO_speed
        self.LD_time = cruise_h / LD_speed


class Mission:
    """Class for a mission"""

    phases_str: str  # Attribute type annotation
    phases: list[MissionPhase]  # Attribute type annotation
    loitering_times: dict[Literal["R", "C", "D"], float]
    payload_masses: dict[Literal["DR", "RC", "CC", "CD", "CR"], float]
    TO_speed: dict[Literal["R", "C", "D"], float]
    LD_speed: dict[Literal["R", "C", "D"], float]
    distances: dict[Literal["DR", "RC", "CC", "CD"], float]

    def __init__(
        self,
        phases_str: str,
    ):
        self.phases_str = phases_str
        self.loitering_times = toml["config"]["mission"]["loitering_times"]
        self.payload_masses = toml["config"]["mission"]["payload_masses"]
        self.TO_speed = toml["config"]["mission"]["TO_speed"]
        self.LD_speed = toml["config"]["mission"]["LD_speed"]
        self.distances = toml["config"]["mission"]["distances"]
        self.phases = self.legs_obj

    @property
    def legs_obj(self) -> list[MissionPhase]:
        """Recalculates the mission phases (to be used when a mission reset is needed)"""
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

    @property
    def legs_dict(self) -> list[MissionPhase.__dict__]:  # type: ignore
        phases_dict = []
        for phase in self.phases:
            phases_dict.append(phase.__dict__)
        return phases_dict

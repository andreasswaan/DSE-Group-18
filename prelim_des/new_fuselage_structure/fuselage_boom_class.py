from prelim_des.drone import Drone
from typing import TYPE_CHECKING, Literal
from prelim_des.constants import g

from prelim_des.new_wing_structure.material_class import Material
import numpy as np

if TYPE_CHECKING:
    from prelim_des.new_fuselage_structure.fuselage_section_class import FuselageSection


class FuselageBoom:
    normal_stress: float
    "stress must be in Pa (N/m^2)"
    shear_flow_delta: float
    "delta shear (N/m)"
    y_pos: float
    "distance must be in m"
    z_pos: float
    "distance must be in m"
    area: float
    "area must be in m^2"
    section: "FuselageSection"

    material: "Material"

    def __init__(
        self,
        y_pos,
        z_pos,
        area=None,
        type: Literal["spar", "stringer"] = "stringer",
        id=None,
    ):
        self.y_pos = y_pos
        self.z_pos = z_pos
        self.area = area
        self.section = None
        self.type = type
        self.failure_method = None
        self.id = id
        self.type = type
        pass

    @property
    def normal_stress(self):
        Ixx = self.section.Ixx
        Izz = self.section.Izz
        Ixz = self.section.Ixz
        denominator = Ixx * Izz - Ixz**2
        numerator_1 = (
            self.section.analysis_moment_y * Izz - self.section.analysis_moment_z * Ixz
        ) * self.z_pos
        numerator_2 = (
            self.section.analysis_moment_z * Ixx - self.section.analysis_moment_y * Ixz
        ) * self.y_pos
        normal_stress = (numerator_1 + numerator_2) / denominator
        total_area = sum(boom.area for boom in self.section.booms)
        normal_force = self.section.normal_force
        normal_force_stress = normal_force / (total_area) * self.area / total_area
        return normal_stress + normal_force_stress

    @property
    def shear_flow_delta(self):
        Ixx = self.section.Ixx
        Izz = self.section.Izz
        Ixz = self.section.Ixz
        denominator = Ixx * Izz - Ixz**2

        numerator_1 = (
            -(self.section.analysis_shear_z * Izz - self.section.analysis_shear_y * Ixz)
            * self.z_pos
            * self.area
        )
        numerator_2 = (
            -(self.section.analysis_shear_y * Ixx - self.section.analysis_shear_z * Ixz)
            * self.y_pos
            * self.area
        )

        shear_flow_delta = (abs(numerator_1) + abs(numerator_2)) / denominator

        return shear_flow_delta

    @property
    def radius(self):
        return np.sqrt(self.area / np.pi)

    def weight(self, length):
        """Return the weight in N"""
        return self.area * length * self.material.density * g

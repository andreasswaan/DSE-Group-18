
from typing import TYPE_CHECKING, Literal
from prelim_des.constants import g
from prelim_des.new_wing_structure.material_class import Material
import numpy as np

if TYPE_CHECKING:
    from prelim_des.new_wing_structure.wing_section_class import WingSection


class WingBoom:
    normal_stress: float
    "stress must be in Pa (N/m^2)"
    shear_flow_delta: float
    "delta shear (N/m)"
    x_pos: float
    "distance must be in m"
    z_pos: float
    "distance must be in m"
    area: float
    "area must be in m^2"
    section: "WingSection"

    material: "Material"

    def __init__(
        self,
        x_pos,
        z_pos,
        area=None,
        type: Literal["spar", "stringer"] = "stringer",
        id=None,
    ):
        self.x_pos = x_pos
        self.z_pos = z_pos
        self.area = area
        self.section = None
        self.type = type
        self.failure_method = ""
        self.id = id
        pass

    @property
    def normal_stress(self):
        Ixx = self.section.Ixx
        Izz = self.section.Izz
        Ixz = self.section.Ixz
        denominator = Ixx * Izz - Ixz**2
        numerator_1 = (
            self.section.analysis_moment_x * Izz - self.section.analysis_moment_z * Ixz
        ) * self.z_pos
        numerator_2 = (
            self.section.analysis_moment_z * Ixx - self.section.analysis_moment_x * Ixz
        ) * self.x_pos
        normal_stress = (numerator_1 + numerator_2) / denominator

        return normal_stress

    @property
    def shear_flow_delta(self):
        Ixx = self.section.Ixx
        Izz = self.section.Izz
        Ixz = self.section.Ixz
        denominator = Ixx * Izz - Ixz**2

        numerator_1 = (
            -(self.section.analysis_shear_z * Izz - self.section.analysis_shear_x * Ixz)
            * self.z_pos
            * self.area
        )
        numerator_2 = (
            -(self.section.analysis_shear_x * Ixx - self.section.analysis_shear_z * Ixz)
            * self.x_pos
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

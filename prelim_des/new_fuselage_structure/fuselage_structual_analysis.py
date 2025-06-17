# from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from prelim_des.drone import Drone
    from prelim_des.mission import Mission
    

from prelim_des.new_fuselage_structure.fuselage_boom_class import FuselageBoom
from prelim_des.new_fuselage_structure.fuselage_section_class import FuselageSection
from prelim_des.new_fuselage_structure.utils.thickness_to_stress_estimation import (
    calculate_stress_ansys,
)
from prelim_des.new_wing_structure.wing_boom_class import WingBoom
from prelim_des.new_wing_structure.wing_section_class import WingSection
from prelim_des.new_wing_structure.wing_structural_analysis import (
    WingStructuralAnalysis,
)
from prelim_des.constants import g
from prelim_des.performance import Performance
from prelim_des.utils.arrows import Arrow3D
from prelim_des.utils.import_toml import load_toml
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
import numpy as np
from prelim_des.new_wing_structure.material_class import Material

toml = load_toml()


class FuselageStructuralAnalysis:
    """ASSUMPTION: torsion is neglected for now"""

    """ASSUMPTION: it is perfectly rectangle"""

    sections = list["FuselageSection"]

    def __init__(
        self,
        drone,
        material: Material,
    ):
        self.drone = drone
        self.material = material
        self.wing_analysis: WingStructuralAnalysis = None
        self.sections: list[FuselageSection] = []
        self.amount_of_booms_horizontal = toml["config"]["structures"]["fuselage"][
            "amount_of_booms_horizontal"
        ]
        self.amount_of_booms_vertical = toml["config"]["structures"]["fuselage"][
            "amount_of_booms_vertical"
        ]

        self.start_spar_boom_area = toml["config"]["structures"]["fuselage"][
            "start_spar_boom_area"
        ]

        self.start_stringer_boom_area = toml["config"]["structures"]["fuselage"][
            "start_stringer_boom_area"
        ]

        """Start Section"""
        self.amount_of_sections_start = toml["config"]["structures"]["fuselage"][
            "amount_of_sections_start"
        ]
        self.length_start_section = toml["config"]["structures"]["fuselage"][
            "length_start_section"
        ]
        self.height_start_section = toml["config"]["structures"]["fuselage"][
            "height_start_section"
        ]
        self.width_start_section = toml["config"]["structures"]["fuselage"][
            "width_start_section"
        ]

        """Middle Section"""
        self.amount_of_sections_middle = toml["config"]["structures"]["fuselage"][
            "amount_of_sections_middle"
        ]
        self.length_middle_section = toml["config"]["structures"]["fuselage"][
            "length_middle_section"
        ]
        self.height_middle_section = toml["config"]["structures"]["fuselage"][
            "height_middle_section"
        ]
        self.width_middle_section = toml["config"]["structures"]["fuselage"][
            "width_middle_section"
        ]

        """End Section"""
        self.amount_of_sections_end = toml["config"]["structures"]["fuselage"][
            "amount_of_sections_end"
        ]
        self.length_end_section = toml["config"]["structures"]["fuselage"][
            "length_end_section"
        ]
        self.height_end_section = toml["config"]["structures"]["fuselage"][
            "height_end_section"
        ]
        self.width_end_section = toml["config"]["structures"]["fuselage"][
            "width_end_section"
        ]

        self.wing_to_fuselage_root_chord_ratio = toml["config"]["structures"][
            "fuselage"
        ]["wing_to_fuselage_root_chord_ratio"]

        self.safety_factor = toml["config"]["structures"]["fuselage"]["safety_factor"]

        self.web_thickness = toml["config"]["structures"]["fuselage"]["web_thickness"]
        self.propeller_rod_length = toml["config"]["structures"]["fuselage"][
            "propeller_rod_length"
        ]

        self.wing_pos = (
            -self.length_middle_section / 2 - self.length_start_section
        )  # ASSUMPTION

    def make_fuselage_structure(self):
        start_length_fraction = (
            self.length_start_section / self.amount_of_sections_start
        )
        start_width_taper = (
            self.width_middle_section - self.width_start_section
        ) / self.length_start_section

        start_height_taper = (
            self.height_middle_section - self.height_start_section
        ) / self.length_start_section
        for i in range(self.amount_of_sections_start):

            section = FuselageSection(
                self.drone, f"Fuselage Section Start {i}", length=start_length_fraction
            )
            section.x_pos = -i * start_length_fraction
            self.sections.append(section)

            width = (
                self.width_start_section + start_width_taper * i * start_length_fraction
            )
            height = (
                self.height_start_section
                + start_height_taper * i * start_length_fraction
            )

            boom_1 = FuselageBoom(
                width / 2, -height / 2, type="spar"
            )  # corner boom top left
            section.add_boom(boom_1)
            width_fraction = width / self.amount_of_booms_horizontal
            height_fraction = height / self.amount_of_booms_vertical

            """Top flange"""
            for j in range(self.amount_of_booms_horizontal):
                j += 1
                boom = FuselageBoom(
                    width / 2 - j * width_fraction,
                    -height / 2,
                )
                section.add_boom(boom)

            boom_2 = FuselageBoom(
                -width / 2, -height / 2, type="spar"
            )  # corner boom top right
            section.add_boom(boom_2)

            """Right flange"""
            for j in range(self.amount_of_booms_vertical):
                j += 1
                boom = FuselageBoom(
                    -width / 2,
                    -height / 2 + j * height_fraction,
                )
                section.add_boom(boom)

            boom_3 = FuselageBoom(
                -width / 2, height / 2, type="spar"
            )  # corner boom bottom right
            section.add_boom(boom_3)

            """Bottom flange"""
            for j in range(self.amount_of_booms_horizontal):
                j += 1
                boom = FuselageBoom(-width / 2 + j * width_fraction, height / 2)
                section.add_boom(boom)

            boom_4 = FuselageBoom(
                width / 2, height / 2, type="spar"
            )  # corner boom bottom left
            section.add_boom(boom_4)

            """Bottom flange"""
            for j in range(self.amount_of_booms_vertical):
                j += 1
                boom = FuselageBoom(
                    width / 2,
                    -height / 2 + j * height_fraction,
                )
                section.add_boom(boom)

        """MIDDLE SECTION"""
        middle_length_fraction = (
            self.length_middle_section / self.amount_of_sections_middle
        )

        for i in range(self.amount_of_sections_middle):
            section = FuselageSection(
                self.drone,
                f"Fuselage Section Middle {i}",
                length=middle_length_fraction,
            )
            section.x_pos = -i * middle_length_fraction - self.length_start_section
            self.sections.append(section)

            width = self.width_middle_section
            height = self.height_middle_section

            boom_1 = FuselageBoom(
                width / 2, -height / 2, type="spar"
            )  # corner boom top left
            section.add_boom(boom_1)
            width_fraction = width / self.amount_of_booms_horizontal
            height_fraction = height / self.amount_of_booms_vertical
            """Top flange"""
            for j in range(self.amount_of_booms_horizontal):
                j += 1
                boom = FuselageBoom(
                    width / 2 - j * width_fraction,
                    -height / 2,
                )
                section.add_boom(boom)

            boom_2 = FuselageBoom(
                -width / 2, -height / 2, type="spar"
            )  # corner boom top right
            section.add_boom(boom_2)

            """Right flange"""
            for j in range(self.amount_of_booms_vertical):
                j += 1
                boom = FuselageBoom(
                    -width / 2,
                    -height / 2 + j * height_fraction,
                )
                section.add_boom(boom)

            boom_3 = FuselageBoom(
                -width / 2, height / 2, type="spar"
            )  # corner boom bottom right
            section.add_boom(boom_3)

            """Bottom flange"""
            for j in range(self.amount_of_booms_horizontal):
                j += 1
                boom = FuselageBoom(-width / 2 + j * width_fraction, height / 2)
                section.add_boom(boom)

            boom_4 = FuselageBoom(
                width / 2, height / 2, type="spar"
            )  # corner boom bottom left
            section.add_boom(boom_4)

            """Bottom flange"""
            for j in range(self.amount_of_booms_vertical):
                j += 1
                boom = FuselageBoom(
                    width / 2,
                    -height / 2 + j * height_fraction,
                )
                section.add_boom(boom)

        """END SECTION"""
        end_length_fraction = self.length_end_section / self.amount_of_sections_end
        end_width_taper = (
            self.width_end_section - self.width_middle_section
        ) / self.length_end_section

        end_height_taper = (
            self.height_end_section - self.height_middle_section
        ) / self.length_end_section
        for i in range(self.amount_of_sections_end + 1):
            section = FuselageSection(
                self.drone,
                f"Fuselage Section End {i}",
                length=middle_length_fraction,
            )
            section.x_pos = (
                -i * end_length_fraction
                - self.length_start_section
                - self.length_middle_section
            )
            self.sections.append(section)

            width = (
                self.width_middle_section + end_width_taper * i * end_length_fraction
            )
            height = (
                self.height_middle_section + end_height_taper * i * end_length_fraction
            )

            boom_1 = FuselageBoom(
                width / 2, -height / 2, type="spar"
            )  # corner boom top left
            section.add_boom(boom_1)
            width_fraction = width / self.amount_of_booms_horizontal
            height_fraction = height / self.amount_of_booms_vertical

            """Top flange"""
            for j in range(self.amount_of_booms_horizontal):
                j += 1
                boom = FuselageBoom(
                    width / 2 - j * width_fraction,
                    -height / 2,
                )
                section.add_boom(boom)

            boom_2 = FuselageBoom(
                -width / 2, -height / 2, type="spar"
            )  # corner boom top right
            section.add_boom(boom_2)

            """Right flange"""
            for j in range(self.amount_of_booms_vertical):
                j += 1
                boom = FuselageBoom(
                    -width / 2,
                    -height / 2 + j * height_fraction,
                )
                section.add_boom(boom)

            boom_3 = FuselageBoom(
                -width / 2, height / 2, type="spar"
            )  # corner boom bottom right
            section.add_boom(boom_3)

            """Bottom flange"""
            for j in range(self.amount_of_booms_horizontal):
                j += 1
                boom = FuselageBoom(-width / 2 + j * width_fraction, height / 2)
                section.add_boom(boom)

            boom_4 = FuselageBoom(
                width / 2, height / 2, type="spar"
            )  # corner boom bottom left
            section.add_boom(boom_4)

            """Bottom flange"""
            for j in range(self.amount_of_booms_vertical):
                j += 1
                boom = FuselageBoom(
                    width / 2,
                    -height / 2 + j * height_fraction,
                )
                section.add_boom(boom)

    def plot_fuselage_structure(self):
        self.sections: list[FuselageSection]

        x_points = []
        y_points = []
        z_points = []
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        # Set the initial view: elev (vertical angle), azim (horizontal angle), roll (not directly supported)
        ax.view_init(elev=-30, azim=-45, roll=-180)

        for i, section in enumerate(self.sections):
            for j, boom in enumerate(self.sections[i].booms):

                y_points.append(float(boom.y_pos))
                z_points.append(float(boom.z_pos))
                x_points.append(float(section.x_pos))

            shear_z_height = float(-section.shear_z) / 100
            shear_z_arrow = Arrow3D(
                [section.x_pos, section.x_pos],
                [0, 0],
                [0, -shear_z_height],
                mutation_scale=5,
                arrowstyle="-|>",
                color="r",
            )

            ax.add_artist(shear_z_arrow)

            shear_x_height = float(section.normal_force) / 100
            shear_x_arrow = Arrow3D(
                [section.x_pos, section.x_pos - shear_x_height],
                [0, 0],
                [0, 0],
                mutation_scale=5,
                arrowstyle="-|>",
                color="r",
            )

            ax.add_artist(shear_x_arrow)

        ax.scatter(x_points, y_points, z_points, c="b")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Wing Structure 3D Plot")
        plt.show()

    def add_wing_analysis(self, wing_analysis: WingStructuralAnalysis):
        self.wing_analysis = wing_analysis

    def apply_wing_reaction_forces(self):
        ratio = self.wing_to_fuselage_root_chord_ratio
        if self.wing_analysis is None:
            raise RuntimeError(
                "There is no wing in the fuselage analysis and \
                thus no reaction forces can be calculated"
            )
        wing_moment_x, wing_moment_z, wing_force_x, wing_force_z = (
            self.wing_analysis.reaction_forces()
        )

        width_root_section = self.wing_analysis.sections[0].booms[0].x_pos * 2 * ratio
        back_section_position = -width_root_section / 2 + self.wing_pos

        front_section_position = width_root_section / 2 + self.wing_pos

        front_section_index = min(
            range(len(self.sections)),
            key=lambda idx: abs(self.sections[idx].x_pos - front_section_position),
        )
        back_section_index = min(
            range(len(self.sections)),
            key=lambda idx: abs(self.sections[idx].x_pos - back_section_position),
        )
        amount_of_sections = back_section_index - front_section_index + 1
        if amount_of_sections == 0:
            amount_of_sections = 1 + 1

        per_section_wing_shear_z = -wing_force_z / amount_of_sections
        per_section_wing_normal_x = -wing_force_x / amount_of_sections

        for sec in self.sections[front_section_index : back_section_index + 1]:
            sec.shear_z += per_section_wing_shear_z
            sec.normal_force += per_section_wing_normal_x
            sec.ansys_load += wing_moment_x / amount_of_sections
            sec.special_section = True

        print("front_section_index", front_section_index)
        print("back_section_index", back_section_index)
        print("amount of sections", amount_of_sections)

        # times 2 because 2 wings, moments of both wings will
        # be calculated otherwise
        # closest_section.shear_z += -wing_force_z * 2
        # closest_section.normal_force += -wing_force_x * 2

    def apply_propeller_loads(self):
        location_front_props = -self.length_start_section
        location_back_props = -self.length_start_section - self.length_middle_section
        thrust_per_propeller = self.drone.MTOW / 4 * g * 2  # ASSUMPTION
        moment_by_propeller = thrust_per_propeller * self.propeller_rod_length
        closest_front_section = min(
            self.sections, key=lambda section: abs(section.x_pos - location_front_props)
        )

        closest_back_section = min(
            self.sections, key=lambda section: abs(section.x_pos - location_back_props)
        )
        closest_front_section.shear_z += -2 * thrust_per_propeller
        closest_back_section.shear_z += -2 * thrust_per_propeller

        closest_back_section.special_section = True
        closest_back_section.ansys_load -= moment_by_propeller / 2
        closest_front_section.special_section = True
        closest_front_section.ansys_load -= moment_by_propeller / 2

    def apply_back_propeller_loads(self):
        last_section = self.sections[-1]
        thrust_per_propeller = (
            self.drone.aero.constant_drag_distribution() * self.drone.wing.span
        ) * 1.5  # ASSUMPTION

        last_section.normal_force += thrust_per_propeller

    def apply_tail_loads(self):
        tail_shear_z = 5  # TODO
        tail_shear_y = 100
        tail_drag_x = -10
        tail_position = -1.3  # [m]

        closest_section = min(
            self.sections, key=lambda section: abs(section.x_pos - tail_position)
        )

        closest_section.shear_z += tail_shear_z
        closest_section.shear_y += tail_shear_y
        closest_section.normal_force += tail_drag_x

    def apply_propeller_motor_weight_front(self):
        shear_z = (
            self.drone.propulsion.motor.weight()
            + self.drone.propulsion.ver_prop.weight()
        ) * 2
        position = self.length_start_section  # [m]
        moment = position * shear_z
        closest_section = min(
            self.sections, key=lambda section: abs(section.x_pos - position)
        )
        closest_section.special_section = True
        closest_section.ansys_load += moment
        closest_section.shear_z += shear_z

    def apply_propeller_motor_weight_back(self):
        shear_z = (
            self.drone.propulsion.motor.weight()
            + self.drone.propulsion.ver_prop.weight()
        ) * 2
        position = self.length_start_section + self.length_middle_section  # [m]
        moment = position * shear_z
        closest_section = min(
            self.sections, key=lambda section: abs(section.x_pos - position)
        )
        closest_section.special_section = True
        closest_section.ansys_load += moment
        closest_section.shear_z += shear_z

    def apply_delivery_mech_weight(self):
        shear_z = toml["config"]["payload"]["del_mech_weight"]
        position = self.length_start_section + self.length_middle_section / 2  # [m]
        closest_section = min(
            self.sections, key=lambda section: abs(section.x_pos - position)
        )
        closest_section.shear_z += shear_z

    def apply_boom_material(self):
        for _, section in enumerate(self.sections):
            for _, boom in enumerate(section.booms):
                boom.material = self.material

    def calc_analysis_forces(self, PLOT=False):
        """This is basically the internal load diagrams"""
        self.sections: list[FuselageSection]
        cutoff_index = (
            self.amount_of_sections_start + self.amount_of_sections_middle // 2
        )
        boundary_condition_section = self.sections[cutoff_index]
        diagram_position = []
        diagram_shear_z = []
        diagram_shear_y = []
        diagram_analysis_normal_force = []
        diagram_analysis_moment_z = []
        diagram_analysis_moment_y = []
        diagram_position_p2 = []
        diagram_shear_z_p2 = []
        diagram_shear_y_p2 = []
        diagram_analysis_normal_force_p2 = []
        diagram_analysis_moment_z_p2 = []
        diagram_analysis_moment_y_p2 = []

        for i, section in enumerate(self.sections[0 : cutoff_index + 1]):

            section.analysis_normal_force = sum(
                sec.normal_force for sec in self.sections[: i + 1 :]
            )
            section.analysis_shear_z = sum(
                sec.weight + sec.shear_z for sec in self.sections[: i + 1 :]
            )
            section.analysis_shear_y = sum(
                sec.shear_y for sec in self.sections[: i + 1 :]
            )

            section.analysis_moment_z = sum(
                sec.moment_z + sec.analysis_shear_y for sec in self.sections[: i + 1 :]
            ) / (
                self.amount_of_sections_start + self.amount_of_sections_middle // 2 + 1
            )
            section.analysis_moment_y = sum(
                sec.moment_y + sec.analysis_shear_z for sec in self.sections[: i + 1 :]
            ) / (
                self.amount_of_sections_start + self.amount_of_sections_middle // 2 + 1
            )

            diagram_position.append(section.x_pos)
            diagram_shear_z.append(-float(section.analysis_shear_z))
            diagram_shear_y.append(-float(section.analysis_shear_y))
            diagram_analysis_normal_force.append(-float(section.analysis_normal_force))
            diagram_analysis_moment_z.append(-float(section.analysis_moment_z))
            diagram_analysis_moment_y.append(-float(section.analysis_moment_y))

        reversed_sections = self.sections[::-1]
        for i, section in enumerate(reversed_sections[0 : cutoff_index + 1]):

            section.analysis_normal_force = sum(
                sec.normal_force for sec in reversed_sections[: i + 1 :]
            )
            section.analysis_shear_z = sum(
                sec.weight + sec.shear_z for sec in reversed_sections[: i + 1 :]
            )
            section.analysis_shear_y = sum(
                sec.shear_y for sec in reversed_sections[: i + 1 :]
            )

            section.analysis_moment_z = sum(
                sec.moment_z + sec.analysis_shear_y
                for sec in reversed_sections[: i + 1 :]
            ) / (
                self.amount_of_sections_start + self.amount_of_sections_middle // 2 + 1
            )
            section.analysis_moment_y = sum(
                sec.moment_y + sec.analysis_shear_z
                for sec in reversed_sections[: i + 1 :]
            ) / (
                self.amount_of_sections_start + self.amount_of_sections_middle // 2 + 1
            )

            diagram_position_p2.append(section.x_pos)
            diagram_shear_z_p2.append(-float(section.analysis_shear_z))
            diagram_shear_y_p2.append(-float(section.analysis_shear_y))
            diagram_analysis_normal_force_p2.append(
                -float(section.analysis_normal_force)
            )
            diagram_analysis_moment_z_p2.append(-float(section.analysis_moment_z))
            diagram_analysis_moment_y_p2.append(-float(section.analysis_moment_y))

        diagram_position = diagram_position + diagram_position_p2[::-1]
        diagram_shear_z = diagram_shear_z + diagram_shear_z_p2[::-1]
        diagram_shear_y = diagram_shear_y + diagram_shear_y_p2[::-1]
        diagram_analysis_normal_force = (
            diagram_analysis_normal_force + diagram_analysis_normal_force_p2[::-1]
        )
        diagram_analysis_moment_z = (
            diagram_analysis_moment_z + diagram_analysis_moment_z_p2[::-1]
        )
        diagram_analysis_moment_y = (
            diagram_analysis_moment_y + diagram_analysis_moment_y_p2[::-1]
        )
        if PLOT:
            fig, axs = plt.subplots(3, 2, sharex=True, figsize=(8, 8))
            axs[0][0].plot(
                diagram_position,
                diagram_analysis_normal_force,
                label="Normal Force",
                color="green",
            )
            axs[0][0].set_xlabel("Position (x)")
            axs[0][0].set_ylabel("Normal Force")
            axs[0][0].set_title("Fuselage Normal Force Diagram")
            axs[0][0].legend()
            axs[0][0].grid(True)

            axs[1][0].plot(diagram_position, diagram_shear_z, label="Shear Z")
            axs[1][0].set_ylabel("Shear Force Z")
            axs[1][0].set_title("Fuselage Shear Force Z Diagram")
            axs[1][0].legend()
            axs[1][0].grid(True)

            axs[1][1].plot(
                diagram_position, diagram_shear_y, label="Shear Y", color="orange"
            )
            axs[1][1].set_xlabel("Position (x)")
            axs[1][1].set_ylabel("Shear Force Y")
            axs[1][1].set_title("Fuselage Shear Force Y Diagram")
            axs[1][1].legend()
            axs[1][1].grid(True)

            axs[2][0].plot(
                diagram_position,
                diagram_analysis_moment_y,
                label="Moment Y",
                color="orange",
            )
            axs[2][0].set_xlabel("Position (x)")
            axs[2][0].set_ylabel("Moment Y")
            axs[2][0].set_title("Fuselage Moment Y Diagram")
            axs[2][0].legend()
            axs[2][0].grid(True)

            axs[2][1].plot(
                diagram_position,
                diagram_analysis_moment_z,
                label="Moment Z",
                color="orange",
            )
            axs[2][1].set_xlabel("Position (x)")
            axs[2][1].set_ylabel("Moment Z")
            axs[2][1].set_title("Fuselage Moment Z Diagram")
            axs[2][1].legend()
            axs[2][1].grid(True)

            plt.tight_layout()
            plt.show()

    def apply_boom_area(self, spar_area: float | None, stringer_area: float | None):

        if spar_area == None:
            spar_area = self.start_spar_boom_area
        if stringer_area == None:
            stringer_area = self.start_stringer_boom_area

        for _, section in enumerate(self.sections):

            for i, boom in enumerate(section.booms):
                added_area = 0
                if False:
                    added_area = (
                        self.web_thickness
                        * abs(
                            section.booms[i - 1].y_pos
                            - boom.y_pos
                            + section.booms[i - 1].z_pos
                            - boom.z_pos
                        )
                        / 6
                    )
                if boom.type == "spar":

                    boom.area = spar_area + added_area
                else:
                    boom.area = stringer_area + added_area

    def check_buckling_fail(self, boom: FuselageBoom):
        length = boom.section.length
        l_e_l_const = 2  # taken from literature
        critical_stress = (
            np.pi**2 * boom.material.E / ((l_e_l_const * length) / boom.radius) ** 2
        )
        if -critical_stress < boom.normal_stress * self.safety_factor:
            # the critical stress is higher than the normal stress,
            # careful with signs because only checking compression \
            # thus the structure does not fail
            return False
        else:
            return True

    def check_shear_fail(self, section: FuselageSection):
        if section.special_section:
            ansys_stress = (
                calculate_stress_ansys(self.web_thickness)
                * section.ansys_load
                * 3
                / 0.05**2
            )
        else:
            ansys_stress = 0
        """This needs to be checked"""
        sum_shear_flow = sum(abs(boom.shear_flow_delta) for boom in section.booms)

        tau = sum_shear_flow / self.web_thickness + ansys_stress

        if tau * self.safety_factor > self.material.mat_tau_max:
            """Structure fails"""
            return True
        else:
            """Structure passes"""
            return False

    def check_tensile_ult_fail(self, boom: FuselageBoom):
        if abs(boom.normal_stress) * self.safety_factor > self.material.mat_stress_uts:
            return True
        else:
            return False

    def check_structure_fail(self) -> tuple[bool, list[FuselageBoom]]:
        failed_booms: list[WingBoom] = []
        for _, section in enumerate(self.sections):
            for _, boom in enumerate(section.booms):
                if self.check_tensile_ult_fail(boom):
                    boom.failure_method = "tensile_ult"
                    print("Failure method", "tensile_ult")

                    failed_booms.append(boom)
                    continue
                if self.check_buckling_fail(boom):
                    boom.failure_method = "buckling_fail"
                    print("Failure method", "buckling_fail")

                    failed_booms.append(boom)
                    continue
            if self.check_shear_fail(section):
                boom.failure_method = "shear_fail"
                print("Failure method", "shear_fail")

                failed_booms.extend(section.booms)
                continue

        if len(failed_booms) == 0:
            return False, _
        else:
            return True, failed_booms

    @property
    def weight(self):
        weight = 0
        for _, section in enumerate(self.sections):
            weight += section.weight
        return weight

    def perform_iterations(self):
        area_reduction_factor = 0.001
        web_grow_factor = 0.0001
        area_spar = self.start_spar_boom_area
        area_stringer = self.start_stringer_boom_area

        while True:
            self.apply_boom_material()
            self.apply_boom_area(area_spar, area_stringer)
            self.calc_analysis_forces()
            failed, failed_booms = self.check_structure_fail()
            if failed:
                if all(
                    boom.failure_method == "shear_fail" or boom.failure_method == None
                    for boom in failed_booms
                ):
                    print("shear_fail")
                    self.web_thickness += web_grow_factor
                    area_spar = area_spar + area_spar * area_reduction_factor
                    area_stringer = (
                        area_stringer + area_stringer * area_reduction_factor
                    )

                elif all(boom.type == "spar" for boom in failed_booms):
                    print("Failed spar")
                    area_spar = area_spar + 2 * area_spar * area_reduction_factor

                elif all(boom.type == "stringer" for boom in failed_booms):
                    print("Failed stringer")
                    area_stringer = (
                        area_stringer + 2 * area_stringer * area_reduction_factor
                    )

                else:
                    area_spar = area_spar + area_spar * area_reduction_factor
                    area_stringer = (
                        area_stringer + area_stringer * area_reduction_factor
                    )
                    self.apply_boom_material()
                    self.apply_boom_area(area_spar, area_stringer)
                    self.calc_analysis_forces()
                    print("Done with iterations")
                    print("Minimum thickness", self.web_thickness)

                    break
                for boom in failed_booms:
                    boom.failure_method == None
            if self.web_thickness < 0.001:
                self.web_thickness = 0.001
            area_spar = area_spar - area_spar * area_reduction_factor
            area_stringer = area_stringer - area_stringer * area_reduction_factor
            # print("Thickness", self.web_thickness)
            # print("area_spar", area_spar)
            # print("area_stringer", area_stringer)
            print("weight", self.weight)


if __name__ == "__main__":
    from prelim_des.drone import Drone
    from prelim_des.mission import Mission
    mission = Mission("DRCCRCD")
    drone = Drone()
    perf = Performance(drone, mission)
    material = Material("al_6061_t4")
    print("Material density", material.density)
    drone.perf = perf
    drone.class_1_weight_estimate()
    drone.wing.S = perf.wing_area(drone.OEW)
    print("MTOW", drone.MTOW)
    print("Root chord", drone.wing.c_root)
    print("tip chord", drone.wing.c_tip)
    print("Span", drone.wing.span)

    fuselage_structural_analysis = FuselageStructuralAnalysis(drone, material)
    fuselage_structural_analysis.make_fuselage_structure()
    # fuselage_structural_analysis.plot_wing_structure()

    wing_structural_analysis = WingStructuralAnalysis(drone, material)
    wing_structural_analysis.make_wing_structure()
    wing_structural_analysis.apply_lift()
    wing_structural_analysis.apply_drag()
    wing_structural_analysis.perform_iterations()
    wing_structural_analysis.plot_wing_structure()

    fuselage_structural_analysis.add_wing_analysis(wing_structural_analysis)
    fuselage_structural_analysis.apply_wing_reaction_forces()
    fuselage_structural_analysis.apply_propeller_loads()
    fuselage_structural_analysis.apply_back_propeller_loads()
    fuselage_structural_analysis.apply_tail_loads()
    fuselage_structural_analysis.apply_propeller_motor_weight_front()
    fuselage_structural_analysis.apply_propeller_motor_weight_back()
    fuselage_structural_analysis.apply_delivery_mech_weight()
    fuselage_structural_analysis.perform_iterations()
    print("Final weight", fuselage_structural_analysis.weight)
    print("Final weight wing", wing_structural_analysis.weight)

    fuselage_structural_analysis.calc_analysis_forces(True)
    fuselage_structural_analysis.plot_fuselage_structure()

    # for sec in fuselage_structural_analysis.sections:
    #     for boom in sec.booms:
    #         print(boom.type)
    # fuselage_structural_analysis.apply_boom_area(None, None)

    # fuselage_structural_analysis.calc_analysis_forces()

    # fuselage_structural_analysis.plot_fuselage_structure()

    # wing_structural_analysis.apply_lift()
    # wing_structural_analysis.apply_drag()

    # # wing_structural_analysis.add_point_load(1, 1, 1, Fx=-50)
    # # wing_structural_analysis.reaction_forces()

    # wing_structural_analysis.perform_iterations()
    # # wing_structural_analysis.plot_wing_structure()
    # wing_structural_analysis.weight

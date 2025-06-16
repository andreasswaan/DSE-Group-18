from prelim_des.drone import Drone
from prelim_des.maneuvre_envelope import plot_maneuver_and_gust_envelope
from prelim_des.mission import Mission
from prelim_des.new_wing_structure.wing_boom_class import WingBoom
from prelim_des.new_wing_structure.wing_section_class import WingSection
from prelim_des.performance import Performance
from prelim_des.utils.arrows import Arrow3D
from prelim_des.utils.import_toml import load_toml
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
import numpy as np
from prelim_des.new_wing_structure.material_class import Material
from prelim_des.constants import g

toml = load_toml()


class WingStructuralAnalysis:
    """ASSUMPTION: torsion is neglected for now"""

    """ASSUMPTION: it is perfectly rectangle"""

    sections = list["WingSection"]

    def __init__(self, drone: Drone, material: Material):
        self.drone = drone
        self.material = material
        self.sections_amount = toml["config"]["structures"]["wing"][
            "amount_of_sections"
        ]
        self.ratio_width_box_to_root_chord = toml["config"]["structures"]["wing"][
            "ratio_width_box_to_root_chord"
        ]
        self.sections: list[WingSection] = []
        self.amount_of_booms_horizontal = toml["config"]["structures"]["wing"][
            "amount_of_booms_horizontal"
        ]
        self.amount_of_booms_vertical = toml["config"]["structures"]["wing"][
            "amount_of_booms_vertical"
        ]
        self.point_loads = []

        self.start_spar_boom_area = toml["config"]["structures"]["wing"][
            "start_spar_boom_area"
        ]

        self.start_stringer_boom_area = toml["config"]["structures"]["wing"][
            "start_stringer_boom_area"
        ]

        self.safety_factor = toml["config"]["structures"]["wing"]["safety_factor"]

        self.web_thickness = toml["config"]["structures"]["wing"]["web_thickness"]

    def make_wing_structure(self):
        wing_span_fraction = (self.drone.wing.span / 2) / self.sections_amount
        for i in range(self.sections_amount + 1):
            section = WingSection(
                self.drone, f"Wing Section {i}", length=wing_span_fraction
            )
            section.y_pos = i * wing_span_fraction
            self.sections.append(section)

            width = (
                self.drone.wing.chord(i * wing_span_fraction)
                * self.ratio_width_box_to_root_chord
            )

            height = (
                self.drone.wing.chord(i * wing_span_fraction)
                * self.ratio_width_box_to_root_chord
                * self.drone.wing.thick_over_chord
            )
            boom_id = 0
            boom_1 = WingBoom(
                width / 2, -height / 2, type="spar", id=boom_id
            )  # corner boom top left
            section.add_boom(boom_1)
            boom_id += 1

            width_fraction = width / self.amount_of_booms_horizontal

            """Top flange"""
            for j in range(self.amount_of_booms_horizontal):
                j += 1
                boom = WingBoom(width / 2 - j * width_fraction, -height / 2, id=boom_id)
                section.add_boom(boom)
                section.top_flange_ids.append(boom_id)
                boom_id += 1

            boom_2 = WingBoom(
                -width / 2, -height / 2, type="spar", id=boom_id
            )  # corner boom top right
            section.add_boom(boom_2)
            boom_id += 1

            height_fraction = height / self.amount_of_booms_vertical
            """Right flange"""
            for j in range(self.amount_of_booms_vertical):
                j += 1
                boom = WingBoom(
                    -width / 2, -height / 2 + j * height_fraction, id=boom_id
                )
                section.add_boom(boom)
                section.right_flange_ids.append(boom_id)
                boom_id += 1

            boom_3 = WingBoom(
                width / 2, height / 2, type="spar", id=boom_id
            )  # corner boom bottom left
            section.add_boom(boom_3)
            boom_id += 1

            """Bottom flange"""
            for j in range(self.amount_of_booms_horizontal):
                j += 1
                boom = WingBoom(-width / 2 + j * width_fraction, height / 2, id=boom_id)
                section.add_boom(boom)
                section.bottom_flange_ids.append(boom_id)
                boom_id += 1

            boom_4 = WingBoom(
                -width / 2, height / 2, type="spar", id=boom_id
            )  # corner boom bottom right
            section.add_boom(boom_4)
            boom_id += 1

            """Left flange"""
            for j in range(self.amount_of_booms_vertical):
                j += 1
                boom = WingBoom(width / 2, height / 2 - j * height_fraction, id=boom_id)
                section.add_boom(boom)
                section.left_flange_ids.append(boom_id)
                boom_id += 1

    def plot_wing_structure(self):
        self.sections: list[WingSection]
        x_points = []
        y_points = []
        z_points = []
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        # Set the initial view: elev (vertical angle), azim (horizontal angle), roll (not directly supported)
        ax.view_init(elev=-30, azim=-45, roll=-180)

        for i, section in enumerate(self.sections):
            for j, boom in enumerate(self.sections[i].booms):

                x_points.append(float(-boom.x_pos))
                z_points.append(float(-boom.z_pos))
                y_points.append(float(section.y_pos))

            shear_z_height = float(-section.shear_z) / 10
            shear_z_arrow = Arrow3D(
                [0, 0],
                [section.y_pos, section.y_pos],
                [0, -shear_z_height],
                mutation_scale=5,
                arrowstyle="-|>",
                color="r",
            )

            ax.add_artist(shear_z_arrow)

            shear_x_height = float(section.shear_x) / 10
            shear_x_arrow = Arrow3D(
                [0, shear_x_height],
                [section.y_pos, section.y_pos],
                [0, 0],
                mutation_scale=5,
                arrowstyle="-|>",
                color="r",
            )

            ax.add_artist(shear_x_arrow)

            # x_points.append(float(0))
            # z_points.append(float(-section.shear_z) / 1000)
            # y_points.append(float(section.y_pos))

        ax.scatter(x_points, y_points, z_points, c="b")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Wing Structure 3D Plot")
        plt.show()

    def apply_lift(self):
        self.sections: list[WingSection]

        for _, section in enumerate(self.sections):
            lift_per_section = (
                self.drone.aero.elliptical_lift_distribution(section.y_pos)
                * self.drone.wing.span
                / 2
                / len(self.sections)
            )
            section.shear_z += -lift_per_section * plot_maneuver_and_gust_envelope(
                self.drone
            )  # TODO: to be checked

    def apply_drag(self):
        self.sections: list[WingSection]
        drag_per_section = (
            self.drone.aero.constant_drag_distribution()
            * self.drone.wing.span
            / 2
            / len(self.sections)
        )
        for _, section in enumerate(self.sections):
            section.shear_x += (
                -drag_per_section * plot_maneuver_and_gust_envelope(self.drone) ** 2
            )  # TODO: to be checked

    def add_point_load(self, x, y, z, Fx=0, Fy=0, Fz=0):
        self.sections: list[WingSection]

        # Step 1: find closest section in y coordinate
        closest_section = min(self.sections, key=lambda section: abs(section.y_pos - y))
        self.point_loads.append({"x": x, "y": y, "z": z})

        closest_section.shear_x += Fx
        closest_section.normal_force += Fy
        closest_section.shear_z += Fz

    def apply_boom_material(self):
        for _, section in enumerate(self.sections):
            for _, boom in enumerate(section.booms):
                boom.material = self.material

    def reaction_forces(self):
        reaction_moment_x = 0
        for _, section in enumerate(self.sections):
            reaction_moment_x -= (section.shear_z + section.weight) * section.y_pos
        # print("Reaction moment x", reaction_moment_x)

        reaction_moment_z = 0
        for _, section in enumerate(self.sections):
            reaction_moment_z -= section.shear_x * section.y_pos
        # print("Reaction moment z", reaction_moment_z)

        reaction_force_x = 0
        for _, section in enumerate(self.sections):
            reaction_force_x -= section.shear_x
        # print("Reaction force x", reaction_force_x)

        reaction_force_z = 0
        for _, section in enumerate(self.sections):
            reaction_force_z -= section.shear_z + section.weight
        # print("Reaction force z", reaction_force_z)

        # print("analysis_shear_z", reaction_force_z)
        return reaction_moment_x, reaction_moment_z, reaction_force_x, reaction_force_z

    def calc_analysis_forces(self):
        """This is basically the internal load diagrams"""
        self.sections: list[WingSection]

        for i, section in enumerate(self.sections):

            section.analysis_normal_force = sum(
                sec.normal_force + sec.normal_force for sec in self.sections[i::]
            )
            section.analysis_shear_z = sum(
                sec.shear_z + sec.weight for sec in self.sections[i::]
            )
            section.analysis_shear_x = sum(sec.shear_x for sec in self.sections[i::])

            section.analysis_moment_z = sum(
                sec.moment_z + sec.shear_x * sec.y_pos for sec in self.sections[i::]
            )
            section.analysis_moment_x = sum(
                sec.moment_x + sec.shear_z * sec.y_pos for sec in self.sections[i::]
            )

        print("analysis_shear_z", self.sections[0].analysis_shear_z)

    def apply_propeller_folding_mech_weight(self):
        shear_z = 0.225 * g  # TODO: siddarth implement the regression
        position = 0.85  # [m]

        closest_section = min(
            self.sections, key=lambda section: abs(section.y_pos - position)
        )
        closest_section.shear_z += shear_z

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
                            section.booms[i - 1].x_pos
                            - boom.x_pos
                            + section.booms[i - 1].z_pos
                            - boom.z_pos
                        )
                        / 6
                    )
                if boom.type == "spar":
                    boom.area = spar_area + added_area
                else:
                    boom.area = stringer_area + added_area

    def check_buckling_fail(self, boom: WingBoom):
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

    def check_shear_fail(self, boom: WingBoom):
        """This needs to be checked"""
        if boom.type == "spar":
            tau = boom.shear_flow_delta / self.web_thickness

            if tau * self.safety_factor > self.material.mat_tau_max:
                """Structure fails"""
                return True
            else:
                """Structure passes"""
                return False
        else:
            return False

    def check_tensile_ult_fail(self, boom: WingBoom):
        if abs(boom.normal_stress) * self.safety_factor > self.material.mat_stress_uts:
            return True
        else:
            return False

    def check_structure_fail(self):
        failed_booms: list[WingBoom] = []
        for _, section in enumerate(self.sections):
            for _, boom in enumerate(section.booms):
                if self.check_tensile_ult_fail(boom):
                    boom.failure_method = "tensile_ult"
                    failed_booms.append(boom)
                    continue
                if self.check_buckling_fail(boom):
                    boom.failure_method = "buckling_fail"
                    failed_booms.append(boom)
                    continue
                if self.check_shear_fail(boom):
                    boom.failure_method = "shear_fail"
                    failed_booms.append(boom)
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
        print("wing weight", weight, "N")
        return weight

    def perform_iterations(self):
        area_reduction_factor = 0.01
        area_spar = self.start_spar_boom_area
        area_stringer = self.start_stringer_boom_area

        while True:
            self.apply_boom_material()
            self.apply_boom_area(area_spar, area_stringer)
            self.calc_analysis_forces()
            failed, failed_booms = self.check_structure_fail()
            if failed:
                if all(boom.type == "spar" for boom in failed_booms):
                    print("Failed spar")
                    area_spar = area_spar + 2 * area_spar * area_reduction_factor

                if all(boom.type == "stringer" for boom in failed_booms):
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
                    break

            area_spar = area_spar - area_spar * area_reduction_factor
            area_stringer = area_stringer - area_stringer * area_reduction_factor

        print("Final spar boom area", self.sections[0].booms[0].area)
        print("Final stringer boom area", self.sections[0].booms[1].area)
        print("First section height", self.sections[0].booms[0].z_pos)


if __name__ == "__main__":
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

    wing_structural_analysis = WingStructuralAnalysis(drone, material)
    wing_structural_analysis.make_wing_structure()
    wing_structural_analysis.apply_lift()
    wing_structural_analysis.apply_drag()
    wing_structural_analysis.perform_iterations()

    # wing_structural_analysis.add_point_load(1, 1, 1, Fx=-50)
    # wing_structural_analysis.reaction_forces()

    wing_structural_analysis.plot_wing_structure()
    wing_structural_analysis.weight

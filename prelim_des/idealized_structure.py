import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from prelim_des.utils.import_toml import load_toml
from prelim_des.constants import g
import csv


toml = load_toml()


def elliptical_lift_distribution(y: float, b: float, L_total: float) -> float:
    """
    Computes lift per unit span (N/m) at spanwise position y from centerline.
    Assumes elliptical distribution.

    Parameters:
        y (float): Position along span (from root, 0 ≤ y ≤ b/2)
        b (float): Full wingspan
        L_total (float): Total lift over the entire wing

    Returns:
        float: Lift per unit span at y (N/m)
    """
    return (4 * L_total / (np.pi * b)) * np.sqrt(1 - (2 * y / b) ** 2)


def constant_weight_distribution(y: float, b: float, W_total: float) -> float:
    """
    Computes weight per unit span (N/m) at spanwise position y from centerline.
    Assumes constant weight distribution along the wing.

    Parameters:
        y (float): Position along span (from root, 0 ≤ y ≤ b/2)
        b (float): Full wingspan
        W_total (float): Total weight supported by the wing (N)

    Returns:
        float: Weight per unit span at y (N/m)
    """
    return W_total / (b / 2)  # Divide by half-span (you model half wing)


def find_critical_stress(
    sections, stresses_per_section, shear_stresses_per_section=None
):
    critical = {
        "max_ratio": 0,
        "section_idx": None,
        "boom_idx": None,
        "stress": None,
        "allowable": None,
        "type": None,
    }
    for i, section in enumerate(sections):
        for j, boom in enumerate(section.booms):
            bending = stresses_per_section[i][j]
            allowable = boom.material.yield_strength
            ratio = abs(bending) / allowable if allowable else 0
            if ratio > critical["max_ratio"]:
                critical.update(
                    {
                        "max_ratio": ratio,
                        "section_idx": i,
                        "boom_idx": j,
                        "stress": bending,
                        "allowable": allowable,
                        "type": "bending",
                    }
                )
            # Optionally check shear as well
            if shear_stresses_per_section:
                shear = shear_stresses_per_section[i][j]
                tau_allow = boom.material.tau_max
                tau_ratio = abs(shear) / tau_allow if tau_allow else 0
                if tau_ratio > critical["max_ratio"]:
                    critical.update(
                        {
                            "max_ratio": tau_ratio,
                            "section_idx": i,
                            "boom_idx": j,
                            "stress": shear,
                            "allowable": tau_allow,
                            "type": "shear",
                        }
                    )
    return critical


def euler_buckling_stress(E, K, L, r):
    # buckling mode: one free end, the other fixed
    # r = 2
    return (np.pi**2 * E) / ((K * L / r) ** 2)


class Material:
    def __init__(
        self,
        name: str,
        E: float,
        G: float,
        density: float,
        yield_strength: float,
        uts: float,
        tau_max: float,
        epsilon_max: float,
    ):
        self.name = name
        self.E = E * 1e9  # GPa → Pa
        self.G = G * 1e9  # GPa → Pa
        self.density = density * 1000  # g/cm³ → kg/m³
        self.yield_strength = yield_strength * 1e6  # MPa → Pa
        self.uts = uts * 1e6  # MPa → Pa
        self.tau_max = tau_max * 1e6  # MPa → Pa
        self.epsilon_max = epsilon_max  # Unitless

    def __repr__(self):
        return f"Material({self.name}, E={self.E:.2e} Pa, ρ={self.density} kg/m³)"


def load_materials(toml: dict) -> dict[str, Material]:
    material_dict = {}
    all_materials = toml["config"]["material"]
    for mat_name, props in all_materials.items():
        # Use only mat_E_1 if present, else mat_E, else mat_E_2
        if "mat_E_1" in props:
            E = props["mat_E_1"]
        elif "mat_E" in props:
            E = props["mat_E"]
        elif "mat_E_2" in props:
            E = props["mat_E_2"]
        else:
            raise ValueError(f"Material {mat_name} missing modulus E")

        # Use only mat_delta_y_1 if present, else mat_delta_y, else mat_delta_y_2
        if "mat_delta_y_1" in props:
            yield_strength = props["mat_delta_y_1"]
        elif "mat_delta_y" in props:
            yield_strength = props["mat_delta_y"]
        elif "mat_delta_y_2" in props:
            yield_strength = props["mat_delta_y_2"]
        else:
            yield_strength = 0  # or raise error

        material = Material(
            name=mat_name,
            E=E,
            G=props.get("mat_G", 0),
            density=props["mat_rho"],
            yield_strength=yield_strength,
            uts=props.get("mat_delta_uts", 0),
            tau_max=props.get("mat_tau_max", 0),
            epsilon_max=props.get("mat_epsilon_max", 0),
        )
        material_dict[mat_name] = material
    return material_dict


materials = load_materials(toml)


class Boom:
    def __init__(
        self,
        x: float,
        y: float,
        area: float,
        material: Material | None = None,
        material_name: str = None,
        materials: dict[str, Material] = None,
        boom_type="regular",
    ):
        self.x = x  # [m]
        self.y = y  # [m]
        self.area = area  # [m^2]
        self.type = boom_type

        if material:  # Use direct material object
            self.material = material
        elif material_name and materials:  # Look up from dictionary
            if material_name not in materials:
                raise ValueError(
                    f"Material '{material_name}' not found in materials dictionary."
                )
            self.material = materials[material_name]
        else:
            raise ValueError(
                "You must specify either a `material` or `material_name` with `materials` dictionary."
            )

    def __repr__(self):
        return f"Boom({self.x:.2f}, {self.y:.2f}, A={self.area}, Type={self.type}, Material={self.material.name})"


class IdealizedSection:
    def __init__(self, booms: list[Boom]):
        self.booms = booms
        self.centroid_x, self.centroid_y = self.calc_centroid()
        self.Ixx, self.Iyy, self.Ixy = self.calc_moments()

    def calc_centroid(self):
        A_total = sum(b.area for b in self.booms)
        x_c = sum(b.area * b.x for b in self.booms) / A_total
        y_c = sum(b.area * b.y for b in self.booms) / A_total
        return x_c, y_c

    def calc_moments(self):
        x_c, y_c = self.centroid_x, self.centroid_y
        Ixx = sum(b.area * (b.y - y_c) ** 2 for b in self.booms)
        Iyy = sum(b.area * (b.x - x_c) ** 2 for b in self.booms)
        Ixy = sum(b.area * (b.x - x_c) * (b.y - y_c) for b in self.booms)
        return Ixx, Iyy, Ixy

    def bending_stress(self, Mx: float, My: float) -> list[float]:
        x_c, y_c = self.centroid_x, self.centroid_y
        Ixx, Iyy, Ixy = self.Ixx, self.Iyy, self.Ixy
        denom = Ixx * Iyy - Ixy**2
        stresses = []
        for b in self.booms:
            y = b.y - y_c
            x = b.x - x_c
            sigma = -(My * Ixx * x - Mx * Ixy * x + Mx * Iyy * y - My * Ixy * y) / denom
            stresses.append(sigma)
        return stresses

    # def bending_stress_from_lift(
    #     self, lift_distribution: list[float], dy: float
    # ) -> list[float]:
    #     """
    #     Computes bending stress at each boom due to distributed lift.
    #     lift_distribution: list of lift values [N] at each section (same length as number of sections).
    #     dy: distance between sections [m].
    #     Returns: list of lists, each sublist is the stress at each boom for that section.
    #     """
    #     stresses_per_section = []
    #     n_sections = len(lift_distribution)
    #     # Calculate bending moment at each section (root to tip)
    #     # M(y) = sum of lift * arm from y to tip
    #     moments = []
    #     for i in range(n_sections):
    #         arm = np.arange(i, n_sections) * dy - (i * dy)
    #         moment = np.sum(np.array(lift_distribution[i:]) * arm)
    #         moments.append(moment)
    #     # For each section, calculate stress at each boom
    #     for moment in moments:
    #         # Assume moment is about z-axis (bending in vertical plane, My)
    #         My = moment
    #         Mx = 0.0
    #         x_c, y_c = self.centroid_x, self.centroid_y
    #         Ixx, Iyy, Ixy = self.Ixx, self.Iyy, self.Ixy
    #         denom = Ixx * Iyy - Ixy**2
    #         stresses = []
    #         for b in self.booms:
    #             y = b.y - y_c
    #             x = b.x - x_c
    #             sigma = -(
    #                 My * Ixx * x - Mx * Ixy * x + Mx * Iyy * y - My * Ixy * y
    #             ) / denom
    #             stresses.append(sigma)
    #         stresses_per_section.append(stresses)
    #     return stresses_per_section

    import matplotlib.pyplot as plt

    def plot_section(self):
        fig, ax = plt.subplots()
        ax.set_aspect("equal")

        # Separate booms by type for color coding
        for boom in self.booms:
            if hasattr(boom, "type") and boom.type == "spar":
                color = "red"
            else:
                color = "blue"

            # Marker size scaled to boom area
            marker_size = (
                boom.area * 1e6 * 100
            )  # scale for visibility (1e6: m² → mm², *100: visual tweak)

            ax.plot(boom.x, boom.y, "o", color=color, markersize=np.sqrt(marker_size))

        # Plot centroid
        ax.axhline(self.centroid_y, color="gray", linestyle="--", linewidth=1)
        ax.axvline(self.centroid_x, color="gray", linestyle="--", linewidth=1)
        ax.plot(self.centroid_x, self.centroid_y, "k+", markersize=10)

        ax.set_title("Idealized Cross Section")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    def mass(self, segment_length=1.0) -> float:
        """
        Returns the total mass of the section over a given spanwise segment length [m].
        """
        total_mass = 0.0
        for boom in self.booms:
            volume = boom.area * segment_length  # [m^3]
            mass = volume * boom.material.density  # [kg]
            total_mass += mass
        return total_mass

    def shear_flow(self, Vz: float) -> list[float]:
        """
        Compute shear flow at each boom location due to vertical shear force Vz.
        Assumes an open thin-walled section with booms.
        """
        Ixx = self.Ixx
        y_c = self.centroid_y
        q_values = []

        for boom in self.booms:
            Q = sum(
                b.area * (b.y - y_c)
                for b in self.booms
                if b.y > boom.y  # only booms above the current boom
            )
            q = Vz * Q / Ixx
            q_values.append(q)

        return q_values

    def shear_stress(self, Vz: float, thickness: float = 0.002) -> list[float]:
        """
        Compute shear stress at each boom by dividing shear flow by assumed thickness.
        Default thickness is 2 mm.
        """
        q_values = self.shear_flow(Vz)
        tau_values = [q / thickness for q in q_values]
        return tau_values


def create_rectangular_section(
    width,
    height,
    n_regular_booms: int,
    spar_cap_area: float,
    regular_boom_area: float,
    material_name: str,
    materials: dict[str, Material],
) -> IdealizedSection:
    booms = []

    # Corner positions for spar caps (fixed at corners)
    corners = [
        (-width / 2, -height / 2),
        (width / 2, -height / 2),
        (width / 2, height / 2),
        (-width / 2, height / 2),
    ]

    for x, y in corners:
        booms.append(
            Boom(
                x,
                y,
                spar_cap_area,
                boom_type="spar",
                material_name=material_name,
                materials=materials,
            )
        )

    # Distribute regular booms along the sides (between spar caps)
    # Total of 4 sides
    regular_booms_per_side = [n_regular_booms // 4] * 4
    for i in range(n_regular_booms % 4):
        regular_booms_per_side[i] += 1  # spread remaining booms

    for side, n_booms in enumerate(regular_booms_per_side):
        # Get start and end corner
        x0, y0 = corners[side]
        x1, y1 = corners[(side + 1) % 4]

        for i in range(1, n_booms + 1):
            # Divide the side into (n_booms + 1) segments
            x = x0 + (x1 - x0) * i / (n_booms + 1)
            y = y0 + (y1 - y0) * i / (n_booms + 1)
            booms.append(
                Boom(
                    x,
                    y,
                    regular_boom_area,
                    boom_type="regular",
                    material_name=material_name,
                    materials=materials,
                )
            )

    return IdealizedSection(booms)


def create_circular_section(
    diameter: float,
    n_booms: int,
    boom_area: float,
    material_name: str,
    materials: dict[str, Material],
    cap_area: float = None,  # Optional: area for top/bottom "caps"
) -> IdealizedSection:
    """
    Creates an idealized circular section with booms evenly spaced around the circumference.
    Optionally adds two "cap" booms at the top and bottom.
    """
    booms = []
    radius = diameter / 2

    # Place regular booms evenly around the circle
    for i in range(n_booms):
        theta = 2 * np.pi * i / n_booms
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        booms.append(
            Boom(
                x,
                y,
                boom_area,
                boom_type="regular",
                material_name=material_name,
                materials=materials,
            )
        )

    # Optionally add "cap" booms at top and bottom (y = +r, y = -r)
    if cap_area is not None:
        for y_cap in [radius, -radius]:
            booms.append(
                Boom(
                    0.0,
                    y_cap,
                    cap_area,
                    boom_type="cap",
                    material_name=material_name,
                    materials=materials,
                )
            )

    return IdealizedSection(booms)


class FuselageStructure:
    def __init__(
        self,
        length: float,
        n_sections: int,
        root_section: IdealizedSection,
        taper_ratio: float = 1.0,  # For constant-diameter, keep at 1.0
    ):
        self.length = length
        self.n_sections = n_sections
        self.taper_ratio = taper_ratio
        self.root_section = root_section
        self.dz = length / (n_sections - 1)
        self.sections = self.generate_sections()

    def generate_sections(self) -> list[tuple[float, IdealizedSection]]:
        """
        Returns a list of (x_position, IdealizedSection) from nose to tail.
        """
        sections = []
        for i in range(self.n_sections):
            x = (i / (self.n_sections - 1)) * self.length
            scale = 1 - (1 - self.taper_ratio) * (i / (self.n_sections - 1))
            scaled_booms = [
                Boom(
                    x=boom.x * scale,
                    y=boom.y * scale,
                    area=boom.area,
                    boom_type=boom.type,
                    material=boom.material,
                )
                for boom in self.root_section.booms
            ]
            section = IdealizedSection(scaled_booms)
            sections.append((x, section))
        return sections

    def compute_bending_stresses(
        self, Mz_per_section: list[float], My_per_section: list[float] = None
    ) -> list[list[float]]:
        """
        Returns a list of [stress at each boom] for each section.
        """
        stresses_per_section = []
        for i, (x_pos, section) in enumerate(self.sections):
            Mz = Mz_per_section[i]
            My = My_per_section[i] if My_per_section else 0
            # Pass Mz as "Mx" and My as "My" to the section's bending_stress method,
            # since the section method expects (Mx, My), but for fuselage, Mx = Mz (longitudinal axis)
            stresses = section.bending_stress(Mx=Mz, My=My)
            stresses_per_section.append(stresses)
        return stresses_per_section

    def compute_bending_moments_with_point_loads(
        self,
        Mz_distributed: list[float],
        My_distributed: list[float],
        point_loads: list[dict],
    ) -> tuple[list[float], list[float]]:
        """
        Returns two lists: (Mz_per_section, My_per_section)
        - Mz: bending moment about z-axis (from Px)
        - My: bending moment about y-axis (from Pz)
        Includes effects of distributed moments and point loads.
        point_loads: list of dicts, each with {"x": position, "Px": ..., "Pz": ...}
        """
        Mz_list = []
        My_list = []
        section_positions = [x for x, _ in self.sections]
        for i, x in enumerate(section_positions):
            Mz = Mz_distributed[i] if Mz_distributed else 0
            My = My_distributed[i] if My_distributed else 0
            for pl in point_loads:
                if pl["x"] >= x:
                    arm = pl["x"] - x
                    Px = pl.get("Px", 0)
                    Pz = pl.get("Pz", 0)
                    Mz += Px * arm
                    My += Pz * arm
            Mz_list.append(Mz)
            My_list.append(My)
        return Mz_list, My_list

    def plot_3d_fuselage(
        self, stresses_per_section: list[list[float]], point_loads: list[dict] = None
    ):
        import matplotlib.colors as mcolors
        from matplotlib import cm

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.view_init(azim=210)

        # Flatten all stresses for color normalization
        all_stresses = [
            s for section_stresses in stresses_per_section for s in section_stresses
        ]
        norm = mcolors.Normalize(vmin=min(all_stresses), vmax=max(all_stresses))
        cmap = cm.get_cmap("viridis")

        for i, (x_pos, section) in enumerate(self.sections):
            stresses = stresses_per_section[i]
            for boom, stress in zip(section.booms, stresses):
                color = cmap(norm(stress))
                # x: fuselage length, y: sideways, z: upwards
                ax.scatter(
                    x_pos,  # x: fuselage length
                    boom.x,  # y: sideways
                    boom.y,  # z: upwards
                    color=color,
                    s=boom.area * 1e6 * 50,
                )

        # --- Plot point load arrows ---
        if point_loads:
            for pl in point_loads:
                # Default to 0 if not specified
                Px = pl.get("Px", 0)
                Pz = pl.get("Pz", 0)
                # Arrow at (sideways=0, fuselage x=pl["x"], upwards=0)
                ax.quiver(
                    pl["x"],
                    0,
                    0,  # base: x=fuselage length, y=sideways, z=upwards
                    0,
                    Px / 1000,
                    Pz
                    / 1000,  # direction: y=sideways, z=upwards, scaled for visibility
                    color="red",
                    arrow_length_ratio=0.2,
                    linewidth=3,
                    alpha=0.9,
                    label="Point Load",
                )

        ax.set_title("Fuselage Structure (Booms colored by Bending Stress)")
        ax.set_xlabel("x [m] (fuselage length)")
        ax.set_ylabel("y [m] (sideways)")
        ax.set_zlabel("z [m] (upwards)")
        ax.grid(True)

        mappable = cm.ScalarMappable(norm=norm, cmap=cmap)
        mappable.set_array(all_stresses)
        cbar = plt.colorbar(mappable, ax=ax, pad=0.1)
        cbar.set_label("Bending Stress [Pa]")

        plt.tight_layout()
        plt.show()


# def plot_section_stress(section: IdealizedSection, stresses: list[float], title="Section Stress"):
#     import matplotlib.pyplot as plt
#     import matplotlib.colors as mcolors
#     from matplotlib import cm
#     fig, ax = plt.subplots()
#     ax.set_aspect("equal")

#     # Normalize for color
#     norm = mcolors.Normalize(vmin=min(stresses), vmax=max(stresses))
#     cmap = cm.get_cmap("viridis")

#     for boom, stress in zip(section.booms, stresses):
#         color = cmap(norm(stress))
#         marker_size = boom.area * 1e6 * 100
#         ax.plot(boom.x, boom.y, "o", color=color, markersize=np.sqrt(marker_size))

#     ax.axhline(section.centroid_y, color="gray", linestyle="--", linewidth=1)
#     ax.axvline(section.centroid_x, color="gray", linestyle="--", linewidth=1)
#     ax.plot(section.centroid_x, section.centroid_y, "k+", markersize=10)
#     ax.set_title(title)
#     ax.set_xlabel("x [m]")
#     ax.set_ylabel("y [m]")
#     plt.colorbar(cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax, label="Bending Stress [Pa]")
#     plt.grid(True)
#     plt.tight_layout()
#     plt.show()

# if __name__ == "__main__":
#     width = 0.2  # m
#     height = 0.12  # m
#     n_booms = 15
#     boom_area = 1e-5  # m^2
#     spar_cap_area = 2e-5  # 20 mm²

#     section = create_rectangular_section(
#         width, height, n_booms, spar_cap_area, boom_area
#     )
#     print(f"Centroid: ({section.centroid_x:.4f}, {section.centroid_y:.4f}) m")
#     print(f"Ixx = {section.Ixx:.6e} m^4")
#     print(f"Iyy = {section.Iyy:.6e} m^4")
#     print(f"Ixy = {section.Ixy:.6e} m^4")

#     Mx = 10.0  # Nm
#     My = 5.0  # Nm
#     stresses = section.bending_stress(Mx, My)
#     for i, sigma in enumerate(stresses):
#         print(f"Boom {i+1}: Stress = {sigma:.2f} Pa")

#     section.plot_section()


class WingStructure:
    def __init__(
        self,
        span: float,
        n_sections: int,
        root_section: IdealizedSection,
        taper_ratio: float,
    ):
        self.span = span
        self.n_sections = n_sections
        self.taper_ratio = taper_ratio
        self.root_section = root_section
        self.dy = span / 2 / (n_sections - 1)
        self.sections = self.generate_sections()
        self.total_weight = None

    def generate_sections(self) -> list[tuple[float, IdealizedSection]]:
        """
        Returns a list of (y_position, IdealizedSection) from root to tip.
        """
        sections = []

        for i in range(self.n_sections):
            y = (i / (self.n_sections - 1)) * (
                self.span / 2
            )  # Half-span (symmetric wing)
            scale = 1 - (1 - self.taper_ratio) * (i / (self.n_sections - 1))

            scaled_booms = [
                Boom(
                    x=boom.x * scale,
                    y=boom.y * scale,
                    area=boom.area,  # * (scale**2),
                    boom_type=boom.type,
                    material=boom.material,
                )
                for boom in self.root_section.booms
            ]
            section = IdealizedSection(scaled_booms)
            sections.append((y, section))

        return sections

    def compute_total_weight(self, g: float = 9.81) -> float:
        """
        Computes total structural weight of the wing based on boom areas, material densities, and span.
        Assumes the total wing (not just half).
        """
        total_weight = 0.0
        for _, section in self.sections:
            for boom in section.booms:
                volume = boom.area * self.dy  # dy is the section spacing
                mass = volume * boom.material.density
                total_weight += mass * g
        return 2 * total_weight  # Full span (you model only half)

    def compute_weight_distribution(self) -> list[float]:
        """
        Returns a list of weight per section [N] for half-wing.
        """
        if self.total_weight is None:
            raise ValueError(
                "Set self.total_weight before computing weight distribution."
            )

        weight_per_section = []
        for y_pos, _ in self.sections:
            w_y = constant_weight_distribution(y_pos, self.span, self.total_weight)
            weight_per_section.append(w_y * self.dy)
        return weight_per_section

    def compute_net_vertical_load(
        self, lift_per_section: list[float], weight_per_section: list[float]
    ) -> list[float]:
        """
        Computes net vertical force per section (lift - weight).
        """
        return [L - W for L, W in zip(lift_per_section, weight_per_section)]

    def compute_shear_forces(self, net_vertical_load: list[float]) -> list[float]:
        """
        Returns list of shear forces [N] at each section along span.
        """
        shear_forces = []
        for i in range(len(net_vertical_load)):
            shear = sum(net_vertical_load[i:])
            shear_forces.append(shear)
        return shear_forces

    def compute_bending_moments(self, net_vertical_load: list[float]) -> list[float]:
        # def compute_bending_moments(self, lift_per_section: list[float]) -> list[float]:
        """
        Returns a list of bending moments at each section (about x-axis).
        """
        moments_x = []
        for i, (y_pos, _) in enumerate(self.sections):
            moment = 0
            for j in range(i, len(self.sections)):
                y_out, _ = self.sections[j]
                arm = y_out - y_pos
                moment += net_vertical_load[j] * arm
            moments_x.append(moment)

        # # Plot the moment distribution along the span
        # y_positions = [y for y, _ in self.sections]
        # plt.figure()
        # plt.plot(y_positions, moments_x, marker="o")
        # plt.xlabel("Spanwise Position y [m]")
        # plt.ylabel("Bending Moment Mx [Nm]")
        # plt.title("Bending Moment Distribution Along Wing Span")
        # plt.grid(True)
        # plt.tight_layout()
        # plt.show()
        return moments_x

    def compute_bending_stresses(
        self, net_vertical_load: list[float]
    ) -> tuple[list[float], list[list[float]]]:
        moments_x = self.compute_bending_moments(net_vertical_load)
        stresses_per_section = []
        for i, (y_pos, section) in enumerate(self.sections):
            stresses = section.bending_stress(Mx=moments_x[i], My=0)
            stresses_per_section.append(stresses)
        return moments_x, stresses_per_section

    def compute_vertical_deflections(
        self, net_vertical_load: list[float]
    ) -> list[float]:
        """
        Returns the vertical deflection at each section along the span.
        Assumes constant EI (uses root section Ixx and material E).
        """
        E = self.root_section.booms[0].material.E  # Pa
        Ixx = self.root_section.Ixx  # m^4
        dy = self.dy

        moments_x = self.compute_bending_moments(net_vertical_load)

        # Integrate twice to get deflection (Euler-Bernoulli, discrete)
        theta = [0.0]  # slope at root
        for i in range(1, len(moments_x)):
            dtheta = moments_x[i - 1] * dy / (E * Ixx)
            theta.append(theta[-1] + dtheta)
        w = [0.0]  # deflection at root
        for i in range(1, len(theta)):
            dw = theta[i - 1] * dy
            w.append(w[-1] + dw)
        return w

    def plot_3d_wing(
        self,
        stresses_per_section: list[list[float]],
        lift_per_section: list[float] = None,
        weight_per_section: list[float] = None,
    ):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        from matplotlib import cm
        import matplotlib.colors as mcolors

        # --- Flatten all stresses for color normalization ---
        all_stresses = [
            stress
            for section_stresses in stresses_per_section
            for stress in section_stresses
        ]
        min_stress = min(all_stresses)
        max_stress = max(all_stresses)
        norm = mcolors.Normalize(vmin=min_stress, vmax=max_stress)
        cmap = cm.get_cmap("viridis")

        # --- Plot booms colored by stress ---
        for i, (y_pos, section) in enumerate(self.sections):
            stresses = stresses_per_section[i]
            for boom, stress in zip(section.booms, stresses):
                color = cmap(norm(stress))
                x = boom.x
                z = boom.y
                ax.scatter(
                    x,
                    y_pos,
                    z,
                    color=color,
                    s=boom.area * 1e6 * 50,
                )

        # --- Add load arrows (lift) ---
        arrow_scale = None
        if lift_per_section:
            arrow_scale = (
                max(lift_per_section) / 0.1 if max(lift_per_section) != 0 else 1.0
            )

            for i, (y_pos, section) in enumerate(self.sections):
                x_c = np.mean([boom.x for boom in section.booms])
                upper_boom = max(section.booms, key=lambda b: b.y)
                z_c = upper_boom.y
                lift = lift_per_section[i]
                ax.quiver(
                    x_c,
                    y_pos,
                    z_c,
                    0,
                    0,
                    lift / arrow_scale,
                    color="red",
                    arrow_length_ratio=0.2,
                    linewidth=2,
                    alpha=0.7,
                    label="Lift" if i == 0 else None,
                )

        # --- Add load arrows (weight, downward) ---
        if weight_per_section:
            # Use the same arrow_scale as lift for direct comparison
            if arrow_scale is None:
                arrow_scale = (
                    max(weight_per_section) / 0.1
                    if max(weight_per_section) != 0
                    else 1.0
                )

            for i, (y_pos, section) in enumerate(self.sections):
                x_c = np.mean([boom.x for boom in section.booms])
                lower_boom = min(section.booms, key=lambda b: b.y)
                z_c = lower_boom.y
                weight = weight_per_section[i]
                ax.quiver(
                    x_c,
                    y_pos,
                    z_c,
                    0,
                    0,
                    -weight / arrow_scale,
                    color="blue",
                    arrow_length_ratio=0.2,
                    linewidth=2,
                    alpha=0.7,
                    label="Weight" if i == 0 else None,
                )

        ax.set_title("3D Wing Structure (Color-coded by Local Stress)")
        ax.set_xlabel("x [m] (chordwise)")
        ax.set_ylabel("y [m] (spanwise)")
        ax.set_zlabel("z [m] (vertical)")
        ax.grid(True)

        # Add color bar for stress
        mappable = cm.ScalarMappable(norm=norm, cmap=cmap)
        mappable.set_array(all_stresses)
        cbar = plt.colorbar(mappable, ax=ax, pad=0.1)
        cbar.set_label("Bending Stress [Pa]")

        # Add legend for arrows
        handles = []
        if lift_per_section:
            handles.append(plt.Line2D([0], [0], color="red", lw=2, label="Lift"))
        if weight_per_section:
            handles.append(plt.Line2D([0], [0], color="blue", lw=2, label="Weight"))
        if handles:
            ax.legend(handles=handles, loc="upper left")

        plt.tight_layout()
        plt.show()

    def plot_deformed_wing(self, vertical_deflections: list[float]):
        import matplotlib.colors as mcolors
        from matplotlib import cm

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        # Normalize deflections for coloring
        min_defl = min(vertical_deflections)
        max_defl = max(vertical_deflections)
        norm = mcolors.Normalize(vmin=min_defl, vmax=max_defl)
        cmap = cm.get_cmap("plasma")

        for i, (y_pos, section) in enumerate(self.sections):
            dz = vertical_deflections[i]
            color = cmap(norm(dz))
            for boom in section.booms:
                x = boom.x
                y = y_pos
                z = boom.y + dz  # add deflection to original z
                ax.scatter(x, y, z, color=color, s=boom.area * 1e6 * 50)

        ax.set_title(
            "Vertical Deflection of Wing Structure (Booms colored by Deflection)"
        )
        ax.set_xlabel("x [m] (chordwise)")
        ax.set_ylabel("y [m] (spanwise)")
        ax.set_zlabel("z [m] (vertical)")
        ax.grid(True)

        # Add color bar for deflection
        mappable = cm.ScalarMappable(norm=norm, cmap=cmap)
        mappable.set_array(vertical_deflections)
        cbar = plt.colorbar(mappable, ax=ax, pad=0.1)
        cbar.set_label("Vertical Deflection [m]")

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    SAFETY_FACTOR = 2.0

    # Create fuselage cross-section
    fuselage_root_section = create_circular_section(
        diameter=1.0,
        n_booms=16,
        boom_area=1e-5,
        material_name="al_6061_t4",
        materials=materials,
        cap_area=2e-5,  # Optional, can omit if not needed
    )

    fuselage = FuselageStructure(
        length=5.0,  # total fuselage length
        n_sections=20,
        root_section=fuselage_root_section,
        taper_ratio=1.0,  # 1.0 for constant diameter, <1 for tapered
    )

    # Get fuselage section positions
    section_positions = [x for x, _ in fuselage.sections]

    # If you have distributed moments, set them here (or use zeros)
    Mz_distributed = [0] * fuselage.n_sections  # or your distributed moment
    My_distributed = [0] * fuselage.n_sections

    # Define point loads
    point_loads = [
        {"x": 3.0, "Pz": -2000},  # 2000 N downward at x=3.0 m
        {"x": 0.0, "Px": 1500},  # 1500 N sideways (positive x) at x=4.0 m
    ]

    # Compute moments including point loads
    Mz_per_section, My_per_section = fuselage.compute_bending_moments_with_point_loads(
        Mz_distributed, My_distributed, point_loads
    )

    # Compute and plot stresses as before
    fuselage_stresses_per_section = fuselage.compute_bending_stresses(
        Mz_per_section, My_per_section
    )
    fuselage.plot_3d_fuselage(fuselage_stresses_per_section, point_loads=point_loads)

    # Create root cross-section
    root_section = create_rectangular_section(
        width=0.6,
        height=0.072,
        n_regular_booms=12,
        spar_cap_area=2e-5,
        regular_boom_area=1e-5,
        material_name="al_6061_t4",
        materials=materials,
    )

    wing = WingStructure(
        span=2,  # total wingspan → half span = 1 m
        n_sections=10,
        root_section=root_section,
        taper_ratio=0.4,
    )

    dy = wing.dy
    # b_half = wing.span / 2
    b = wing.span
    L_total = 100  # total lift in N, replace with actual value

    lift_per_section = []
    for y, _ in wing.sections:
        L_prime = elliptical_lift_distribution(y, b, L_total)
        lift = L_prime * dy
        lift_per_section.append(lift)

    weight_per_section = [sec.mass(dy) * 9.81 for _, sec in wing.sections]
    total_vertical_load = [
        lift - weight for lift, weight in zip(lift_per_section, weight_per_section)
    ]

    # Compute internal shear force from tip to root
    shear_forces = []
    running_shear = 0.0
    for net_load in reversed(total_vertical_load):
        running_shear += net_load
        shear_forces.insert(0, running_shear)

    # Compute shear stresses per section
    shear_stresses_per_section = []
    for (y, sec), Vz in zip(wing.sections, shear_forces):
        shear_stresses = sec.shear_stress(
            Vz=Vz, thickness=0.002
        )  # adjust thickness if needed
        shear_stresses_per_section.append(shear_stresses)

    moments_x, stresses_per_section = wing.compute_bending_stresses(total_vertical_load)
    wing.plot_3d_wing(stresses_per_section, lift_per_section, weight_per_section)

    # wing.plot_3d_wing(lift_per_section)

    vertical_deflections = wing.compute_vertical_deflections(total_vertical_load)
    wing.plot_deformed_wing(vertical_deflections)

    critical = find_critical_stress(
        [sec for _, sec in wing.sections],
        stresses_per_section,
        shear_stresses_per_section,
    )

    # Apply safety factor to allowable
    allowable_with_sf = critical["allowable"] / SAFETY_FACTOR
    utilization_with_sf = (
        abs(critical["stress"]) / allowable_with_sf if allowable_with_sf else 0
    )

    print("\n=== CRITICAL STRESS REPORT ===")
    print(f"Max utilization ratio (with SF={SAFETY_FACTOR}): {utilization_with_sf:.3f}")
    print(
        f"Section index: {critical['section_idx']}, Boom index: {critical['boom_idx']}"
    )
    print(
        f"Stress: {critical['stress']:.2e} Pa, Allowable/SF: {allowable_with_sf:.2e} Pa"
    )
    print(f"Failure mode: {critical['type']}")
    if utilization_with_sf >= 1.0:
        print("WARNING: STRUCTURE IS OVERSTRESSED (with safety factor)!")
    else:
        print("Structure is safe (all utilization ratios < 1.0, with safety factor)")
    print("==============================\n")

    # Buckling check for the critical boom
    critical_boom = wing.sections[critical["section_idx"]][1].booms[
        critical["boom_idx"]
    ]
    E = critical_boom.material.E
    K = 2.0  # free-fixed end condition
    L = dy  # use segment length as effective length
    A = critical_boom.area
    r = np.sqrt(A / np.pi)  # crude estimate for a circular boom

    sigma_cr = euler_buckling_stress(E, K, L, r)
    sigma_cr_with_sf = sigma_cr / SAFETY_FACTOR

    print(f"Buckling stress (with SF): {sigma_cr_with_sf:.2e} Pa")
    if abs(critical["stress"]) > sigma_cr_with_sf:
        print("WARNING: Buckling failure at critical boom!")
    else:
        print("No buckling at critical boom (with safety factor).")

    with open("wing_sections.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "y [m]",
                "Ixx [m^4]",
                "Iyy [m^4]",
                "Ixy [m^4]",
                "x_c [m]",
                "y_c [m]",
                "mass [kg]",
            ]
        )

        # Estimate segment length between stations
        dy = wing.span / (2 * (wing.n_sections - 1))  # half-span, (n-1) segments

        for y, sec in wing.sections:
            writer.writerow(
                [
                    y,
                    sec.Ixx,
                    sec.Iyy,
                    sec.Ixy,
                    sec.centroid_x,
                    sec.centroid_y,
                    sec.mass(segment_length=dy),
                ]
            )

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
        self.dy = span / (n_sections - 1)
        self.sections = self.generate_sections()

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

    def plot_3d_wing(self, lift_per_section: list[float] = None):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        from matplotlib import cm
        import matplotlib.colors as mcolors

        # --- Compute moments at each section (simple cantilever assumption) ---
        # Mx = 0 (no moment about x), My = sum of lift outboard * distance from section
        moments_x = []
        for i, (y_pos, _) in enumerate(self.sections):
            if lift_per_section:
                moment = 0
                for j in range(i, len(self.sections)):
                    y_out, _ = self.sections[j]
                    arm = y_out - y_pos
                    moment += lift_per_section[j] * arm
                moments_x.append(moment)
            else:
                moments_x.append(0)

        # --- Compute all stresses for color normalization ---
        all_stresses = []
        for i, (y_pos, section) in enumerate(self.sections):
            stresses = section.bending_stress(Mx=moments_x[i], My=0)
            all_stresses.extend(stresses)
        min_stress = min(all_stresses)
        max_stress = max(all_stresses)
        norm = mcolors.Normalize(vmin=min_stress, vmax=max_stress)
        cmap = cm.get_cmap("viridis")

        # --- Plot booms colored by stress ---
        for i, (y_pos, section) in enumerate(self.sections):
            stresses = section.bending_stress(Mx=moments_x[i], My=0)
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
        if lift_per_section:
            arrow_scale = max(lift_per_section) / 0.1  # Adjust denominator for arrow length scaling

            for i, (y_pos, section) in enumerate(self.sections):
                # Compute centroid of section for arrow base
                x_c = np.mean([boom.x for boom in section.booms])
                upper_boom = max(section.booms, key=lambda b: b.y)
                z_c = upper_boom.y
                lift = lift_per_section[i]
                # Arrow points in +z (vertical) direction
                ax.quiver(
                    x_c, y_pos, z_c,   # base position
                    0, 0, lift / arrow_scale,  # direction vector (scaled)
                    color="red", arrow_length_ratio=0.2, linewidth=2, alpha=0.7
                )

        ax.set_title("3D Wing Structure (Color-coded by Local Lift)")
        ax.set_xlabel("x [m] (chordwise)")
        ax.set_ylabel("y [m] (spanwise)")
        ax.set_zlabel("z [m] (vertical)")
        ax.grid(True)

        # Add color bar for stress
        mappable = cm.ScalarMappable(norm=norm, cmap=cmap)
        mappable.set_array(all_stresses)
        cbar = plt.colorbar(mappable, ax=ax, pad=0.1)
        cbar.set_label("Bending Stress [Pa]")

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
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

    wing.plot_3d_wing(lift_per_section)

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

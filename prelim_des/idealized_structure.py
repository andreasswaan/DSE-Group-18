from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from prelim_des.drone import Drone
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib import cm
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d import Axes3D
from prelim_des.utils.import_toml import load_toml
from prelim_des.constants import g

# === CONFIG & MATERIALS ===

toml = load_toml()


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

# === LOAD DISTRIBUTIONS ===


def elliptical_lift_distribution(y: float, drone: Drone) -> float:
    """
    Computes lift per unit span (N/m) at spanwise position y from centerline.
    Assumes elliptical distribution. small change hello

    Parameters:
        y (float): Position along span (from root, 0 ≤ y ≤ b/2)

    Returns:
        float: Lift per unit span at y (N/m)
    """
    b = float(drone.wing.span)  # Use the drone's wing span
    CL_max = drone.aero.CL_max
    V_max = toml["config"]["mission"]["max_velocity"]
    L_total = float(drone.aero.lift(V_max, CL_max))  # Total lift at max velocity
    return (4 * L_total / (np.pi * b)) * np.sqrt(1 - (2 * y / b) ** 2)


# def constant_weight_distribution(
#     y: float, W_total: float, drone: Drone
# ) -> float:
#     """
#     Computes weight per unit span (N/m) at spanwise position y from centerline.
#     Assumes constant weight distribution along the wing.

#     Parameters:
#         y (float): Position along span (from root, 0 ≤ y ≤ b/2)
#         b (float): Full wingspan
#         W_total (float): Total weight supported by the wing (N)

#     Returns:
#         float: Weight per unit span at y (N/m)
#     """

#     return W_total / (b / 2)  # Divide by half-span (modelling half wing)


def constant_drag_distribution(drone: Drone) -> float:
    """
    Returns drag per unit span (N/m) at spanwise position y.
    Assumes constant drag distribution along the wing.
    """
    b = float(drone.wing.span)
    D_total = float(drone.aero.drag(toml["config"]["mission"]["max_velocity"]))

    return D_total / (b / 2)  # Divide by half-span (modelling half wing)


# === STRUCTURAL CLASSES ===


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
        Default thickness is 2 mm, shall be decided within the structures department.
        """
        q_values = self.shear_flow(Vz)
        tau_values = [q / thickness for q in q_values]
        return tau_values

    def torsional_shear_flow(self, torque: float) -> float:
        """
        Compute average shear flow in a closed section due to applied torque.
        Assumes closed thin-walled section.
        """
        # Estimate enclosed area (polygon area formula)
        x = np.array([boom.x for boom in self.booms])
        y = np.array([boom.y for boom in self.booms])
        area = 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))
        if area == 0:
            raise ValueError("Section area is zero; cannot compute torsion.")
        q = torque / (2 * area)
        return q

    def torsional_shear_stress(self, torque: float, thickness: float = 0.002) -> float:
        """
        Compute average shear stress due to torsion.
        """
        q = self.torsional_shear_flow(torque)
        return q / thickness

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


# === SECTION GENERATORS ===


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

    section = IdealizedSection(booms)
    section.width = width  # <-- Add this line
    section.height = height  # <-- (optional, for symmetry)
    return section


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


# === STRUCTURES ASSEMBLY ===


class ConnectorStructure:
    def __init__(
        self,
        length: float,
        n_sections: int,
        root_section: IdealizedSection,
        start_point: tuple[float, float, float],  # (x, y, z) at point load
        end_point: tuple[float, float, float],  # (x, y, z) at wing surface
    ):
        self.length = length
        self.n_sections = n_sections
        self.root_section = root_section
        self.start_point = np.array(start_point)
        self.end_point = np.array(end_point)
        self.sections = self.generate_sections()

    def generate_sections(self) -> list[tuple[np.ndarray, IdealizedSection]]:
        sections = []
        for i in range(self.n_sections):
            frac = i / (self.n_sections - 1)
            pos = self.start_point * (1 - frac) + self.end_point * frac
            # No scaling of cross-section for now, but you could add tapering if needed
            section = IdealizedSection(
                [
                    Boom(
                        x=boom.x,
                        y=boom.y,
                        area=boom.area,
                        boom_type=boom.type,
                        material=boom.material,
                    )
                    for boom in self.root_section.booms
                ]
            )
            sections.append((pos, section))
        return sections


class FuselageStructure:
    def __init__(
        self,
        length: float,
        root_section: IdealizedSection,
        n_sections: int,
        taper_ratio: float = 1.0,  # For constant-diameter, keep at 1.0
    ):
        self.length = length
        self.width = root_section.width
        self.height = root_section.height
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
        n = self.n_sections
        L = self.length
        
        # Define fractions for each region (adjust as needed)
        taper_frac = 0.2  # 20% of length for each taper
        const_frac = 1 - 2 * taper_frac
    
        for i in range(n):
            x = (i / (n - 1)) * L
            frac = x / L

            # Taper-in zone
            if frac < taper_frac:
                local_scale = 0.5 + 0.5 * (frac / taper_frac)  # from 0.5 to 1.0
            # Constant zone
            elif frac < taper_frac + const_frac:
                local_scale = 1.0
            # Taper-out zone
            else:
                local_scale = 1.0 - 0.5 * ((frac - taper_frac - const_frac) / taper_frac)  # from 1.0 to 0.5

            scaled_booms = [
                Boom(
                    x=boom.x * local_scale,
                    y=boom.y * local_scale,
                    area=boom.area,
                    boom_type=boom.type,
                    material=boom.material,
                )
                for boom in self.root_section.booms
            ]
            section = IdealizedSection(scaled_booms)
            sections.append((x, section))
        return sections

    def compute_weight_distribution(self) -> list[float]:
        """
        Returns a list of weight per section [N] for the fuselage,
        based on actual structural mass distribution.
        """
        dz = self.dz
        weight_per_section = []
        for _, section in self.sections:
            w_z = section.mass(dz) * g  # [N] for this section
            weight_per_section.append(w_z)
        return weight_per_section

    def compute_bending_moments_with_point_loads(
        self,
        point_loads: list[dict],
    ) -> tuple[list[float], list[float]]:
        """
        Returns two lists: (Mz_per_section, My_per_section)
        - Mz: bending moment about z-axis (from vertical forces and point moments)
        - My: bending moment about y-axis (from drag and point moments)
        Includes effects of point loads and point moments.
        """
        Mz_list = []
        My_list = []
        section_positions = [x for x, _ in self.sections]
        for i, x in enumerate(section_positions):
            Mz = 0
            My = 0
            for pl in point_loads:
                if pl["x"] >= x:
                    arm = pl["x"] - x
                    Pz = pl.get("Pz", 0)
                    Px = pl.get("Px", 0)
                    # Bending about z-axis from vertical force
                    Mz += Pz * arm
                    # Bending about y-axis from drag force
                    My += Px * arm
                    # Add any direct moments applied at this point
                    if abs(pl["x"] - x) < 1e-6:  # at this section
                        Mz += pl.get("Mz", 0)
                        My += pl.get("My", 0)
            Mz_list.append(Mz)
            My_list.append(My)
        return Mz_list, My_list

    def compute_bending_stresses(
        self, Mz_per_section: list[float], My_per_section: list[float]
    ) -> list[list[float]]:
        """
        Returns a list of [stress at each boom] for each section.
        """
        stresses_per_section = []
        for i, (x_pos, section) in enumerate(self.sections):
            Mz = Mz_per_section[i] if Mz_per_section else 0
            My = My_per_section[i] if My_per_section else 0
            # Pass both moments to the section's bending_stress method
            stresses = section.bending_stress(Mx=Mz, My=0)
            stresses_per_section.append(stresses)
        return stresses_per_section

    import numpy as np

    def draw_moment_arrow(
        self, ax, origin, axis="z", radius=0.1, direction=1, color="purple", lw=2
    ):
        # origin: (x, y, z)
        # axis: 'z' for moment about z, 'y' for moment about y
        # direction: 1 for CCW, -1 for CW (relative to axis)
        theta = np.linspace(0, np.pi / 1.5, 30)  # arc angle
        if axis == "z":
            x = origin[0] + radius * np.cos(theta)
            y = origin[1] + direction * radius * np.sin(theta)
            z = np.full_like(x, origin[2])
            # Arrowhead
            ax.quiver(
                x[-1],
                y[-1],
                z[-1],
                -0.05 * np.sin(theta[-1]),
                0.05 * np.cos(theta[-1]),
                0,
                color=color,
                linewidth=lw,
            )
        elif axis == "y":
            x = origin[0] + radius * np.cos(theta)
            y = np.full_like(x, origin[1])
            z = origin[2] + direction * radius * np.sin(theta)
            ax.quiver(
                x[-1],
                y[-1],
                z[-1],
                -0.05 * np.sin(theta[-1]),
                0,
                0.05 * np.cos(theta[-1]),
                color=color,
                linewidth=lw,
            )
        ax.plot(x, y, z, color=color, linewidth=lw)

    def plot_3d_fuselage(
        self,
        stresses_per_section: list[list[float]],
        point_loads: list[dict] = None,
        weight_per_section: list[float] = None,
        arrow_scale: float = 1.0,
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
        cmap = plt.colormaps["viridis"]

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
                    s=boom.area * 1e6 * 20,
                )

        # --- Plot point load arrows ---
        if point_loads:
            for pl in point_loads:
                # Default to 0 if not specified
                Px = pl.get("Px", 0)
                Py = pl.get("Py", 0)
                Pz = pl.get("Pz", 0)
                # Plot drag arrow if Px is nonzero
                if Px != 0:
                    ax.quiver(
                        pl["x"],
                        pl["y"],
                        pl["z"],
                        Px / arrow_scale,
                        0,
                        0,
                        color="green",  # or another color for drag
                        arrow_length_ratio=0.2,
                        linewidth=3,
                        alpha=0.9,
                        label="Drag",  # Only label the first time if needed
                    )
                # Plot vertical force arrow as before
                if Pz != 0:
                    ax.quiver(
                        pl["x"],
                        pl["y"],
                        pl["z"],
                        0,
                        0,
                        Pz / arrow_scale,
                        color="red",
                        arrow_length_ratio=0.2,
                        linewidth=3,
                        alpha=0.9,
                        label="Lift - Weight",
                    )

            # --- Plot moment (rotating) arrows if present ---
        for pl in point_loads:
            # Draw Mz (about z-axis)
            if abs(pl.get("Mz", 0)) > 1e-6:
                self.draw_moment_arrow(
                    ax,
                    (pl["x"], pl["y"], pl["z"]),
                    axis="z",
                    radius=0.1,
                    direction=np.sign(pl["Mz"]),
                    color="purple",
                    lw=3,
                )
            # Draw My (about y-axis)
            if abs(pl.get("My", 0)) > 1e-6:
                self.draw_moment_arrow(
                    ax,
                    (pl["x"], pl["y"], pl["z"]),
                    axis="y",
                    radius=0.1,
                    direction=np.sign(pl["My"]),
                    color="orange",
                    lw=3,
                )

        if weight_per_section:
            arrow_scale = (
                max(weight_per_section) / 0.1 if max(weight_per_section) != 0 else 1.0
            )
            for i, (x_pos, section) in enumerate(self.sections):
                x_c = np.mean([boom.x for boom in section.booms])
                y_c = np.mean([boom.y for boom in section.booms])
                weight = weight_per_section[i]
                ax.quiver(
                    x_pos,
                    x_c,
                    y_c,
                    0,
                    0,
                    -weight / arrow_scale,
                    color="blue",
                    arrow_length_ratio=0.2,
                    linewidth=2,
                    alpha=0.7,
                    label="Weight" if i == 0 else None,
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

        ax.xaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.yaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.zaxis.set_major_locator(ticker.MaxNLocator(4))

        ax.set_xlim(-0.3, 1.5)
        ax.set_ylim(-0.8, 0.8)
        ax.set_zlim(-0.5, 0.5)

        plt.tight_layout()
        plt.show()

    def compute_weight_distribution(self) -> list[float]:
        """
        Returns a list of weight per section [N] for the fuselage,
        based on actual structural mass distribution.
        """
        dz = self.dz
        weight_per_section = []
        for _, section in self.sections:
            w_z = section.mass(dz) * g  # [N] for this section
            weight_per_section.append(w_z)
        return weight_per_section

    def mass(self) -> float:
        """
        Returns the total structural mass of the fuselage [kg].
        """
        dz = self.length / (self.n_sections - 1)
        return sum(section.mass(dz) for _, section in self.sections)

    def compute_bending_moments_with_distributed_and_point_loads(
        self,
        distributed_loads: list[float],
        point_loads: list[dict],
    ) -> tuple[list[float], list[float]]:
        """
        Returns two lists: (Mz_per_section, My_per_section)
        - Mz: bending moment about z-axis (from vertical loads)
        Includes effects of distributed loads (e.g., fuselage weight) and point loads.
        """
        Mz_list = []
        My_list = []
        section_positions = [x for x, _ in self.sections]
        n = len(section_positions)
        for i, x in enumerate(section_positions):
            Mz = 0.0
            # Distributed loads (from i to end)
            for j in range(i, n):
                x_j = section_positions[j]
                arm = x_j - x
                Mz += distributed_loads[j] * arm
            # Point loads
            for pl in point_loads:
                if pl["x"] >= x:
                    arm = pl["x"] - x
                    Pz = pl.get("Pz", 0)
                    Mz += Pz * arm
            Mz_list.append(Mz)
            My_list.append(0.0)  # Not used, but kept for compatibility
        return Mz_list, My_list

    def compute_torsional_moments(self, point_loads: list[dict]) -> list[float]:
        """
        Returns a list of torsional moments (about x-axis) at each fuselage section.
        """
        Mx_list = []
        section_positions = [x for x, _ in self.sections]
        for i, x in enumerate(section_positions):
            Mx = 0.0
            for pl in point_loads:
                if pl["x"] >= x:
                    # Direct torsional moment
                    Mx += pl.get("Mx", 0)
                    # Torsion from side force (Py) at a lever arm (z)
                    Py = pl.get("Py", 0)
                    Mx += Py * (pl.get("z", 0))
            Mx_list.append(Mx)
        return Mx_list


class WingStructure:
    def __init__(
        self,
        n_sections: int,
        root_section: IdealizedSection,
        drone: Drone,
    ):
        self.span = float(drone.wing.span)
        self.n_sections = n_sections
        self.taper_ratio = float(drone.wing.taper)
        self.root_section = root_section
        self.dy = self.span / 2 / (n_sections - 1)
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
                    area=boom.area * (scale**2),
                    boom_type=boom.type,
                    material=boom.material,
                )
                for boom in self.root_section.booms
            ]
            section = IdealizedSection(scaled_booms)
            sections.append((y, section))

        return sections

    def compute_total_weight(self) -> float:
        """
        Computes total structural weight of the wing based on boom areas, material densities, and span.
        Assumes the total wing (not just half).
        """
        dy = self.dy
        weight_per_section = []
        total_weight = 0.0
        for _, section in self.sections:
            w_y = section.mass(dy) * g  # [N] for this section
            weight_per_section.append(w_y)
        return weight_per_section  # Full span (you model only half)

    def compute_weight_distribution(self) -> list[float]:
        """
        Returns a list of weight per section [N] for half-wing.
        """
        # if self.total_weight is None:
        #     raise ValueError(
        #         "Set self.total_weight before computing weight distribution."

        dz = self.dz
        weight_per_section = []
        for _, section in self.sections:
            w_z = section.mass(dz) * g  # [N] for this section
            weight_per_section.append(w_z)
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
        return moments_x

    def compute_bending_moments_with_point_loads(
        self, net_vertical_load: list[float], point_loads: list[dict]
    ) -> list[float]:
        """
        Returns a list of bending moments at each section (about x-axis),
        including effects of point loads (e.g., payload, landing gear).
        point_loads: list of dicts, each with {"y": position, "Pz": load}
        """
        moments_x = []
        section_positions = [y for y, _ in self.sections]
        for i, y in enumerate(section_positions):
            moment = 0
            # Distributed loads (as before)
            for j in range(i, len(self.sections)):
                y_out, _ = self.sections[j]
                arm = y_out - y
                moment += net_vertical_load[j] * arm
            # Add point loads
            for pl in point_loads:
                if pl["y"] >= y:
                    arm = pl["y"] - y
                    Pz = pl.get("Pz", 0)
                    moment += Pz * arm
            moments_x.append(moment)
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
        point_loads: list[dict] = None,
        drag_per_section: list[float] = None,
        connector_sections: list[dict] = None,
        connector_stresses_per_section: list[list[float]] = None,
        arrow_scale: float = 1.0,
    ):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        # --- Flatten all stresses for color normalization ---
        all_stresses = [
            stress
            for section_stresses in stresses_per_section
            for stress in section_stresses
        ]
        if connector_stresses_per_section:
            all_stresses += [
                stress
                for section_stresses in connector_stresses_per_section
                for stress in section_stresses
            ]
        min_stress = min(all_stresses)
        max_stress = max(all_stresses)
        norm = mcolors.Normalize(vmin=min_stress, vmax=max_stress)
        cmap = plt.colormaps["viridis"]

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
                    s=boom.area * 1e6 * 20,
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

        # --- Add drag arrows (if provided) ---
        if drag_per_section:
            arrow_scale = (
                max(drag_per_section) / 0.1 if max(drag_per_section) != 0 else 1.0
            )
            for i, (y_pos, section) in enumerate(self.sections):
                # Place drag arrow at mean chordwise position (x), at y=y_pos, z=mean boom height
                x_c = np.mean([boom.x for boom in section.booms])
                z_c = np.mean([boom.y for boom in section.booms])
                drag = drag_per_section[i]
                ax.quiver(
                    x_c,
                    y_pos,
                    z_c,
                    drag / arrow_scale,
                    0,
                    0,  # Arrow in +x (chordwise) direction
                    color="green",
                    arrow_length_ratio=0.2,
                    linewidth=2,
                    alpha=0.7,
                    label="Drag" if i == 0 else None,
                )

        if point_loads:
            for pl in point_loads:
                x = pl.get("x", 0)
                y = pl["y"]
                z = pl.get("z", 0)
                Pz = pl.get("Pz", 0)
                ax.quiver(
                    x,
                    y,
                    z,  # base at (x, y, z)
                    0,
                    0,
                    Pz / arrow_scale,  # direction: vertical, scaled for visibility
                    color="red",
                    arrow_length_ratio=0.2,
                    linewidth=3,
                    alpha=0.9,
                    label="Point Load",
                )

        # --- Plot connectors ---
        if connector_sections and connector_stresses_per_section:
            for conn, stresses in zip(
                connector_sections, connector_stresses_per_section
            ):
                x = conn.get("x", 0)
                y = conn.get("y", 0)
                z = conn.get("z", 0)
                section = conn.get("section", None)
                if section:
                    for boom, stress in zip(section.booms, stresses):
                        color = cmap(norm(stress))
                        ax.scatter(
                            x + boom.x,
                            y,
                            z + boom.y,
                            color=color,
                            s=boom.area * 1e6 * 20,
                            marker="o",
                            alpha=0.8,
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
        if drag_per_section:
            handles.append(plt.Line2D([0], [0], color="green", lw=2, label="Drag"))
        if weight_per_section:
            handles.append(plt.Line2D([0], [0], color="blue", lw=2, label="Weight"))
        if connector_sections:
            handles.append(
                plt.Line2D(
                    [0],
                    [0],
                    color="magenta",
                    marker="D",
                    linestyle="",
                    label="Connector",
                )
            )
        if handles:
            ax.legend(handles=handles, loc="upper left")

        ax.xaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.yaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.zaxis.set_major_locator(ticker.MaxNLocator(4))

        ax.set_xlim(-1, 1)
        ax.set_ylim(0, self.span / 2)
        ax.set_zlim(-0.2, 0.2)

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
        cmap = plt.colormaps["plasma"]

        for i, (y_pos, section) in enumerate(self.sections):
            dz = vertical_deflections[i]
            color = cmap(norm(dz))
            for boom in section.booms:
                x = boom.x
                y = y_pos
                z = boom.y + dz  # add deflection to original z
                ax.scatter(x, y, z, color=color, s=boom.area * 1e6 * 20)

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

        ax.xaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.yaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.zaxis.set_major_locator(ticker.MaxNLocator(4))

        ax.set_xlim(-1, 1)
        ax.set_ylim(0, self.span / 2)
        ax.set_zlim(-0.2, 0.2)

        plt.tight_layout()
        plt.show()


class TailStructure:
    def __init__(
        self,
        horiz_span: float,
        horiz_chord: float,
        vert_span: float,
        vert_chord: float,
        n_sections: int,
        horiz_section: IdealizedSection,
        vert_section: IdealizedSection,
        x0: float = 0.0,  # x-location of tail root (relative to fuselage)
        z0: float = 0.0,  # z-location of tail root (relative to fuselage)
    ):
        self.horiz_span = horiz_span
        self.horiz_chord = horiz_chord
        self.vert_span = vert_span
        self.vert_chord = vert_chord
        self.n_sections = n_sections
        self.horiz_section = horiz_section
        self.vert_section = vert_section
        self.x0 = x0
        self.z0 = z0
        self.horiz_sections = self.generate_horizontal_sections()
        self.vert_sections = self.generate_vertical_sections()

    def generate_horizontal_sections(self):
        # Sweep horizontal stabilizer along y (spanwise), centered at z0
        sections = []
        for i in range(self.n_sections):
            y = -self.horiz_span / 2 + i * self.horiz_span / (self.n_sections - 1)
            pos = np.array([self.x0, y, self.z0])
            sections.append((pos, self.horiz_section))
        return sections

    def generate_vertical_sections(self):
        # Sweep vertical stabilizer along z (upwards), centered at y=0
        sections = []
        for i in range(self.n_sections):
            z = self.z0 + i * self.vert_span / (self.n_sections - 1)
            pos = np.array([self.x0, 0.0, z])
            sections.append((pos, self.vert_section))
        return sections

    def mass(self):
        # Sum mass of all sections (approximate)
        horiz_mass = sum(
            sec.mass(self.horiz_span / (self.n_sections - 1))
            for _, sec in self.horiz_sections
        )
        vert_mass = sum(
            sec.mass(self.vert_span / (self.n_sections - 1))
            for _, sec in self.vert_sections
        )
        return horiz_mass + vert_mass

    def compute_weight_distribution(self) -> list[float]:
        """
        Returns a list of weight per section [N] for the tail (horizontal + vertical),
        based on actual structural mass distribution.
        """
        dz_h = self.horiz_span / (self.n_sections - 1)
        dz_v = self.vert_span / (self.n_sections - 1)
        weight_per_section_h = [sec.mass(dz_h) * g for _, sec in self.horiz_sections]
        weight_per_section_v = [sec.mass(dz_v) * g for _, sec in self.vert_sections]
        return weight_per_section_h, weight_per_section_v

    def compute_bending_stresses(
        self, horiz_loads: list[float], vert_loads: list[float]
    ):
        """
        Compute bending stresses for both horizontal and vertical stabilizers.
        horiz_loads: list of distributed loads (N) for horizontal tail (per section)
        vert_loads: list of distributed loads (N) for vertical tail (per section)
        Returns: (horiz_stresses_per_section, vert_stresses_per_section)
        """
        # Horizontal tail: bending about z (like a mini-wing)
        horiz_stresses_per_section = []
        for i, (pos, section) in enumerate(self.horiz_sections):
            # Simple cantilever: moment at section = sum of loads outboard * arm
            moment = 0
            for j in range(i, len(self.horiz_sections)):
                y_out = self.horiz_sections[j][0][1]
                y_here = pos[1]
                arm = y_out - y_here
                moment += horiz_loads[j] * arm
            stresses = section.bending_stress(Mx=moment, My=0)
            horiz_stresses_per_section.append(stresses)

        # Vertical tail: bending about y (vertical cantilever)
        vert_stresses_per_section = []
        for i, (pos, section) in enumerate(self.vert_sections):
            moment = 0
            for j in range(i, len(self.vert_sections)):
                z_out = self.vert_sections[j][0][2]
                z_here = pos[2]
                arm = z_out - z_here
                moment += vert_loads[j] * arm
            stresses = section.bending_stress(Mx=0, My=moment)
            vert_stresses_per_section.append(stresses)

        return horiz_stresses_per_section, vert_stresses_per_section

    def plot_3d_tail(
        self,
        horiz_stresses_per_section: list[list[float]],
        vert_stresses_per_section: list[list[float]],
        arrow_scale: float,
        horiz_loads: list[float] = None,
        vert_loads: list[float] = None,
    ):
        import matplotlib.colors as mcolors
        from matplotlib import cm

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.view_init(azim=210)

        # Combine all stresses for color normalization
        all_stresses = [s for sec in horiz_stresses_per_section for s in sec] + [
            s for sec in vert_stresses_per_section for s in sec
        ]
        norm = mcolors.Normalize(vmin=min(all_stresses), vmax=max(all_stresses))
        cmap = plt.colormaps["viridis"]

        # Plot horizontal tail booms (spanwise in y)
        for i, (pos, section) in enumerate(self.horiz_sections):
            stresses = horiz_stresses_per_section[i]
            for boom, stress in zip(section.booms, stresses):
                color = cmap(norm(stress))
                ax.scatter(
                    pos[0] + boom.x,  # x: along fuselage
                    pos[1] + boom.y,  # y: sideways (span)
                    pos[2],  # z: vertical (fixed)
                    color=color,
                    s=boom.area * 1e6 * 20,  # factor for visibility
                )

        # Plot vertical tail booms (spanwise in z)
        for i, (pos, section) in enumerate(self.vert_sections):
            stresses = vert_stresses_per_section[i]
            for boom, stress in zip(section.booms, stresses):
                color = cmap(norm(stress))
                ax.scatter(
                    pos[0] + boom.x,  # x: along fuselage
                    pos[1],  # y: sideways (fixed, usually 0)
                    pos[2] + boom.y,  # z: upwards (span)
                    color=color,
                    s=boom.area * 1e6 * 20,  # factor for visibility
                )

        # Optionally, plot load arrows
        if horiz_loads:
            for i, (pos, section) in enumerate(self.horiz_sections):
                x_c = np.mean([boom.x for boom in section.booms])
                z_c = np.mean([boom.y for boom in section.booms])
                load = horiz_loads[i]
                ax.quiver(
                    pos[0] + x_c,
                    pos[1] + z_c,
                    pos[2],
                    0,
                    0,
                    load / arrow_scale,
                    color="red",
                    arrow_length_ratio=0.2,
                    linewidth=2,
                    alpha=0.7,
                    label="Horiz Load" if i == 0 else None,
                )
        if vert_loads:
            for i, (pos, section) in enumerate(self.vert_sections):
                x_c = np.mean([boom.x for boom in section.booms])
                y_c = np.mean([boom.y for boom in section.booms])
                load = vert_loads[i]
                ax.quiver(
                    pos[0] + x_c,
                    pos[1],
                    pos[2] + y_c,
                    0,
                    load / arrow_scale,
                    0,
                    color="blue",
                    arrow_length_ratio=0.2,
                    linewidth=2,
                    alpha=0.7,
                    label="Vert Load" if i == 0 else None,
                )

        ax.set_title("3D Tail Structure (Booms colored by Bending Stress)")
        ax.set_xlabel("x [m] (Fuselage length, nose to tail)")
        ax.set_ylabel("y [m] (Sideways, starboard +)")
        ax.set_zlabel("z [m] (Upwards)")
        ax.grid(True)

        mappable = cm.ScalarMappable(norm=norm, cmap=cmap)
        mappable.set_array(all_stresses)
        cbar = plt.colorbar(mappable, ax=ax, pad=0.1)
        cbar.set_label("Bending Stress [Pa]")

        ax.xaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.yaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.zaxis.set_major_locator(ticker.MaxNLocator(4))

        ax.set_xlim(0.5, 1.5)
        ax.set_ylim(-0.5, 0.5)
        ax.set_zlim(-0.1, 0.3)

        plt.tight_layout()
        plt.show()


# === ANALYSIS & UTILITY FUNCTIONS ===


def find_critical_stress(
    sections,
    stresses_per_section,
    shear_stresses_per_section=None,
    torsional_stresses_per_section=None,
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

            if torsional_stresses_per_section:
                tau_torsion = torsional_stresses_per_section[i][j]
                tau_allow = boom.material.tau_max
                tau_ratio = abs(tau_torsion) / tau_allow if tau_allow else 0
                if tau_ratio > critical["max_ratio"]:
                    critical.update(
                        {
                            "max_ratio": tau_ratio,
                            "section_idx": i,
                            "boom_idx": j,
                            "stress": tau_torsion,
                            "allowable": tau_allow,
                            "type": "torsion",
                        }
                    )
    return critical


def euler_buckling_stress(E, K, L, r):
    # buckling mode: one free end, the other fixed
    # r = 2
    return (np.pi**2 * E) / ((K * L / r) ** 2)


# TEST TEST TEST TEST
def size_wing_for_min_mass(
    wing: "WingStructure",
    lift_per_section: list[float],
    weight_per_section: list[float],
    shear_thickness: float,  # = 0.002,
    safety_factor: float,  # = 2.0,
    area_scale_start: float = 3.0,  # Start with a large, safe scale
    area_scale_step: float = 0.02,
    min_scale: float = 0.01,
    max_iter: int = 200,
    wing_point_loads: list[dict] = None,
):
    original_areas = [
        [boom.area for boom in section.booms] for _, section in wing.sections
    ]
    area_scale = area_scale_start
    last_safe = None

    for _ in range(max_iter):
        # Reset all boom areas to original, then scale
        for orig_areas, (_, section) in zip(original_areas, wing.sections):
            for boom, orig_area in zip(section.booms, orig_areas):
                boom.area = orig_area * area_scale

            # Recompute section properties after area change
            section.centroid_x, section.centroid_y = section.calc_centroid()
            section.Ixx, section.Iyy, section.Ixy = section.calc_moments()

        # Recompute loads and stresses
        total_vertical_load = [
            L - W for L, W in zip(lift_per_section, weight_per_section)
        ]
        moments_x, stresses_per_section = wing.compute_bending_stresses(
            total_vertical_load
        )

        # --- Add point loads to the correct sections ---
        if wing_point_loads:
            for pl in wing_point_loads:
                idx = min(
                    range(len(wing.sections)),
                    key=lambda i: abs(wing.sections[i][0] - pl["y"]),
                )
                total_vertical_load[idx] += pl.get("Pz", 0)

        moments_x, stresses_per_section = wing.compute_bending_stresses(
            total_vertical_load
        )

        # Shear stresses
        shear_forces = []
        running_shear = 0.0
        for net_load in reversed(total_vertical_load):
            running_shear += net_load
            shear_forces.insert(0, running_shear)
        shear_stresses_per_section = []
        for (y, sec), Vz in zip(wing.sections, shear_forces):
            shear_stresses = sec.shear_stress(Vz=Vz, thickness=shear_thickness)
            shear_stresses_per_section.append(shear_stresses)

        # Find critical stress (yield/shear)
        critical = find_critical_stress(
            [sec for _, sec in wing.sections],
            stresses_per_section,
            shear_stresses_per_section,
        )
        allowable_with_sf = critical["allowable"] / safety_factor
        utilization_with_sf = (
            abs(critical["stress"]) / allowable_with_sf if allowable_with_sf else 0
        )

        # Buckling check for the critical boom
        dy = wing.dy
        critical_boom = wing.sections[critical["section_idx"]][1].booms[
            critical["boom_idx"]
        ]
        E = critical_boom.material.E
        K = 2.0  # free-fixed
        L = dy
        A = critical_boom.area
        r = np.sqrt(A / np.pi)
        sigma_cr = euler_buckling_stress(E, K, L, r)
        sigma_cr_with_sf = sigma_cr / safety_factor
        buckling_utilization = (
            abs(critical["stress"]) / sigma_cr_with_sf if sigma_cr_with_sf else 0
        )

        # Check both criteria
        if utilization_with_sf < 1.0 and buckling_utilization < 1.0:
            last_safe = (area_scale, sum(sec.mass(dy) for _, sec in wing.sections) * 2)
            area_scale -= area_scale_step
            if area_scale < min_scale:
                break
        else:
            # The previous scale was the last safe one
            if last_safe is not None:
                return last_safe[1], last_safe[0]
            else:
                raise RuntimeError(
                    "Initial area is not strong enough! Increase area_scale_start."
                )

        # print(
        #     f"Area scale: {area_scale:.3f}, Utilization (yield): {utilization_with_sf:.3f}, "
        #     f"Utilization (buckling): {buckling_utilization:.3f}"
        # )

    if last_safe is not None:
        return last_safe[1], last_safe[0]
    raise RuntimeError("Failed to find a safe structure within max_iter iterations.")


# TEST TEST TEST TEST
def size_fuselage_for_min_mass(
    fuselage: "FuselageStructure",
    distributed_loads: list[float],
    shear_thickness: float,  # = 0.002,
    safety_factor: float,  # = 2.0,
    area_scale_start: float = 30.0,
    area_scale_step: float = 0.02,
    min_scale: float = 0.01,
    max_iter: int = 200,
    fuselage_point_loads: list[dict] = None,
    min_boom_area: float = 1e-5,
):
    """
    Sizing loop for minimal fuselage mass, considering distributed and point loads.
    """
    original_areas = [
        [boom.area for boom in section.booms] for _, section in fuselage.sections
    ]
    area_scale = area_scale_start
    last_safe = None

    for _ in range(max_iter):
        # Reset all boom areas to original, then scale
        for orig_areas, (_, section) in zip(original_areas, fuselage.sections):
            for boom, orig_area in zip(section.booms, orig_areas):
                boom.area = max(orig_area * area_scale, min_boom_area)
            section.centroid_x, section.centroid_y = section.calc_centroid()
            section.Ixx, section.Iyy, section.Ixy = section.calc_moments()

        dz = fuselage.dz
        # distributed_loads = [section.mass(dz) * g for _, section in fuselage.sections]
        self_weight_per_section = [section.mass(dz) * g for _, section in fuselage.sections]
        # Add self-weight to the external distributed loads
        total_distributed_loads = [ext + selfw for ext, selfw in zip(distributed_loads, self_weight_per_section)]

        # --- Bending moments and stresses ---
        Mz_per_section, My_per_section = (
            fuselage.compute_bending_moments_with_distributed_and_point_loads(
                total_distributed_loads, fuselage_point_loads or []
            )
        )
        stresses_per_section = fuselage.compute_bending_stresses(
            Mz_per_section, My_per_section
        )

        # --- Shear stresses ---
        # For shear, add point loads to distributed loads at the correct section
        shear_vertical_load = distributed_loads.copy()
        if fuselage_point_loads:
            for pl in fuselage_point_loads:
                idx = min(
                    range(len(fuselage.sections)),
                    key=lambda i: abs(fuselage.sections[i][0] - pl["x"]),
                )
                shear_vertical_load[idx] += pl.get("Pz", 0)

        # Compute shear forces (from tail to nose)
        shear_forces = []
        running_shear = 0.0
        for net_load in reversed(shear_vertical_load):
            running_shear += net_load
            shear_forces.insert(0, running_shear)

        shear_stresses_per_section = []
        for (x, sec), Vz in zip(fuselage.sections, shear_forces):
            shear_stresses = sec.shear_stress(Vz=Vz, thickness=shear_thickness)
            shear_stresses_per_section.append(shear_stresses)

        # After computing torsional moments:
        torsional_moments = fuselage.compute_torsional_moments(
            fuselage_point_loads or []
        )
        torsional_stresses_per_section = []
        for (x, sec), Mx in zip(fuselage.sections, torsional_moments):
            tau_torsion = sec.torsional_shear_stress(Mx)
            # For compatibility with find_critical_stress, make a list for each boom
            torsional_stresses_per_section.append([tau_torsion] * len(sec.booms))

        # --- Find critical stress (yield/shear) ---
        critical = find_critical_stress(
            [sec for _, sec in fuselage.sections],
            stresses_per_section,
            shear_stresses_per_section,
            torsional_stresses_per_section,
        )
        allowable_with_sf = critical["allowable"] / safety_factor
        utilization_with_sf = (
            abs(critical["stress"]) / allowable_with_sf if allowable_with_sf else 0
        )

        # --- Buckling check for the critical boom ---
        dz = fuselage.dz
        critical_boom = fuselage.sections[critical["section_idx"]][1].booms[
            critical["boom_idx"]
        ]
        E = critical_boom.material.E
        K = 2.0  # free-fixed
        L = dz
        A = critical_boom.area
        r = np.sqrt(A / np.pi)
        sigma_cr = euler_buckling_stress(E, K, L, r)
        sigma_cr_with_sf = sigma_cr / safety_factor
        buckling_utilization = (
            abs(critical["stress"]) / sigma_cr_with_sf if sigma_cr_with_sf else 0
        )

        # --- Check both criteria ---
        if utilization_with_sf < 1.0 and buckling_utilization < 1.0:
            last_safe = (area_scale, sum(sec.mass(dz) for _, sec in fuselage.sections))
            # print(f"[DEBUG] Area scale: {area_scale:.3f}, Utilization: {utilization_with_sf:.3f}, Buckling: {buckling_utilization:.3f}")
            area_scale -= area_scale_step
            if area_scale < min_scale:
                # print("[DEBUG] Area scale hit minimum allowed value.")
                break
        else:
            if last_safe is not None:
                return last_safe[1], last_safe[0]
            else:
                raise RuntimeError(
                    "Initial area is not strong enough! Increase area_scale_start."
                )

    if last_safe is not None:
        return last_safe[1], last_safe[0]
    raise RuntimeError("Failed to find a safe structure within max_iter iterations.")


def get_fuselage_dimensions(case: int):
    """
    Returns (width, height, length) for the fuselage based on the selected case.
    All dimensions in meters.
    """
    # FIX FIX FIX PLACEHOLDERS (realistic)
    # Pull these values from somewhere in the python code, from Simonas
    if case == 1:
        width = 0.675
        height = 0.450
        length = 0.630
    elif case == 2:
        width = 0.675
        height = 0.270
        length = 1.260
    else:
        raise ValueError("Invalid case. Choose 1 or 2.")

    clearance = 0.2  # 20% clearance
    width = width * (1 + clearance)
    height = height * (1 + clearance)
    length = length * (1 + clearance)
    return width, height, length


def get_fuselage_payload_weights(case: int):
    """
    Returns loads for the fuselage based on the selected case.
    All dimensions in N.
    """
    battery_weight = 0.5 * g
    sensors_weight = 0.1 * g
    computing_module_weight = 0.1 * g
    miscellaneous_weight = 0.1 * g
    # FIX FIX FIX, from Simonas, Andreas (?) and Ishaan
    # These values are placeholders and should be replaced with actual values

    if case == 1:
        pizza_weight_1 = 2.5 * g
        pizza_weight_2 = 0 * g
        mechanisms_weight = 1.31 * g
        payload_insulator_weight = 0.41 * g
    elif case == 2:
        pizza_weight_1 = 1.25 * g
        pizza_weight_2 = 1.25 * g
        mechanisms_weight = 1.58 * g
        payload_insulator_weight = 0.56 * g
    else:
        raise ValueError("Invalid case. Choose 1 or 2.")

    return (
        battery_weight,
        sensors_weight,
        computing_module_weight,
        miscellaneous_weight,
        pizza_weight_1,
        pizza_weight_2,
        mechanisms_weight,
        payload_insulator_weight,
    )


def size_tail_for_min_mass(
    tail: "TailStructure",
    horiz_loads: list[float],
    vert_loads: list[float],
    shear_thickness: float,  # = 0.002,
    safety_factor: float,  # = 2.0,
    area_scale_start: float = 3.0,
    area_scale_step: float = 0.02,
    min_scale: float = 0.01,
    max_iter: int = 200,
    min_boom_area: float = 1e-5,
):
    original_horiz_areas = [
        [boom.area for boom in section.booms] for _, section in tail.horiz_sections
    ]
    original_vert_areas = [
        [boom.area for boom in section.booms] for _, section in tail.vert_sections
    ]
    area_scale = area_scale_start
    last_safe = None

    for _ in range(max_iter):
        # Scale horizontal tail booms
        for orig_areas, (_, section) in zip(original_horiz_areas, tail.horiz_sections):
            for boom, orig_area in zip(section.booms, orig_areas):
                boom.area = max(orig_area * area_scale, min_boom_area)
            section.centroid_x, section.centroid_y = section.calc_centroid()
            section.Ixx, section.Iyy, section.Ixy = section.calc_moments()
        # Scale vertical tail booms
        for orig_areas, (_, section) in zip(original_vert_areas, tail.vert_sections):
            for boom, orig_area in zip(section.booms, orig_areas):
                boom.area = max(orig_area * area_scale, min_boom_area)
            section.centroid_x, section.centroid_y = section.calc_centroid()
            section.Ixx, section.Iyy, section.Ixy = section.calc_moments()

        # Compute stresses
        h_stress, v_stress = tail.compute_bending_stresses(horiz_loads, vert_loads)

        # Find critical stress
        critical_h = find_critical_stress(
            [sec for _, sec in tail.horiz_sections], h_stress
        )
        critical_v = find_critical_stress(
            [sec for _, sec in tail.vert_sections], v_stress
        )
        critical = max(
            [critical_h, critical_v],
            key=lambda c: (abs(c["stress"] / c["allowable"]) if c["allowable"] else 0),
        )

        allowable_with_sf = critical["allowable"] / safety_factor
        utilization_with_sf = (
            abs(critical["stress"]) / allowable_with_sf if allowable_with_sf else 0
        )

        # Buckling check (use horizontal tail as reference)
        dz = tail.horiz_span / (tail.n_sections - 1)
        critical_boom = tail.horiz_sections[critical["section_idx"]][1].booms[
            critical["boom_idx"]
        ]
        E = critical_boom.material.E
        K = 2.0
        L = dz
        A = critical_boom.area
        r = np.sqrt(A / np.pi)
        sigma_cr = euler_buckling_stress(E, K, L, r)
        sigma_cr_with_sf = sigma_cr / safety_factor
        buckling_utilization = (
            abs(critical["stress"]) / sigma_cr_with_sf if sigma_cr_with_sf else 0
        )

        if utilization_with_sf < 1.0 and buckling_utilization < 1.0:
            last_safe = (area_scale, tail.mass())
            area_scale -= area_scale_step
            if area_scale < min_scale:
                break
        else:
            if last_safe is not None:
                return last_safe[1], last_safe[0]
            else:
                raise RuntimeError(
                    "Initial area is not strong enough! Increase area_scale_start."
                )

    if last_safe is not None:
        return last_safe[1], last_safe[0]
    raise RuntimeError("Failed to find a safe structure within max_iter iterations.")


# === MAIN EXECUTION ===


def run_structure_analysis(
    drone: Drone,
    prop_connection: str = "wing",
    # prop_connection: "wing" or "fuselage"
    fuselage_case=2,  # or 2, (1 for chubby, 2 for elongated fuselage)
    banked=False,  # Set to False for normal cruise, True for banked case
    plot=False,
):
    from prelim_des.maneuvre_envelope import plot_maneuver_and_gust_envelope
    
    n_max = plot_maneuver_and_gust_envelope(drone, plot=False)
    
    # FIX FIX FIX, those values are educated guesses, but what values should they have? These might be correct
    SAFETY_FACTOR = 1.5 * n_max
    shear_thickness = 0.002  # meters, skin thickness for shear stress calculations
    min_boom_area = 1e-5  # m^2, minimum area for a boom

    # Create root cross-section
    # FIX THIS -> call correct values
    root_section = create_rectangular_section(
        width=0.6,
        height=0.072,
        n_regular_booms=12,
        spar_cap_area=1e-4,
        regular_boom_area=5e-5,
        material_name="al_6061_t4",
        materials=materials,
    )

    wing = WingStructure(
        n_sections=10,
        root_section=root_section,
        drone=drone,
    )

    # For example:
    # Create fuselage cross-section

    dy = wing.dy
    # b_half = wing.span / 2
    b = wing.span

    fuselage_width, fuselage_height, fuselage_length = get_fuselage_dimensions(
        fuselage_case
    )

    # FIX FIX FIX

    fuselage_root_section = create_rectangular_section(
        width=fuselage_width,
        height=fuselage_height,
        n_regular_booms=12,
        spar_cap_area=1e-4,
        regular_boom_area=5e-5,
        material_name="al_6061_t4",
        materials=materials,
    )

    fuselage = FuselageStructure(
        length=fuselage_length,
        n_sections=20,
        root_section=fuselage_root_section,
        taper_ratio=1.0,
    )

    # Store initial wing and fuselage areas
    original_wing_areas = [
        [boom.area for boom in section.booms] for _, section in wing.sections
    ]
    original_fuselage_areas = [
        [boom.area for boom in section.booms] for _, section in fuselage.sections
    ]

    section_positions = [x for x, _ in fuselage.sections]
    Mz_distributed = [0] * fuselage.n_sections
    My_distributed = [0] * fuselage.n_sections

    (
        battery_weight,
        sensors_weight,
        computing_module_weight,
        miscellaneous_weight,
        pizza_weight_1,
        pizza_weight_2,
        mechanisms_weight,
        payload_insulator_weight,
    ) = get_fuselage_payload_weights(fuselage_case)

    lift_per_section = [
        elliptical_lift_distribution(y, drone) * dy for y, _ in wing.sections
    ]
    weight_per_section = [sec.mass(dy) * g for _, sec in wing.sections]
    drag_per_section = [
        constant_drag_distribution(drone) * dy for y, _ in wing.sections
    ]

    # Calculate total forces on the wing
    total_lift = sum(lift_per_section)
    total_weight = sum(weight_per_section)
    total_drag = sum(drag_per_section)

    net_vertical_force = total_lift - total_weight  # Pz (upwards positive)
    net_drag_force = total_drag

    # Calculate moments at the two connection points (left and right)
    connection_points = [
        {
            "x": 0.9 * fuselage_length,
            "y": 0.5 * fuselage_width,
            "z": 0.5 * fuselage_height,
        },
        {
            "x": 0.9 * fuselage_length,
            "y": -0.5 * fuselage_width,
            "z": 0.5 * fuselage_height,
        },
    ]

    # For each connection, sum moments from all sections
    for conn in connection_points:
        Mz = 0.0  # Moment about z-axis (from vertical forces, i.e., lift-weight)
        My = 0.0  # Moment about y-axis (from drag)
        for (y_pos, _), lv, drag in zip(
            wing.sections,
            [l - w for l, w in zip(lift_per_section, weight_per_section)],
            drag_per_section,
        ):
            # Moment arm is spanwise distance from section to connection point
            arm_y = conn["y"] - y_pos  # y_conn - y_section
            # Moment from vertical force (about z): Mz += (lift-weight) * arm_y * dy
            Mz += lv * arm_y * dy
            # Moment from drag (about y): My += drag * arm_y * dy
            My += drag * arm_y * dy
        conn["Mz"] = Mz
        conn["My"] = My

        # FIX FIX FIX

    if fuselage_case == 1:
        point_loads = [
            {
                "x": conn["x"],
                "y": conn["y"],
                "z": conn["z"],
                "Pz": net_vertical_force,
                "Px": net_drag_force,
                "Mz": conn["Mz"],
                "My": conn["My"],
            }
            for conn in connection_points
        ]
        [
            {"x": 0.5 * fuselage_length, "y": 0.0, "z": 0.0, "Pz": -battery_weight},
            {"x": 0.05 * fuselage_length, "y": 0.0, "z": 0.0, "Pz": -sensors_weight},
            {
                "x": 0.5 * fuselage_length,
                "y": 0.0,
                "z": 0.0,
                "Pz": -computing_module_weight,
            },
            {
                "x": 0.5 * fuselage_length,
                "y": 0.0,
                "z": 0.0,
                "Pz": -miscellaneous_weight,
            },
            {"x": 0.5 * fuselage_length, "y": 0.0, "z": 0.0, "Pz": -pizza_weight_1},
            {"x": 0.45 * fuselage_length, "y": 0.0, "z": 0.0, "Pz": -mechanisms_weight},
            {
                "x": 0.5 * fuselage_length,
                "y": 0.0,
                "z": 0.0,
                "Pz": -payload_insulator_weight,
            },  # placeholder, all of them shall be changed to actual values
        ]
        # FIX FIX FIX
    elif fuselage_case == 2:
        # Example: place load at center of the longer cargo bay for case 2
        point_loads = [
            {
                "x": 0.9 * fuselage_length,
                "y": 0.5 * fuselage_width,
                "z": 0.5 * fuselage_height,
                "Pz": net_vertical_force,
                "Px": net_drag_force,
            },
            {
                "x": 0.9 * fuselage_length,
                "y": -0.5 * fuselage_width,
                "z": 0.5 * fuselage_height,
                "Pz": net_vertical_force,
                "Px": net_drag_force,
            },
            {"x": 0.5 * fuselage_length, "y": 0.0, "z": 0.0, "Pz": -battery_weight},
            {"x": 0.05 * fuselage_length, "y": 0.0, "z": 0.0, "Pz": -sensors_weight},
            {
                "x": 0.5 * fuselage_length,
                "y": 0.0,
                "z": 0.0,
                "Pz": -computing_module_weight,
            },
            {
                "x": 0.5 * fuselage_length,
                "y": 0.0,
                "z": 0.0,
                "Pz": -miscellaneous_weight,
            },
            {"x": 0.35 * fuselage_length, "y": 0.0, "z": 0.0, "Pz": -pizza_weight_1},
            {"x": 0.65 * fuselage_length, "y": 0.0, "z": 0.0, "Pz": -pizza_weight_2},
            {"x": 0.45 * fuselage_length, "y": 0.0, "z": 0.0, "Pz": -mechanisms_weight},
            {
                "x": 0.5 * fuselage_length,
                "y": 0.0,
                "z": 0.0,
                "Pz": -payload_insulator_weight,
            },  # placeholder, all of them shall be changed to actual values
        ]
    else:
        raise ValueError("Invalid fuselage case. Choose 1 or 2.")

    fuselage_prop_loads = [
        {"x": 2.5 * root_section.width, "y": b / 2 / 3, "z": 0.0, "Pz": 450 / 4},
        {"x": -1.5 * root_section.width, "y": b / 2 / 3, "z": 0.0, "Pz": 450 / 4},
        {"x": 2.5 * root_section.width, "y": -b / 2 / 3, "z": 0.0, "Pz": 450 / 4},
        {"x": -1.5 * root_section.width, "y": -b / 2 / 3, "z": 0.0, "Pz": 450 / 4},
    ]
    wing_prop_loads = [
        {"x": 1.5 * root_section.width, "y": b / 2 / 3, "z": 0.0, "Pz": 450 / 4},
        {"x": -1.5 * root_section.width, "y": b / 2 / 3, "z": 0.0, "Pz": 450 / 4},
    ]

    if prop_connection == "wing":
        wing_point_loads = wing_prop_loads
        fuselage_point_loads = []  # No fuselage loads in this case

    elif prop_connection == "fuselage":
        fuselage_point_loads = fuselage_prop_loads
        wing_point_loads = []

    else:
        raise ValueError("prop_connection must be 'wing' or 'fuselage'")

    all_fuselage_point_loads = point_loads + fuselage_point_loads
    Mz_per_section, My_per_section = fuselage.compute_bending_moments_with_point_loads(
        all_fuselage_point_loads
    )

    fuselage_stresses_per_section = fuselage.compute_bending_stresses(
        Mz_per_section, My_per_section
    )

    # After computing fuselage_stresses_per_section

    # Compute internal shear force for the fuselage (from tail to nose)
    shear_forces_fuselage = []
    running_shear = 0.0
    # Use the same point/distributed loads as in your moment calculation
    # Compute net vertical load per section (sum of all Pz from point_loads and fuselage_point_loads)
    for i in reversed(range(len(fuselage.sections))):
        x_pos, section = fuselage.sections[i]
        # Sum all Pz loads applied at this section
        net_load = 0.0
        for pl in point_loads + fuselage_point_loads:
            if abs(pl["x"] - x_pos) < 1e-6:  # or use a tolerance
                net_load += pl.get("Pz", 0)
        running_shear += net_load
        shear_forces_fuselage.insert(0, running_shear)

    # Compute shear stresses per section for the fuselage
    shear_stresses_fuselage = []
    for (x, sec), Vz in zip(fuselage.sections, shear_forces_fuselage):
        shear_stresses = sec.shear_stress(Vz=Vz, thickness=0.002)
        shear_stresses_fuselage.append(shear_stresses)

    # fuselage.plot_3d_fuselage(fuselage_stresses_per_section, point_loads=all_fuselage_point_loads)

    # Compute distributed weight per section
    fuselage_weight_per_section = fuselage.compute_weight_distribution()

    # Call the sizing function
    # FIX FIX FIX IMPLEMENT SHEAR THICKNESS SOMEWHERE
    # min_fuselage_mass, fuselage_area_scale = size_fuselage_for_min_mass(
    #     fuselage,
    #     distributed_loads=fuselage_weight_per_section,
    #     shear_thickness=0.002,
    #     safety_factor=SAFETY_FACTOR,
    #     area_scale_start=10.0,
    #     area_scale_step=0.02,
    #     min_scale=0.01,
    #     max_iter=200,
    #     fuselage_point_loads=all_fuselage_point_loads,
    #     min_boom_area=1e-5,
    # )

    # print(f"Minimal safe fuselage mass: {min_fuselage_mass:.2f} kg (area scale factor: {fuselage_area_scale:.2f})")

    # --- Model a connector cross section at the propeller load point ---
    # Choose a location (y_conn) where the load is applied
    y_conn = b / 2 / 3  # same as the y of the point load

    # Define a small rectangle (e.g., 40mm x 20mm)
    conn_width = 0.04  # [m]
    conn_height = 0.02  # [m]
    conn_area = 1e-5  # [m^2] per boom (example)
    conn_material = "al_6061_t4"

    # Create the connector cross section (4 booms, rectangle)
    connector_section = create_rectangular_section(
        width=conn_width,
        height=conn_height,
        n_regular_booms=0,  # only corners
        spar_cap_area=conn_area,
        regular_boom_area=0,
        material_name=conn_material,
        materials=materials,
    )

    # Optionally, plot the connector cross section
    # connector_section.plot_section()

    # --- Model the connector as a structure from point load to wing surface ---
    x_conn = 1.5 * root_section.width  # same as in wing_point_loads
    z_conn = 0.0
    # Find the closest boom in the root section to the connector load point (in x-z plane)
    boom_positions = np.array([[boom.x, boom.y] for boom in root_section.booms])
    conn_point = np.array([x_conn, z_conn])
    distances = np.linalg.norm(boom_positions - conn_point, axis=1)
    closest_boom_idx = np.argmin(distances)
    min_dist = distances[closest_boom_idx]
    connector_length = min_dist  # [m]
    connector_mass = connector_section.mass(segment_length=connector_length)
    # print(f"Connector mass (length={connector_length} m): {connector_mass:.4f} kg")

    # Start at point load, end at closest boom on wing surface
    start_point = (x_conn, y_conn, z_conn)
    end_boom = root_section.booms[closest_boom_idx]
    end_point = (end_boom.x, y_conn, end_boom.y)

    # Create the connector structure (e.g., 10 sections)
    connector_struct = ConnectorStructure(
        length=connector_length,
        n_sections=10,
        root_section=connector_section,
        start_point=start_point,
        end_point=end_point,
    )

    # Compute connector stresses (no bending moment for now, or set as needed)
    connector_stresses_per_section = []
    for pos, section in connector_struct.sections:
        # If you want to compute actual bending moments, replace Mx=... as needed
        stresses = section.bending_stress(Mx=0, My=0)
        connector_stresses_per_section.append(stresses)

    # For plotting: create a list of connector sections along its length
    connector_sections = []
    for pos, section in connector_struct.sections:
        connector_sections.append(
            {
                "x": pos[0],
                "y": pos[1],
                "z": pos[2],
                "section": section,
            }
        )

    # --- Banked flight option ---
    # FIX FIX FIX PULL THEM PROPERLY - FIXED I think? It doesn't seem like we even use this load factor anywhere
    phi_deg = 30  # Bank angle in degrees
    phi_rad = np.radians(phi_deg)
    n_load = 1 / np.cos(phi_rad) if banked else 1.0

    CL_max = drone.aero.CL_max
    V_max = toml["config"]["mission"]["max_velocity"]
    L_total = drone.aero.lift(V_max, CL_max)  # Total lift at max velocity
    L_total_banked = L_total * n_load

    # Use correct total lift for the selected case
    lift_per_section = []
    for y, _ in wing.sections:
        L_prime = elliptical_lift_distribution(y, drone)
        lift = L_prime * dy
        lift_per_section.append(lift)

    weight_per_section = [sec.mass(dy) * g for _, sec in wing.sections]
    total_vertical_load = [
        lift - weight for lift, weight in zip(lift_per_section, weight_per_section)
    ]

    # Add propeller point loads to the correct sections for shear only
    shear_vertical_load = total_vertical_load.copy()
    for pl in wing_point_loads:
        idx = min(
            range(len(wing.sections)), key=lambda i: abs(wing.sections[i][0] - pl["y"])
        )
        shear_vertical_load[idx] += pl.get("Pz", 0)

    # For bending: use only distributed loads and pass point loads separately
    moments_x = wing.compute_bending_moments_with_point_loads(
        total_vertical_load,  # Only distributed loads (no prop loads added)
        wing_point_loads,
    )

    drag_per_section = [
        constant_drag_distribution(drone) * dy for y, _ in wing.sections
    ]

    # Compute internal shear force from tip to root
    shear_forces = []
    running_shear = 0.0
    for net_load in reversed(shear_vertical_load):
        running_shear += net_load
        shear_forces.insert(0, running_shear)

    # Compute shear stresses per section
    shear_stresses_per_section = []
    for (y, sec), Vz in zip(wing.sections, shear_forces):
        shear_stresses = sec.shear_stress(
            Vz=Vz, thickness=shear_thickness
        )  # adjust thickness if needed
        shear_stresses_per_section.append(shear_stresses)

    moments_x = wing.compute_bending_moments_with_point_loads(
        total_vertical_load, wing_point_loads
    )
    stresses_per_section = []
    for i, (y_pos, section) in enumerate(wing.sections):
        stresses = section.bending_stress(Mx=moments_x[i], My=0)
        stresses_per_section.append(stresses)

    # wing.plot_3d_wing(
    # stresses_per_section,
    # lift_per_section,
    # weight_per_section,
    # point_loads=wing_point_loads,
    # drag_per_section=drag_per_section,
    # connector_sections=connector_sections,
    # connector_stresses_per_section=connector_stresses_per_section,)

    # wing.plot_3d_wing(lift_per_section)

    vertical_deflections = wing.compute_vertical_deflections(total_vertical_load)
    # wing.plot_deformed_wing(vertical_deflections)

    # Tail Creation - CHANGE VALUES !!!!!!!!!! --- !!!!!!!!!!! FIX FIX FIX

    horiz_span = 0.6
    horiz_chord = 0.15
    vert_span = 0.25
    vert_chord = 0.12

    horiz_section = create_rectangular_section(
        width=horiz_chord,
        height=0.02,
        n_regular_booms=8,
        spar_cap_area=5e-5,
        regular_boom_area=1e-5,
        material_name="al_6061_t4",
        materials=materials,
    )

    vert_section = create_rectangular_section(
        width=0.02,
        height=vert_chord,
        n_regular_booms=8,
        spar_cap_area=5e-5,
        regular_boom_area=1e-5,
        material_name="al_6061_t4",
        materials=materials,
    )

    tail = TailStructure(
        horiz_span=horiz_span,
        horiz_chord=horiz_chord,
        vert_span=vert_span,
        vert_chord=vert_chord,
        n_sections=10,
        horiz_section=horiz_section,
        vert_section=vert_section,
        x0=1.1 * fuselage_length,  # example: place at rear of fuselage
        z0=0.0,
    )

    arrow_scale = 0.01

    vert_half_idx = tail.n_sections // 2
    z_half = tail.vert_sections[vert_half_idx][0][2]

    vert_loads = [0.0 for _ in range(tail.n_sections)]
    vert_loads[vert_half_idx] = 30.0  # 30 N at halfway up the vertical stabiliser

    horiz_loads = [0.0 for _ in range(tail.n_sections)]
    horiz_loads[0] = 50.0  # 50 N at left tip
    horiz_loads[-1] = 50.0  # 50 N at right tip

    h_stress, v_stress = tail.compute_bending_stresses(horiz_loads, vert_loads)
    if plot:
        tail.plot_3d_tail(
        h_stress,
        v_stress,
        arrow_scale=arrow_scale,
        horiz_loads=horiz_loads,
        vert_loads=vert_loads,
    )

    # --- SIZING FOR BOTH FLIGHT MODES ---

    results = {}

    for flight_mode in ["cruise", "vtol"]:
        point_loads = []
        if flight_mode == "cruise":
            # All lift from wings, no propeller loads
            lift_per_section = [
                elliptical_lift_distribution(y, drone) * dy for y, _ in wing.sections
            ]
            wing_point_loads_mode = []
            fuselage_point_loads_mode = []
            total_lift = sum(lift_per_section)
            wing_reaction = total_lift / 2  # assuming two attach points
            # Use correct x, y, z for your attach points
            fuselage_point_loads_mode = [
                {
                    # FIX FIX FIX
                    "x": 0.9 * fuselage_length,
                    "y": 0.5 * fuselage_width,
                    "z": 0.5 * fuselage_height,
                    "Pz": -wing_reaction,
                },
                {
                    "x": 0.9 * fuselage_length,
                    "y": -0.5 * fuselage_width,
                    "z": 0.5 * fuselage_height,
                    "Pz": -wing_reaction,
                },
            ]

            # After computing tail loads (e.g., horiz_loads, vert_loads)
            # Calculate total vertical force and moment at the root (x0, y0, z0)
            tail_root_x = tail.x0
            tail_root_y = 0.0
            tail_root_z = tail.z0

            # For horizontal tail (bending about z)
            total_horiz_force = sum(horiz_loads)
            moment_horiz = 0.0
            for i, (pos, _) in enumerate(tail.horiz_sections):
                arm = pos[1]  # y-position (spanwise)
                moment_horiz += horiz_loads[i] * arm

            # For vertical tail (bending about y)
            total_vert_force = sum(vert_loads)
            moment_vert = 0.0
            for i, (pos, _) in enumerate(tail.vert_sections):
                arm = pos[2]  # z-position (vertical)
                moment_vert += vert_loads[i] * arm

            # Compute torsional moment from vertical tail loads
            torsion_from_tail = 0.0
            for i, (pos, _) in enumerate(tail.vert_sections):
                z_arm = pos[2] - tail_root_z  # tail_root_z = tail.z0
                torsion_from_tail += vert_loads[i] * z_arm

            # Add as point loads/moments to fuselage
            tail_reaction_load = {
                "x": tail_root_x,
                "y": tail_root_y,
                "z": tail_root_z,
                "Pz": total_horiz_force,  # vertical force from horizontal tail
                "My": moment_horiz,  # moment from horizontal tail
                "Px": 0,
                "Mz": moment_vert,  # moment from vertical tail
                "Mx": torsion_from_tail,
            }
            # Add tail_reaction_load to your fuselage point loads list
            point_loads.append(tail_reaction_load)
        elif flight_mode == "vtol":
            # All lift from propellers, no aerodynamic lift
            lift_per_section = [0.0 for _ in wing.sections]
            if prop_connection == "wing":
                wing_point_loads_mode = wing_prop_loads
                fuselage_point_loads_mode = []
            elif prop_connection == "fuselage":
                wing_point_loads_mode = []
                fuselage_point_loads_mode = fuselage_prop_loads
        else:
            raise ValueError("Unknown flight mode")

        weight_per_section = [sec.mass(dy) * g for _, sec in wing.sections]

        # --- WING SIZING ---
        min_wing_mass, wing_scale = size_wing_for_min_mass(
            wing,
            lift_per_section,
            weight_per_section,
            shear_thickness=shear_thickness,
            safety_factor=SAFETY_FACTOR,
            wing_point_loads=wing_point_loads_mode,
        )
        # Restore original boom areas for wing
        for orig_areas, (_, section) in zip(original_wing_areas, wing.sections):
            for boom, orig_area in zip(section.booms, orig_areas):
                boom.area = orig_area

        # --- TAIL SIZING ---
        min_tail_mass, tail_scale = size_tail_for_min_mass(
            tail,
            horiz_loads,
            vert_loads,
            shear_thickness=shear_thickness,
            safety_factor=SAFETY_FACTOR,
            area_scale_start=3.0,
            area_scale_step=0.02,
            min_scale=0.01,
            max_iter=200,
            min_boom_area=min_boom_area,
        )
        # Restore original boom areas for tail (optional, for next mode)
        for orig_areas, (_, section) in zip(
            [
                [boom.area for boom in section.booms]
                for _, section in tail.horiz_sections
            ],
            tail.horiz_sections,
        ):
            for boom, orig_area in zip(section.booms, orig_areas):
                boom.area = orig_area
        for orig_areas, (_, section) in zip(
            [
                [boom.area for boom in section.booms]
                for _, section in tail.vert_sections
            ],
            tail.vert_sections,
        ):
            for boom, orig_area in zip(section.booms, orig_areas):
                boom.area = orig_area

        # After tail sizing and before fuselage sizing in your loop:
        tail_weight = tail.mass() * g  # [N]
        tail_root_x = tail.x0
        tail_root_y = 0.0
        tail_root_z = tail.z0

        tail_weight_load = {
            "x": tail_root_x,
            "y": tail_root_y,
            "z": tail_root_z,
            "Pz": -tail_weight,  # Downward force
        }
        point_loads.append(tail_weight_load)

        # --- FUSELAGE SIZING ---
        fuselage_weight_per_section = fuselage.compute_weight_distribution()
        # For fuselage, use only point loads (no distributed lift)
        all_fuselage_point_loads_mode = list(point_loads) + fuselage_point_loads_mode

        # print(f"[INFO] Fuselage dimensions (case {fuselage_case}): width={fuselage_width:.3f} m, height={fuselage_height:.3f} m, length={fuselage_length:.3f} m")
        # print(f"[INFO] Fuselage initial structural mass: {fuselage.mass():.3f} kg")

        
        payload_weight = (
            battery_weight
            + sensors_weight
            + computing_module_weight
            + miscellaneous_weight
            + pizza_weight_1
            + pizza_weight_2
            + mechanisms_weight
            + payload_insulator_weight
        )
        
        # FIX FIX FIX SHEAR THICKNESS
        payload_per_section = [payload_weight / fuselage.n_sections for _ in range(fuselage.n_sections)]
        min_fuselage_mass, fuselage_scale = size_fuselage_for_min_mass(
            fuselage,
            distributed_loads=payload_per_section,
            shear_thickness=shear_thickness,
            safety_factor=SAFETY_FACTOR,
            area_scale_start=3.0,
            area_scale_step=0.02,
            min_scale=0.01,
            max_iter=200,
            fuselage_point_loads=all_fuselage_point_loads_mode,
            min_boom_area=min_boom_area,
        )
        # Restore original boom areas for fuselage
        for orig_areas, (_, section) in zip(original_fuselage_areas, fuselage.sections):
            for boom, orig_area in zip(section.booms, orig_areas):
                boom.area = orig_area

        results[flight_mode] = {
            "wing_mass": min_wing_mass,
            "wing_scale": wing_scale,
            "fuselage_mass": min_fuselage_mass,
            "fuselage_scale": fuselage_scale,
            "tail_mass": min_tail_mass,
            "tail_scale": tail_scale,
        }

    # --- REPORT THE LEADING (CRITICAL) MODE FOR EACH STRUCTURE ---

    wing_critical_mode = max(results, key=lambda m: results[m]["wing_mass"])
    fuselage_critical_mode = max(results, key=lambda m: results[m]["fuselage_mass"])
    tail_critical_mode = max(results, key=lambda m: results[m]["tail_mass"])

    print("\n=== STRUCTURE SIZING SUMMARY ===")
    print(
        f"Wing: Critical mode is '{wing_critical_mode}' with mass {results[wing_critical_mode]['wing_mass']:.2f} kg"
    )
    print(
        f"Fuselage: Critical mode is '{fuselage_critical_mode}' with mass {results[fuselage_critical_mode]['fuselage_mass']:.2f} kg"
    )
    print(f"Tail: Mass {results[tail_critical_mode]['tail_mass']:.2f} kg")
    print("==============================\n")

    # --- Store critical mode variables for further use ---
    wing_critical_mass = results[wing_critical_mode]["wing_mass"]
    wing_critical_scale = results[wing_critical_mode]["wing_scale"]
    fuselage_critical_mass = results[fuselage_critical_mode]["fuselage_mass"]
    fuselage_critical_scale = results[fuselage_critical_mode]["fuselage_scale"]
    tail_critical_mass = results[tail_critical_mode]["tail_mass"]
    tail_critical_scale = results[tail_critical_mode]["tail_scale"]

    # Example: print or use these variables
    print(f"Wing critical scale: {wing_critical_scale:.2f}")
    print(f"Fuselage critical scale: {fuselage_critical_scale:.2f}")

    # --- Prepare and plot the critical case for the WING ---
    if wing_critical_mode == "cruise":
        # All lift from wings, no propeller loads
        lift_per_section_plot = [
            elliptical_lift_distribution(y, drone) * dy for y, _ in wing.sections
        ]
        wing_point_loads_plot = []
    elif wing_critical_mode == "vtol":
        # All lift from propellers, no aerodynamic lift
        lift_per_section_plot = [0.0 for _ in wing.sections]
        wing_point_loads_plot = wing_prop_loads

    weight_per_section_plot = [sec.mass(dy) * g for _, sec in wing.sections]
    total_vertical_load_plot = [
        lift - weight
        for lift, weight in zip(lift_per_section_plot, weight_per_section_plot)
    ]
    # For shear, add prop loads if present
    shear_vertical_load_plot = total_vertical_load_plot.copy()
    for pl in wing_point_loads_plot:
        idx = min(
            range(len(wing.sections)), key=lambda i: abs(wing.sections[i][0] - pl["y"])
        )
        shear_vertical_load_plot[idx] += pl.get("Pz", 0)

    moments_x_plot = wing.compute_bending_moments_with_point_loads(
        total_vertical_load_plot, wing_point_loads_plot
    )
    stresses_per_section_plot = []
    for i, (y_pos, section) in enumerate(wing.sections):
        stresses = section.bending_stress(Mx=moments_x_plot[i], My=0)
        stresses_per_section_plot.append(stresses)

    drag_per_section_plot = [
        constant_drag_distribution(drone) * dy for y, _ in wing.sections
    ]

    # --- Compute a global arrow scale for all plots ---

    all_loads = []

    # Wing loads
    if "lift_per_section_plot" in locals():
        all_loads += [abs(L) for L in lift_per_section_plot]
    if "weight_per_section_plot" in locals():
        all_loads += [abs(W) for W in weight_per_section_plot]
    if "wing_point_loads_plot" in locals():
        all_loads += [abs(pl.get("Pz", 0)) for pl in wing_point_loads_plot]

    # Fuselage loads
    if "fuselage_weight_per_section" in locals():
        all_loads += [abs(W) for W in fuselage_weight_per_section]
    if "all_fuselage_point_loads_plot" in locals():
        all_loads += [abs(pl.get("Pz", 0)) for pl in all_fuselage_point_loads_plot]

    arrow_scale = max(all_loads) / 0.1 if all_loads and max(all_loads) != 0 else 1.0

    # Plot the critical wing case
    if plot:
        wing.plot_3d_wing(
            stresses_per_section_plot,
            lift_per_section=lift_per_section_plot,
            weight_per_section=weight_per_section_plot,
            point_loads=wing_point_loads_plot,
            drag_per_section=drag_per_section_plot,
            arrow_scale=arrow_scale,
        )

    # --- Prepare and plot the critical case for the FUSELAGE ---
    if fuselage_critical_mode == "cruise":
        fuselage_point_loads_plot = []
    elif fuselage_critical_mode == "vtol":
        fuselage_point_loads_plot = fuselage_prop_loads

    all_fuselage_point_loads_plot = point_loads + fuselage_point_loads_plot

    Mz_per_section_plot, My_per_section_plot = (
        fuselage.compute_bending_moments_with_distributed_and_point_loads(
            fuselage_weight_per_section, all_fuselage_point_loads_plot
        )
    )
    fuselage_stresses_per_section_plot = fuselage.compute_bending_stresses(
        Mz_per_section_plot, My_per_section_plot
    )

    # Plot the critical fuselage case
    if plot:
        fuselage.plot_3d_fuselage(
            fuselage_stresses_per_section_plot,
            point_loads=all_fuselage_point_loads_plot,
            weight_per_section=fuselage_weight_per_section,
            arrow_scale=arrow_scale,
        )

    """
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

    # Sizing loop for minimal mass (automated)
    min_mass, final_scale = size_wing_for_min_mass(
        wing,
        lift_per_section,
        weight_per_section,
        shear_thickness=0.002,
        safety_factor=SAFETY_FACTOR,
        wing_point_loads=wing_point_loads
    )
    print(f"\n=== OPTIMAL STRUCTURE FOUND ===")
    print(
        f"Minimal safe wing mass: {min_mass:.2f} kg (area scale factor: {final_scale:.2f})"
    )
    print("==============================\n")
    
    # --- Export wing sections ---
    """
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

    return wing_critical_mass, fuselage_critical_mass, tail_critical_mass


# TODO: add the horizontal propeller to tail and add weight of all propellers to the fuselage

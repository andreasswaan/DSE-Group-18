from matplotlib import pyplot as plt
from numpy import np

from .structure_materials import Material


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

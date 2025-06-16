from typing import TYPE_CHECKING

if TYPE_CHECKING:
    pass
from prelim_des.idealized_structure import (
    create_rectangular_section,
    FuselageStructure,
    WingStructure,
    TailStructure,
    materials,
    get_fuselage_dimensions,
    g,
)

import matplotlib.pyplot as plt

if __name__ == "__main__":
    from prelim_des.drone import Drone
    from prelim_des.performance import Performance
    from prelim_des.mission import Mission

    drone = Drone()
    mission = Mission("DRCCRCCRCCD")
    perf = Performance(drone, mission)
    drone.perf = perf
    drone.class_1_weight_estimate()
    drone.wing.S = perf.wing_area(drone.OEW)
    # --- Parameters (match your main code) ---
    fuselage_case = 2  # or 1, as desired

# Fuselage dimensions
fuselage_width, fuselage_height, fuselage_length = get_fuselage_dimensions(
    fuselage_case
)

root_section = create_rectangular_section(
    width=drone.wing.c_root,
    height=drone.wing.thick_over_chord * drone.wing.c_root,
    n_regular_booms=12,
    spar_cap_area=5e-4,
    regular_boom_area=1e-4,
    material_name="al_6061_t4",
    materials=materials,
)


# Wing structure
class DummyDrone:
    class Wing:
        span = drone.wing.span  # Example value, replace with your drone.wing.span
        taper = drone.wing.taper  # Example value, replace with your drone.wing.taper

    wing = Wing()


dummy_drone = DummyDrone()
wing = WingStructure(
    n_sections=10,
    root_section=root_section,
    drone=dummy_drone,
)

# Fuselage root section
fuselage_root_section = create_rectangular_section(
    width=fuselage_width,
    height=fuselage_height,
    n_regular_booms=12,
    spar_cap_area=4e-4,
    regular_boom_area=1e-4,
    material_name="al_6061_t4",
    materials=materials,
)

# Fuselage structure
fuselage = FuselageStructure(
    length=fuselage_length,
    n_sections=20,
    root_section=fuselage_root_section,
    taper_ratio=1.0,
)

# Tail sections
horiz_span = 0.6
horiz_chord = 0.15
vert_span = 0.25
vert_chord = 0.12

horiz_section = create_rectangular_section(
    width=horiz_chord,
    height=0.02,
    n_regular_booms=8,
    spar_cap_area=1e-4,
    regular_boom_area=5e-5,
    material_name="al_6061_t4",
    materials=materials,
)

vert_section = create_rectangular_section(
    width=0.02,
    height=vert_chord,
    n_regular_booms=8,
    spar_cap_area=1e-4,
    regular_boom_area=5e-5,
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
    x0=1.1 * fuselage_length,
    z0=0.0,
)


class Material:
    def __init__(self, density):
        self.density = density


class Boom:
    def __init__(self, x, y, z, area, material):
        self.x = x  # chordwise or local x
        self.y = y  # local y (not used for CG here)
        self.z = z  # vertical position
        self.area = area
        self.material = material


class Section:
    def __init__(self, x_section, booms):
        self.x_section = x_section  # global x of section (e.g. along fuselage)
        self.booms = booms

    def mass(self, segment_length=1.0):
        return sum(
            boom.area * segment_length * boom.material.density for boom in self.booms
        )


# Compute x and z CG for the wing
def compute_wing_cg_xz(wing: WingStructure) -> tuple[float, float]:
    total_mass = 0.0
    x_weighted_sum = 0.0
    z_weighted_sum = 0.0
    dy = wing.dy
    for y, section in wing.sections:
        for boom in section.booms:
            mass = boom.area * dy * boom.material.density
            x_global = boom.x  # chordwise position (relative to wing root)
            z_global = boom.y  # vertical position (in your convention)
            x_weighted_sum += mass * x_global
            z_weighted_sum += mass * z_global
            total_mass += mass
    if total_mass == 0:
        return 0.0, 0.0
    return x_weighted_sum / total_mass, z_weighted_sum / total_mass


# Compute x and z CG for the fuselage
def compute_fuselage_cg_xz(fuselage: FuselageStructure) -> tuple[float, float]:
    total_mass = 0.0
    x_weighted_sum = 0.0
    z_weighted_sum = 0.0
    dz = fuselage.dz
    for x_section, section in fuselage.sections:
        for boom in section.booms:
            mass = boom.area * dz * boom.material.density
            x_global = x_section + boom.x  # section x + local x
            z_global = boom.y  # vertical position (in your convention)
            x_weighted_sum += mass * x_global
            z_weighted_sum += mass * z_global
            total_mass += mass
    if total_mass == 0:
        return 0.0, 0.0
    return x_weighted_sum / total_mass, z_weighted_sum / total_mass


# Compute x and z CG for the tail (horizontal and vertical)
def compute_tail_cg_xz(tail: TailStructure) -> tuple[float, float]:
    total_mass = 0.0
    x_weighted_sum = 0.0
    z_weighted_sum = 0.0
    dz_h = tail.horiz_span / (tail.n_sections - 1)
    dz_v = tail.vert_span / (tail.n_sections - 1)
    # Horizontal tail
    for pos, section in tail.horiz_sections:
        for boom in section.booms:
            mass = boom.area * dz_h * boom.material.density
            x_global = pos[0] + boom.x
            z_global = pos[2]  # horizontal tail is at constant z0
            x_weighted_sum += mass * x_global
            z_weighted_sum += mass * z_global
            total_mass += mass
    # Vertical tail
    for pos, section in tail.vert_sections:
        for boom in section.booms:
            mass = boom.area * dz_v * boom.material.density
            x_global = pos[0] + boom.x
            z_global = pos[2] + boom.y
            x_weighted_sum += mass * x_global
            z_weighted_sum += mass * z_global
            total_mass += mass
    if total_mass == 0:
        return 0.0, 0.0
    return x_weighted_sum / total_mass, z_weighted_sum / total_mass


wing_cg_x, wing_cg_z = compute_wing_cg_xz(wing)
fuselage_cg_x, fuselage_cg_z = compute_fuselage_cg_xz(fuselage)
tail_cg_x, tail_cg_z = compute_tail_cg_xz(tail)

print(f"Wing CG: x={float(wing_cg_x):.3f} m, z={float(wing_cg_z):.3f} m")
print(f"Fuselage CG: x={float(fuselage_cg_x):.3f} m, z={float(fuselage_cg_z):.3f} m")
print(f"Tail CG: x={float(tail_cg_x):.3f} m, z={float(tail_cg_z):.3f} m")


def plot_fuselage_structure_3d(fuselage):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    for x_section, section in fuselage.sections:
        xs = [x_section + boom.x for boom in section.booms]
        ys = [boom.z if hasattr(boom, "z") else 0.0 for boom in section.booms]
        zs = [boom.y for boom in section.booms]
        ax.plot(xs, ys, zs, "o-", alpha=0.5)
    ax.set_xlabel("x (fuselage length) [m]")
    ax.set_ylabel("y (lateral) [m]")
    ax.set_zlabel("z (vertical) [m]")
    ax.set_title("Fuselage Boom Layout (3D)")
    plt.show()


def plot_wing_structure_3d(wing):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    for y, section in wing.sections:
        xs = [boom.x for boom in section.booms]
        ys = [y for _ in section.booms]
        zs = [boom.y for boom in section.booms]
        ax.plot(xs, ys, zs, "o-", alpha=0.5)
    ax.set_xlabel("x (chordwise) [m]")
    ax.set_ylabel("y (spanwise) [m]")
    ax.set_zlabel("z (vertical) [m]")
    ax.set_title("Wing Boom Layout (3D)")
    plt.show()


def plot_tail_structure_3d(tail):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    # Horizontal tail
    for pos, section in tail.horiz_sections:
        xs = [pos[0] + boom.x for boom in section.booms]
        ys = [pos[1] for _ in section.booms]
        zs = [pos[2] for _ in section.booms]
        ax.plot(xs, ys, zs, "o-", color="blue", alpha=0.5)
    # Vertical tail
    for pos, section in tail.vert_sections:
        xs = [pos[0] + boom.x for boom in section.booms]
        ys = [pos[1] for _ in section.booms]
        zs = [pos[2] + boom.y for boom in section.booms]
        ax.plot(xs, ys, zs, "o-", color="red", alpha=0.5)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("Tail Boom Layout (3D)")
    plt.show()


# Example usage:
# plot_fuselage_structure_3d(fuselage)
# plot_wing_structure_3d(wing)
# plot_tail_structure_3d(tail)


import numpy as np


# Dummy material for booms
class DummyMaterial:
    def __init__(self, density=2700):
        self.density = density


dummy_material = DummyMaterial()


# Dummy Section and Boom classes if not already defined
class DummyBoom:
    def __init__(self, x, y, area, material):
        self.x = x  # chordwise
        self.y = y  # vertical
        self.z = 0  # for 3D plotting, lateral (spanwise) is handled by section position
        self.area = area
        self.material = material


class DummySection:
    def __init__(self, booms):
        self.booms = booms


# Create a simple rectangular wing: 5 spanwise sections, each with 4 corner booms
n_sections = 5
span = 1.0  # meters
chord = 0.2  # meters
thickness = 0.03  # meters
area = 1e-4  # m^2 per boom

sections = []
for i in range(n_sections):
    y = i * span / (n_sections - 1)  # spanwise position
    booms = [
        DummyBoom(0, -thickness / 2, area, dummy_material),  # leading edge, bottom
        DummyBoom(0, thickness / 2, area, dummy_material),  # leading edge, top
        DummyBoom(chord, -thickness / 2, area, dummy_material),  # trailing edge, bottom
        DummyBoom(chord, thickness / 2, area, dummy_material),  # trailing edge, top
        DummyBoom(chord, thickness / 3, area, dummy_material),
        DummyBoom(chord, thickness / 7, area, dummy_material),
        DummyBoom(chord, thickness / 4, area, dummy_material),
        DummyBoom(chord, thickness / 6, area, dummy_material),
        DummyBoom(chord, thickness / 5, area, dummy_material),
        DummyBoom(chord, thickness / 8, area, dummy_material),
        DummyBoom(chord, thickness / 9, area, dummy_material),
    ]
    sections.append((y, DummySection(booms)))


# Dummy WingStructure class for compatibility with your plotting/CG code
class DummyWing:
    def __init__(self, sections, dy):
        self.sections = sections
        self.dy = dy


dummy_wing = DummyWing(sections, span / (n_sections - 1))

# Now you can use:
# plot_wing_structure_3d(dummy_wing)
# wing_cg_x, wing_cg_z = compute_wing_cg_xz(dummy_wing)
# print(f"Dummy Wing CG: x={wing_cg_x:.3f} m, z={wing_cg_z:.3f} m")

import numpy as np

from prelim_des.constants import g

from .structure_components import FuselageStructure, WingStructure

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


def size_wing_for_min_mass(
    wing: "WingStructure",
    lift_per_section: list[float],
    weight_per_section: list[float],
    shear_thickness: float = 0.002,
    safety_factor: float = 2.0,
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


def size_fuselage_for_min_mass(
    fuselage: "FuselageStructure",
    distributed_loads: list[float],
    shear_thickness: float = 0.002,
    safety_factor: float = 2.0,
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
        distributed_loads = [section.mass(dz) * g for _, section in fuselage.sections]

        # --- Bending moments and stresses ---
        # Use only distributed loads for moments, pass point loads separately
        Mz_per_section, My_per_section = (
            fuselage.compute_bending_moments_with_distributed_and_point_loads(
                distributed_loads, fuselage_point_loads or []
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

        # --- Find critical stress (yield/shear) ---
        critical = find_critical_stress(
            [sec for _, sec in fuselage.sections],
            stresses_per_section,
            shear_stresses_per_section,
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

    def mass(self) -> float:
        dz = self.length / (self.n_sections - 1)
        return sum(section.mass(dz) for _, section in self.sections)

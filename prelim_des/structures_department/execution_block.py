import csv
import numpy as np

from prelim_des.constants import g
from prelim_des.drone import Drone

from .load_distributions import elliptical_lift_distribution, constant_drag_distribution
from .plotting import plot_3d_fuselage, plot_3d_wing, plot_deformed_wing
from .section_geometry import create_rectangular_section
from .sizing import size_wing_for_min_mass, size_fuselage_for_min_mass, get_fuselage_dimensions, get_fuselage_payload_weights
from .structure_components import ConnectorStructure, FuselageStructure, WingStructure
from .structure_materials import toml, materials

# === MAIN EXECUTION ===

def run_structure_analysis(
    drone: Drone,
    prop_connection: str = "wing",
    # prop_connection: "wing" or "fuselage"
    fuselage_case=2,  # or 2, (1 for chubby, 2 for elongated fuselage)
    banked=True,  # Set to False for normal cruise, True for banked case
):
    SAFETY_FACTOR = 2.0

    # --- All your main logic from the current if __name__ == "__main__": block ---
    # (Copy everything from the current if __name__ == "__main__": block here)

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

    fuselage_root_section = create_rectangular_section(
        width=fuselage_width,
        height=fuselage_height,
        n_regular_booms=12,
        spar_cap_area=2e-5,
        regular_boom_area=1e-5,
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
    
    lift_per_section = [elliptical_lift_distribution(y, drone) * dy for y, _ in wing.sections]
    weight_per_section = [sec.mass(dy) * g for _, sec in wing.sections]
    drag_per_section = [constant_drag_distribution(drone) * dy for y, _ in wing.sections]
    
    # Calculate total forces on the wing
    total_lift = sum(lift_per_section)
    total_weight = sum(weight_per_section)
    total_drag = sum(drag_per_section)

    net_vertical_force = total_lift - total_weight  # Pz (upwards positive)
    net_drag_force = total_drag  
    
    # Calculate moments at the two connection points (left and right)
    connection_points = [
        {"x": 0.9 * fuselage_length, "y": 0.5 * fuselage_width, "z": 0.5 * fuselage_height},
        {"x": 0.9 * fuselage_length, "y": -0.5 * fuselage_width, "z": 0.5 * fuselage_height},
    ]

    # For each connection, sum moments from all sections
    for conn in connection_points:
        Mz = 0.0  # Moment about z-axis (from vertical forces, i.e., lift-weight)
        My = 0.0  # Moment about y-axis (from drag)
        for (y_pos, _), lv, drag in zip(wing.sections, [l-w for l, w in zip(lift_per_section, weight_per_section)], drag_per_section):
            # Moment arm is spanwise distance from section to connection point
            arm_y = conn["y"] - y_pos  # y_conn - y_section
            # Moment from vertical force (about z): Mz += (lift-weight) * arm_y * dy
            Mz += lv * arm_y * dy
            # Moment from drag (about y): My += drag * arm_y * dy
            My += drag * arm_y * dy
        conn["Mz"] = Mz
        conn["My"] = My

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
    min_fuselage_mass, fuselage_area_scale = size_fuselage_for_min_mass(
        fuselage,
        distributed_loads=fuselage_weight_per_section,
        shear_thickness=0.002,
        safety_factor=SAFETY_FACTOR,
        area_scale_start=10.0,
        area_scale_step=0.02,
        min_scale=0.01,
        max_iter=200,
        fuselage_point_loads=all_fuselage_point_loads,
        min_boom_area=1e-5,
    )

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
            Vz=Vz, thickness=0.002
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

    # --- SIZING FOR BOTH FLIGHT MODES ---

    results = {}

    for flight_mode in ["cruise", "vtol"]:
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
            shear_thickness=0.002,
            safety_factor=SAFETY_FACTOR,
            wing_point_loads=wing_point_loads_mode,
        )
        # Restore original boom areas for wing
        for orig_areas, (_, section) in zip(original_wing_areas, wing.sections):
            for boom, orig_area in zip(section.booms, orig_areas):
                boom.area = orig_area

        # --- FUSELAGE SIZING ---
        fuselage_weight_per_section = fuselage.compute_weight_distribution()
        # For fuselage, use only point loads (no distributed lift)
        all_fuselage_point_loads_mode = list(point_loads) + fuselage_point_loads_mode

        # print(f"[INFO] Fuselage dimensions (case {fuselage_case}): width={fuselage_width:.3f} m, height={fuselage_height:.3f} m, length={fuselage_length:.3f} m")
        # print(f"[INFO] Fuselage initial structural mass: {fuselage.mass():.3f} kg")

        min_fuselage_mass, fuselage_scale = size_fuselage_for_min_mass(
            fuselage,
            distributed_loads=fuselage_weight_per_section,
            shear_thickness=0.002,
            safety_factor=SAFETY_FACTOR,
            area_scale_start=3.0,
            area_scale_step=0.02,
            min_scale=0.01,
            max_iter=200,
            fuselage_point_loads=all_fuselage_point_loads_mode,
            min_boom_area=1e-5,
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
        }

    # --- REPORT THE LEADING (CRITICAL) MODE FOR EACH STRUCTURE ---

    wing_critical_mode = max(results, key=lambda m: results[m]["wing_mass"])
    fuselage_critical_mode = max(results, key=lambda m: results[m]["fuselage_mass"])

    print("\n=== STRUCTURE SIZING SUMMARY ===")
    print(
        f"Wing: Critical mode is '{wing_critical_mode}' with mass {results[wing_critical_mode]['wing_mass']:.2f} kg"
    )
    print(
        f"Fuselage: Critical mode is '{fuselage_critical_mode}' with mass {results[fuselage_critical_mode]['fuselage_mass']:.2f} kg"
    )
    print("==============================\n")

    # --- Store critical mode variables for further use ---
    wing_critical_mass = results[wing_critical_mode]["wing_mass"]
    wing_critical_scale = results[wing_critical_mode]["wing_scale"]
    fuselage_critical_mass = results[fuselage_critical_mode]["fuselage_mass"]
    fuselage_critical_scale = results[fuselage_critical_mode]["fuselage_scale"]

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

    return None

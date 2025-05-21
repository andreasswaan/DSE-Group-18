import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from database.read_databse import (
    get_data_from_database,
    get_regression_plane,
)


mission_profile = {
    "cruise_OEW": 0,
    "cruise_MTOW": 8000,
    "loitering_time": 0 * 60,
    "TO_LD": 5,
    "cruise_h": 150,
    "cruise_speed": 15,
    "payload_mass": 3,
}
"""
    cruise OEW length [m],
    cruise MTOW length [m],
    loitering time [s], 
    #TO/landing [#], 
    cruise height [m], 
    cruise speed [m/s]
    payload mass [kg]
"""


mission_profile_2 = [
    {
        "cruise_distance": 2500 * np.sqrt(3),
        "loitering_time": 2 * 60,
        "cruise_h": 200,
        "cruise_speed": 15,
        "payload_mass": 0,
        "TO_speed": 6,
        "LD_speed": 3,
    },
    {
        "cruise_distance": 2500 * np.sqrt(3),
        "loitering_time": 2 * 60,
        "cruise_h": 200,
        "cruise_speed": 15,
        "payload_mass": 2.5,
        "TO_speed": 6,
        "LD_speed": 3,
    },
    {
        "cruise_distance": 2500 * np.sqrt(3),
        "loitering_time": 0.5 * 60,
        "cruise_h": 200,
        "cruise_speed": 15,
        "payload_mass": 0,
        "TO_speed": 6,
        "LD_speed": 3,
    },
]


# Aerodynamics Constants:
L_over_D_cruise = 10.0  # Lift-to-drag ratio during cruise
CL_cruise = 0.5  # lift coefficient (placeholder value)
CD_flat_plate = 1.17  # flat plate drag coefficient at 90deg aoa
CD_cruise = CL_cruise / L_over_D_cruise  # drag coefficient (placeholder value)
ToverDmax_cruise = 1.0  # thrust-to-drag ratio (placeholder value)
ToverWmax_takeoff = 2.0  # thrust-to-weight ratio at takeoff (placeholder value)
n_vert_props = 8  # number of vertical propellers
n_hori_props = 2  # number of vertical propellers
vert_prop_diameter = 0.3  # metres, diameter of vertical propellers
vert_prop_total_area = (
    n_vert_props * np.pi * (vert_prop_diameter / 2) ** 2
)  # m^2, total area of vertical propellers


# Atmospheric Constats:
ρ = 1.225  # Air density in kg/m^3
g = 9.80665  # Acceleration due to gravity in m/s^2


# Database Settings:
db_filter_max_payload = 4.5  # kg, maximum payload for database filter
db_filter_max_range = 70  # km, maximum range for database filter
db = get_data_from_database()


# Flight Assumptions:
V_takeoff = 6.3  # m/s, vertical takeoff speed
V_land = 6.3  # m/s, vertical landing speed
takeoff_time = (
    mission_profile["cruise_h"] / V_takeoff
)  # seconds, time to reach cruise height
range_at_max_payload = mission_profile["cruise_MTOW"] / 1000
percentage_of_mtow_by_wing = 0.8  # []


# Efficiency Constants
η_prop = 0.85  # Propeller efficiency
η_elec = 0.95  # Motor efficiency
eta_hori_props = η_prop  # efficiency of horizontal propellers
eta_vert_props = η_prop  # efficiency of vertical propellers


# Others:
battery_energy_density = (
    250 * 3600
)  # J/kg, energy density of battery (placeholder value)
battery_lowest_limit = 0.2  # Lowest limit of battery (20% of capacity)
max_payload_dimension = 0.5  # metres, maximum payload dimension (side of square)


def drag(S, V, CD):
    return 0.5 * ρ * V**2 * S * CD


def payload_mass_to_mtow(payload_mass):
    # [A, B], C = get_regression_plane(db, db_filter_max_payload, db_filter_max_range)
    [A], C = get_regression_plane(db, db_filter_max_payload, db_filter_max_range)

    # mtow = A * payload_mass + B * range + C  # Example conversion factor
    mtow = A * payload_mass + C  # Example conversion factor

    return mtow * g


def calculate_wing_surface_area(weight, V_cruise):

    S = weight / (0.5 * ρ * V_cruise**2 * CL_cruise)
    return S


def calculate_drag_cruise(S, V_cruise):
    D = 0.5 * ρ * V_cruise**2 * S * CD_cruise
    return D


def calculate_thrust_cruise(D):
    T = D * ToverDmax_cruise  # Assuming level flight, thrust equals drag
    return T


def calculate_power_cruise(T, V_cruise):
    V_cruise = mission_profile["cruise_speed"]
    P_cruise = T * V_cruise  # Power = Thrust * Velocity
    return P_cruise


def calculate_prop_power(needed_thrust):
    """
    Convert thrust needed to provided power by propellors

    Args:
        needed_thrust (float): needed thrust

    Returns:
        float: power needed for this thrust.
    """
    power = (needed_thrust) ** 1.5 / (
        np.sqrt(2 * ρ * vert_prop_total_area) * eta_vert_props
    )
    return power


def calculate_energy_per_mission(P_cruise, S, pct_wing_lift=1, PLOT=False):
    takeoff_nr = mission_profile["TO_LD"]
    cruise_height = mission_profile["cruise_h"]
    avg_mission_time = (
        mission_profile["loitering_time"]
        + (mission_profile["cruise_MTOW"] + mission_profile["cruise_OEW"])
        / mission_profile["cruise_speed"]
    )
    D_takeoff = drag(max_payload_dimension**2 + S, V_takeoff, CD_flat_plate)
    D_land = drag(max_payload_dimension**2 + S, V_land, CD_flat_plate)

    energy_takeoff = (
        (
            takeoff_nr
            * (mtow + D_takeoff) ** 1.5
            / (np.sqrt(2 * ρ * vert_prop_total_area) * eta_vert_props)
        )
        * cruise_height
        / V_takeoff
    )
    print("energy takeoff", energy_takeoff / takeoff_nr, "J per takeoff")
    energy_landing = (
        (
            takeoff_nr
            * (mtow - D_land) ** 1.5
            / (np.sqrt(2 * ρ * vert_prop_total_area) * eta_vert_props)
        )
        * cruise_height
        / V_land
    )
    print("energy landing", energy_landing / takeoff_nr, "J per land")

    energy_cruise_by_props = (
        (
            (mtow * (1 - pct_wing_lift)) ** 1.5
            / (np.sqrt(2 * ρ * vert_prop_total_area) * eta_vert_props)
        )
        * mission_profile["cruise_MTOW"]
        / mission_profile["cruise_speed"]
    )
    print(
        "energy cruise (by props)",
        energy_cruise_by_props,
        "J done by props during cruise",
    )
    energy_cruise = P_cruise * avg_mission_time
    print("energy cruise", energy_cruise)

    energy_per_mission = (
        energy_takeoff + energy_cruise + energy_landing + energy_cruise_by_props
    ) / η_elec
    # Pie chart for energy distribution
    if PLOT:
        energies = [
            energy_takeoff,
            energy_cruise,
            energy_landing,
            energy_cruise_by_props,
        ]
        labels = ["Takeoff", "Cruise (wing)", "Landing", "Cruise (props)"]
        plt.figure(figsize=(6, 6))
        plt.pie(energies, labels=labels, autopct="%1.1f%%", startangle=90)
        plt.title("Energy Distribution per Mission")
        plt.show()

    return energy_per_mission


def size_battery(energy_per_mission):

    battery_mass = (energy_per_mission / battery_energy_density) / (
        1 - battery_lowest_limit
    )  # kg
    return battery_mass


def plot_takeoff_energy_vs_speed(S, speed_range=None):
    if speed_range is None:
        speed_range = np.linspace(1, 15, 30)  # m/s, adjust as needed

    takeoff_energies = []
    for v in speed_range:
        D_takeoff = drag(max_payload_dimension**2 + S, v, CD_flat_plate)
        energy_takeoff = (
            mission_profile["TO_LD"]
            * (mtow + D_takeoff) ** 1.5
            / (np.sqrt(2 * ρ * vert_prop_total_area) * eta_vert_props)
            * mission_profile["cruise_h"]
            / v
        )
        takeoff_energies.append(energy_takeoff)

    plt.figure(figsize=(8, 5))
    plt.plot(speed_range, takeoff_energies, marker="o")
    plt.xlabel("Takeoff Speed (m/s)")
    plt.ylabel("Takeoff Energy (J)")
    plt.title("Takeoff Energy vs Takeoff Speed")
    plt.grid(True)
    plt.show()
    print(
        min(takeoff_energies), "J at", speed_range[np.argmin(takeoff_energies)], "m/s"
    )


# if __name__ == "__main__":
#     PLOT = True
#     mtow = payload_mass_to_mtow(
#         mission_profile["payload_mass"], range_at_max_payload
#     )  # kg
#     S = calculate_wing_surface_area(mtow * percentage_of_mtow_by_wing)  # m^2
#     D = calculate_drag_cruise(S, mission_profile["cruise_speed"])  # N
#     T = calculate_thrust_cruise(D)  # N
#     P_cruise = calculate_power_cruise(T, mission_profile["cruise_speed"])  # W
#     energy_per_mission = calculate_energy_per_mission(P_cruise, S, PLOT)  # J
#     battery_mass = size_battery(energy_per_mission)  # kg

#     print("MTOW:", mtow, "N")
#     print("MTOW:", mtow / g, "kg")
#     print("S:", S, "m^2")
#     print("Battery mass:", battery_mass, "kg")
#     print("Energy per mission:", energy_per_mission, "J")


def perform_calc_mission(
    mission_profile, mtow, OEW, pct_wing_lift=1, PLOT=False, PRINT=True
):
    print(mtow)
    total_energy = 0
    total_TO_energy = 0
    total_LD_energy = 0
    total_cruise_prop_energy = 0
    total_cruise_wing_energy = 0
    total_loitering_energy = 0
    power_provided_by_props = []
    wing_surfaces = []

    for mission_phase in mission_profile:
        payload_weight = mission_phase["payload_mass"] * g
        wing_surface = calculate_wing_surface_area(
            (OEW + payload_weight) * pct_wing_lift, mission_phase["cruise_speed"]
        )
        wing_surfaces.append(wing_surface)

    wing_surface = np.max(wing_surfaces)
    print(f"MTOW {mtow/g} kg")
    print(f"OEW {OEW/g} kg")
    print(f"Wing surface area {wing_surface} m^2")
    print("\n")

    i = 0
    for mission_phase in mission_profile:
        payload_weight = mission_phase["payload_mass"] * g
        D = calculate_drag_cruise(wing_surface, mission_phase["cruise_speed"])  # [N]
        T = calculate_thrust_cruise(D)

        time_cruise = mission_phase["cruise_distance"] / mission_phase["cruise_speed"]
        P_cruise_wing = calculate_power_cruise(T, mission_phase["cruise_speed"])
        E_cruise_wing = P_cruise_wing * time_cruise
        total_energy += E_cruise_wing
        total_cruise_wing_energy += E_cruise_wing

        P_prop_cruise = calculate_prop_power(
            (OEW + payload_weight) * (1 - pct_wing_lift)
        )
        power_provided_by_props.append(P_prop_cruise)
        E_cruise_prop = P_prop_cruise * time_cruise
        total_energy += E_cruise_prop
        total_cruise_prop_energy += E_cruise_prop

        time_TO = (
            mission_phase["cruise_h"] + mission_phase["cruise_speed"] / 2 * g
        ) / mission_phase["TO_speed"]
        drag_TO = drag(
            max_payload_dimension**2 + wing_surface,
            mission_phase["TO_speed"],
            CD_flat_plate,
        )
        P_prop_TO = calculate_prop_power((OEW + payload_weight) + drag_TO)
        power_provided_by_props.append(P_prop_TO)
        E_prop_TO = P_prop_TO * time_TO
        total_energy += E_prop_TO
        total_TO_energy += E_prop_TO
        # print(f"thrust req for TO {OEW + payload_weight + drag_TO}")

        time_LD = mission_phase["cruise_h"] / mission_phase["LD_speed"]
        drag_LD = drag(
            max_payload_dimension**2 + wing_surface,
            mission_phase["LD_speed"],
            CD_flat_plate,
        )
        P_prop_LD = calculate_prop_power(
            (OEW + mission_phase["payload_mass"]) - drag_LD
        )
        power_provided_by_props.append(P_prop_LD)
        E_prop_LD = P_prop_LD * time_LD
        total_energy += E_prop_LD
        total_LD_energy += E_prop_LD

        time_loitering = mission_phase["loitering_time"]
        P_prop_loitering = calculate_prop_power((OEW + mission_phase["payload_mass"]))
        power_provided_by_props.append(P_prop_loitering)
        E_prop_loitering = P_prop_loitering * time_loitering
        total_energy += E_prop_loitering
        total_loitering_energy += E_prop_loitering

        total_phase_energy = E_cruise_wing + E_cruise_prop + E_prop_TO + E_prop_LD
        phase_battery_mass = size_battery(total_phase_energy)

        if PRINT:
            print(f"MISSION PHASE {i}")
            print(f"Energy by cruise wing {E_cruise_wing}")
            print(f"Energy by cruise prop {E_cruise_prop}")
            print(f"Energy for TO {E_prop_TO}")
            print(f"Energy for LD {E_prop_LD}")
            print(f"Energy for loitering {E_prop_loitering}")
            print(f"Total phase energy {total_phase_energy}")
            print(f"Phase battery Mass {phase_battery_mass} kg")
            print("\n")

        i += 1

    battery_mass = size_battery(total_energy) * (1 + battery_lowest_limit)
    hover_noise = 9.2 * np.log(mtow / g) + 71.7
    if PRINT:
        print(f"Total Energy {total_energy} J")
        print(f"Battery Mass {battery_mass} kg")
        print(f"max power props {np.max(power_provided_by_props)} W")
        print(f"Noise during hover {hover_noise} dB(A)")
    if PLOT:
        energies = [
            total_cruise_prop_energy,
            total_cruise_wing_energy,
            total_TO_energy,
            total_LD_energy,
            total_loitering_energy,
        ]
        labels = ["Cruise (props)", "Cruise (wing)", "Takeoff", "Landing", "Loitering"]
        plt.figure(figsize=(6, 6))
        plt.pie(energies, labels=labels, autopct="%1.1f%%", startangle=90)
        plt.title(
            f"Energy Distribution per Mission, ({np.round(total_energy/1000,3)}kJ)"
        )
        plt.show()

    return battery_mass


def iterations(
    mission_profile, pct_wing_lift=1, design_payload=2.5, iterations=30, PLOT=False
):
    mtow = payload_mass_to_mtow(design_payload)  # kg
    print("Start MTOW", mtow)
    structures_mass_frac = 0.35
    structures_mass = mtow / g * structures_mass_frac
    total_nr_motors = n_vert_props + n_hori_props
    motor_mass = total_nr_motors * 0.16
    propellor_mass = total_nr_motors * 0.012
    OEW = mtow - design_payload * g
    mtows = []

    for i in range(iterations):
        structures_mass = mtow / g * structures_mass_frac
        battery_mass = perform_calc_mission(
            mission_profile, mtow, OEW, pct_wing_lift=pct_wing_lift
        )
        new_mtow = (
            battery_mass
            + structures_mass
            + motor_mass
            + propellor_mass
            + design_payload
        ) * g
        new_OEW = new_mtow - design_payload * g
        print(f"new MTOW {new_mtow/g}")
        print(f"new OEW {new_OEW/g}")
        OEW = new_OEW
        mtow = new_mtow
        mtows.append(mtow)
    if PLOT:
        plt.plot(range(len(mtows)), np.array(mtows) / g)
        plt.xlabel("Iteration")
        plt.ylabel("MTOW (kg)")
        plt.title("MTOW Convergence")
        plt.grid()
        plt.show()
    return mtows


if __name__ == "__main__":
    PLOT = True
    design_payload = 2.5
    design_range = 15000
    pct_wing_lift_array = [1]
    mtow = payload_mass_to_mtow(design_payload)  # kg
    OEW = mtow - design_payload * g
    battery_mass = perform_calc_mission(mission_profile_2, mtow, OEW)
    for pct_wing_lift in pct_wing_lift_array:
        print(f"wing lift fraction {pct_wing_lift}")
        mtows = iterations(
            mission_profile_2,
            pct_wing_lift=pct_wing_lift,
            design_payload=design_payload,
            iterations=10,
            PLOT=PLOT,
        )
        print("Final MTOW", mtows[-1] / g)

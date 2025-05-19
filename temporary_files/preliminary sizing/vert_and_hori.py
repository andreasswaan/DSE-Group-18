import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from read_databse import get_data_from_database, apply_db_filters, get_regression_plane
from constants import *
from mpl_toolkits.mplot3d import Axes3D  # Add this import at the top if not present


mission_profile = {
    "cruise_OEW": 0,
    "cruise_MTOW": 8000,
    "loitering_time": 0 * 60,
    "TO_LD": 5,
    "cruise_h": 150,
    "cruise_speed": 15,
}
"""
    cruise OEW length [m],
    cruise MTOW length [m],
    loitering time [s], 
    #TO/landing [#], 
    # cruise height [m], 
    # cruise speed [m/s]
"""
# constants
V_cruise = 15  # m/s, cruise speed
range_at_max_payload = mission_profile["cruise_MTOW"] / 1000
CL_cruise = 1  # lift coefficient (placeholder value)
CD_cruise = CL_cruise / L_over_D_cruise  # drag coefficient (placeholder value)
CD_flat_plate = 1.17  # flat plate drag coefficient at 90deg aoa
ToverDmax_cruise = 1.0  # thrust-to-drag ratio (placeholder value)
ToverWmax_takeoff = 2.0  # thrust-to-weight ratio at takeoff (placeholder value)
n_vert_props = 4  # number of vertical propellers
vert_prop_diameter = 0.3  # metres, diameter of vertical propellers
vert_prop_total_area = (
    n_vert_props * np.pi * (vert_prop_diameter / 2) ** 2
)  # m^2, total area of vertical propellers
eta_hori_props = η_prop  # efficiency of horizontal propellers
eta_vert_props = η_prop  # efficiency of vertical propellers
V_takeoff = 8  # m/s, vertical takeoff speed
V_land = 5  # m/s, vertical landing speed
cruise_height = 150  # metres, height of takeoff or landing
takeoff_time = cruise_height / V_takeoff  # seconds, time to reach cruise height

db_filter_max_payload = 4  # kg, maximum payload for database filter
db_filter_max_range = 40  # km, maximum range for database filter
db = get_data_from_database()


def payload_mass_to_mtow(payload_mass, range):
    [A, B], C = get_regression_plane(db, db_filter_max_payload, db_filter_max_range)
    mtow = A * payload_mass + B * range + C  # Example conversion factor

    print("mtow", mtow, "kg")

    return mtow * g  # Example conversion factor


def calculate_wing_surface_area(mtow):
    V_cruise = mission_profile["cruise_speed"]
    S = mtow / (0.5 * ρ * V_cruise**2 * CL_cruise)
    return S


def calculate_drag_cruise(S, V_cruise):
    V_cruise = mission_profile["cruise_speed"]
    D = 0.5 * ρ * V_cruise**2 * S * CD_cruise
    return D


def calculate_thrust_cruise(D):

    T = D * ToverDmax_cruise  # Assuming level flight, thrust equals drag
    return T


def calculate_power_cruise(T, V_cruise):
    V_cruise = mission_profile["cruise_speed"]
    P_cruise = T * V_cruise  # Power = Thrust * Velocity
    return P_cruise

def calculate_takeoff_energy(mass):
    D_takeoff = drag(max_payload_dimension**2 + S, V_takeoff, CD_flat_plate)
    energy_takeoff = (
        (mass + D_takeoff) ** 1.5
        / (np.sqrt(2 * ρ * vert_prop_total_area) * eta_vert_props)
    ) * cruise_height / V_takeoff
    return energy_takeoff

def calculate_landing_energy(mass):
    D_land = drag(max_payload_dimension**2 + S, V_land, CD_flat_plate)
    energy_landing = (
        (mass - D_land) ** 1.5
        / (np.sqrt(2 * ρ * vert_prop_total_area) * eta_vert_props)
    ) * cruise_height / V_land
    return energy_landing

def calculate_energy_per_mission(P_cruise):
    takeoff_nr = mission_profile["TO_LD"]
    avg_mission_time = (
        mission_profile["loitering_time"]
        + (mission_profile["cruise_MTOW"] + mission_profile["cruise_OEW"])
        / mission_profile["cruise_speed"]
    )
    energy_takeoff = calculate_takeoff_energy(mtow) * takeoff_nr # J
    energy_landing = calculate_landing_energy(mtow) * takeoff_nr # J
    energy_cruise = P_cruise * avg_mission_time
    print("energy cruise", energy_cruise)

    energy_per_mission = (energy_takeoff + energy_cruise + energy_landing) / η_elec
    # Pie chart for energy distribution
    energies = [energy_takeoff, energy_cruise, energy_landing]
    labels = ["Takeoff", "Cruise", "Landing"]
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


mtow = payload_mass_to_mtow(payload_mass, range_at_max_payload)  # kg
S = calculate_wing_surface_area(mtow)  # m^2
D = calculate_drag_cruise(S, mission_profile["cruise_speed"])  # N
T = calculate_thrust_cruise(D)  # N
P_cruise = calculate_power_cruise(T, mission_profile["cruise_speed"])  # W
energy_per_mission = calculate_energy_per_mission(P_cruise)  # J
battery_mass = size_battery(energy_per_mission)  # kg

print("MTOW:", mtow, "N")
print("MTOW:", mtow / g, "kg")
print("S:", S, "m^2")
print("Battery mass:", battery_mass, "kg")
print("Energy per mission:", energy_per_mission, "J")

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

def plot_range_vs_takeoffs_3d(battery_mass, takeoff_mtow_range=range(0, 15), takeoff_oew_range=range(0, 15)):
    X, Y = np.meshgrid(takeoff_mtow_range, takeoff_oew_range)
    Z = np.zeros_like(X, dtype=float)

    for i, n_mtow in enumerate(takeoff_mtow_range):
        for j, n_oew in enumerate(takeoff_oew_range):
            # Copy mission profile and set number of takeoffs/landings
            mission_profile_copy = mission_profile.copy()

            # Estimate available energy from battery
            available_energy = battery_mass * battery_energy_density * (1 - battery_lowest_limit)*η_elec

            # Estimate MTOW and S for a typical payload (or set as needed)
            mtow_local = mtow  # or recalculate if needed
            S_local = S

            # Estimate cruise power
            D_local = calculate_drag_cruise(S_local, mission_profile_copy["cruise_speed"])
            T_local = calculate_thrust_cruise(D_local)
            P_cruise_local = calculate_power_cruise(T_local, mission_profile_copy["cruise_speed"])

            energy_takeoff = calculate_takeoff_energy(mtow_local) * n_mtow + n_oew * calculate_takeoff_energy(mtow-payload_mass*g)
            energy_landing = calculate_landing_energy(mtow_local) * n_mtow + n_oew * calculate_landing_energy(mtow-payload_mass*g)
            max_range = (available_energy - (energy_takeoff + energy_landing))/ P_cruise_local * mission_profile_copy["cruise_speed"] / 1000  # in km
            max_range = max(0, max_range)
            Z[j, i] = max_range


    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(X, Y, Z, cmap='viridis')
    ax.set_xlabel('Number of Takeoffs at MTOW')
    ax.set_ylabel('Number of Takeoffs at OEW')
    ax.set_zlabel('Maximum Range (km)')
    ax.set_title(f'Range vs Takeoffs at MTOW/OEW (Battery mass = {battery_mass:.1f} kg)')
    plt.show()

# Example usage:
plot_range_vs_takeoffs_3d(3)
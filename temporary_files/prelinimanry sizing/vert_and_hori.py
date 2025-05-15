import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from read_databse import get_data_from_database, apply_db_filters, get_regression_plane
from constants import *


mission_profile = {
    "cruise_OEW": 0,
    "cruise_MTOW": 15000,
    "loitering_time": 5 * 60,
    "TO_LD": 3,
    "cruise_h": 150,
    "cruise_speed": 15,
}
"""
    cruise OEW length,
    cruise MTOW length,
    loitering time, 
    #TO/landing, 
    # cruise height, 
    # cruise speed
"""
# constants
V_cruise = 15  # m/s, cruise speed
range_at_max_payload = mission_profile["cruise_MTOW"] / 1000
CL_cruise = 0.5  # lift coefficient (placeholder value)
CD_cruise = CL_cruise / L_over_D_cruise  # drag coefficient (placeholder value)
CD_flat_plate = 1.17  # flat plate drag coefficient at 90deg aoa
ToverDmax_cruise = 1.0  # thrust-to-drag ratio (placeholder value)
ToverWmax_takeoff = 2.0  # thrust-to-weight ratio at takeoff (placeholder value)
n_vert_props = 4  # number of vertical propellers
eta_hori_props = η_prop  # efficiency of horizontal propellers
eta_vert_props = η_prop  # efficiency of vertical propellers
V_takeoff = 10  # m/s, vertical takeoff speed
cruise_height = 150  # metres, height of takeoff or landing
takeoff_time = cruise_height / V_takeoff  # seconds, time to reach cruise height
max_payload_dimension = 0.5  # metres, maximum payload dimension (side of square)
battery_lowest_limit = 0.1  # lowest limit of battery (10% of capacity)

db_filter_max_payload = 4  # kg, maximum payload for database filter
db_filter_max_range = 40  # km, maximum range for database filter
db = get_data_from_database()


def payload_mass_to_mtow(payload_mass, range):
    [A, B], C = get_regression_plane(db, db_filter_max_payload, db_filter_max_range)
    mtow = A * payload_mass + B * range + C  # Example conversion factor

    print(mtow)
    print("mtow")

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


def calculate_power_takeoff(V_takeoff):

    P_takeoff = ToverWmax_takeoff * mtow * V_takeoff / eta_vert_props
    print("p takeoff", P_takeoff)
    return P_takeoff


def size_vert_props(P_takeoff):

    vert_prop_total_area = (ToverWmax_takeoff * mtow) ** 3 / (2 * ρ * P_takeoff**2)
    vert_prop_area = vert_prop_total_area / n_vert_props
    vert_prop_diameter = 2 * np.sqrt(
        vert_prop_area / np.pi
    )  # m, diameter of propellers
    return vert_prop_total_area, vert_prop_diameter


def calculate_evergy_per_mission(P_cruise, S):
    takeoff_nr = mission_profile["TO_LD"]
    cruise_height = mission_profile["cruise_h"]
    avg_mission_time = (
        mission_profile["loitering_time"]
        + (mission_profile["cruise_MTOW"] + mission_profile["cruise_OEW"])
        / mission_profile["cruise_speed"]
    )

    vert_prop_total_area, vert_prop_diameter = size_vert_props(
        calculate_power_takeoff(V_takeoff)
    )
    print(vert_prop_total_area)
    energy_takeoff_hover = (
        takeoff_nr
        * 2
        * mtow**1.5
        / (np.sqrt(2 * ρ * vert_prop_total_area) * eta_vert_props)
    )
    energy_takeoff_climb = takeoff_nr * mtow * cruise_height
    energy_takeoff_drag = (
        3
        * CD_flat_plate
        * 0.5
        * ρ
        * V_takeoff**2
        * (max_payload_dimension**2 + S)
        * cruise_height
    )
    print(energy_takeoff_hover, energy_takeoff_climb, energy_takeoff_drag)
    energy_takeoff = energy_takeoff_hover + energy_takeoff_climb + energy_takeoff_drag

    energy_cruise = P_cruise * avg_mission_time

    energy_per_mission = (energy_takeoff + energy_cruise) / η_elec

    print(f"Energy for takeoff: {energy_takeoff:.2f} J")
    print(f"Energy for cruise: {energy_cruise:.2f} J")
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
energy_per_mission = calculate_evergy_per_mission(P_cruise, S)  # J
battery_mass = size_battery(energy_per_mission)  # kg

print("MTOW:", mtow, "N")
print("MTOW:", mtow / g, "kg")
print("S:", S, "m^2")
print("Battery mass:", battery_mass, "kg")
print("Energy per mission:", energy_per_mission, "J")

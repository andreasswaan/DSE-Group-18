import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# constants
g = 9.81  # m/s^2, acceleration due to gravity
payload_mass = 2.5 * g  # kg, payload mass
V_cruise = 15  # m/s, cruise speed
rho = 1.225  # kg/m^3, air density at sea level
CL_cruise = 0.7  # lift coefficient (placeholder value)
CLoverCD_cruise = 10  # lift-to-drag ratio (placeholder value)
CD_cruise = CL_cruise / CLoverCD_cruise  # drag coefficient (placeholder value)
CD_flat_plate = 1.17  # flat plate drag coefficient at 90deg aoa
ToverDmax_cruise = 1.2  # thrust-to-drag ratio (placeholder value)
ToverWmax_takeoff = 2.0  # thrust-to-weight ratio at takeoff (placeholder value)
n_hori_props = 4  # number of horizontal propellers
n_vert_props = 4  # number of vertical propellers
eta_hori_props = 0.50  # efficiency of horizontal propellers
eta_vert_props = 0.50  # efficiency of vertical propellers
V_takeoff = 10  # m/s, vertical takeoff speed
cruise_height = 150  # metres, height of takeoff or landing
takeoff_time = cruise_height / V_takeoff  # seconds, time to reach cruise height
avg_mission_time = 30*60  # seconds, average mission tim
max_payload_dimension = 0.5  # metres, maximum payload dimension (side of square)
battery_energy_density = 150 * 3600  # J/kg, energy density of battery (placeholder value)

def payload_mass_to_mtow(payload_mass):

    mtow = 4.27 * payload_mass + 3.8 # Example conversion factor
    return mtow  # Example conversion factor

def calculate_wing_surface_area(mtow):

    S = mtow / (0.5 * rho * V_cruise**2 * CL_cruise)
    return S

def calculate_drag_cruise(S, V_cruise):

    D = 0.5 * rho * V_cruise**2 * S * CD_cruise
    return D

def calculate_thrust_cruise(D):

    T = D * ToverDmax_cruise  # Assuming level flight, thrust equals drag
    return T

def calculate_power_cruise(T, V_cruise):

    P_cruise = T * V_cruise  # Power = Thrust * Velocity
    return P_cruise

def calculate_power_takeoff(V_takeoff):

    P_takeoff = ToverWmax_takeoff * mtow * V_takeoff / eta_vert_props
    return P_takeoff

def size_vert_props(P_takeoff):

    vert_prop_total_area = ToverWmax_takeoff * mtow / (2 * rho * V_takeoff**2 * eta_vert_props)
    vert_prop_area = vert_prop_total_area / n_vert_props
    vert_prop_diameter = 2 * np.sqrt(vert_prop_area / np.pi)  # m, diameter of propellers
    return vert_prop_total_area, vert_prop_diameter

def calculate_evergy_per_mission(P_cruise, S):

    vert_prop_total_area, vert_prop_diameter = size_vert_props(calculate_power_takeoff(V_takeoff))
    energy_takeoff_hover = 6 * mtow ** 1.5 / (np.sqrt(2 * rho * vert_prop_total_area) * eta_vert_props)
    energy_takeoff_climb = 3 * mtow * cruise_height
    energy_takeoff_drag = 3 * CD_flat_plate * 0.5 * rho * V_takeoff**2 * (max_payload_dimension**2 + S) * cruise_height
    energy_takeoff = energy_takeoff_hover + energy_takeoff_climb + energy_takeoff_drag

    energy_cruise = P_cruise * avg_mission_time

    energy_per_mission = energy_takeoff + energy_cruise
    return energy_per_mission

def size_battery(energy_per_mission):

    battery_mass = energy_per_mission / battery_energy_density  # kg
    return battery_mass

mtow = payload_mass_to_mtow(payload_mass)  # kg
S = calculate_wing_surface_area(mtow)  # m^2
D = calculate_drag_cruise(S, V_cruise)  # N
T = calculate_thrust_cruise(D)  # N
P_cruise = calculate_power_cruise(T, V_cruise)  # W
energy_per_mission = calculate_evergy_per_mission(P_cruise, S)  # J
battery_mass = size_battery(energy_per_mission)  # Wh

print(mtow, S, D, T, P_cruise, energy_per_mission, battery_mass)
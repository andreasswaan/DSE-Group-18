import numpy as np
import matplotlib.pyplot as plt

A = 3.0  # Example conversion factor for MTOW calculation
B = 2.0  # Example conversion factor for MTOW calculation

η_prop = 0.85  # Propeller efficiency
η_elec = 0.95  # Motor efficiency
L_over_D_cruise = 10.0  # Lift-to-drag ratio during cruise
payload_mass = 3 # Payload weight in kg
V_cruise = 15.0  # Cruise speed in m/s
R_maxPL = 5000 # Range at max payload in m
R_0PL = 10000 # Range at 0 payload in m 
e_bat = 250  # Energy density of the battery in Wh/kg
g = 9.80665 # Acceleration due to gravity in m/s^2
ρ = 1.225  # Air density in kg/m^3


def payload_mass_to_mtow(payload_mass):
    """Convert payload mass to maximum takeoff weight (MTOW) in kg.
    
    Args:
        payload_mass (float): Payload mass in kg.
        
    Returns:
        float: Maximum takeoff weight (MTOW) in kg.
    """
    mtow = A * payload_mass + B # Example conversion factor
    return mtow  # Example conversion factor


def calc_battery_mass(R, W_OE, W_PL):
    
    W_bat = (W_OE + W_PL)/(η_prop * η_elec * L_over_D_cruise * e_bat/g - R)

    return W_bat

def calc_CD_cruise(S, W):
    CD_cruise = W / (0.5 * ρ * V_cruise**2 * S) / L_over_D_cruise  # Drag coefficient during cruise

    return CD_cruise

def calc_energy_per_mission(S, CD_cruise):
    """Calculate energy required for the entire mission.
    
    Returns:
        float: Energy required in J.
    """
    E_cruise_max_PL = 0.5 * ρ * V_cruise**2 * S * CD_cruise * R_maxPL / η_prop  # Energy required for cruise
    E_cruise_0PL = 0.5 * ρ * V_cruise**2 * S * CD_cruise * R_0PL / η_prop  # Energy required for cruise
    E_total = E_cruise_max_PL + E_cruise_0PL  # Total energy required for the mission
    
    return E_total  # Energy in J
    

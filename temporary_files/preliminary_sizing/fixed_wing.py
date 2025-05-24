import numpy as np
from constants import *
from read_database import get_data_from_database, get_regression_plane


mission_profile = {
    "cruise_OEW": 10000,
    "cruise_MTOW": 15000,
    "loitering_time": 5 * 60,
    "TO_LD": 3,  # not applicable for this mission
    "cruise_h": 500,
    "cruise_speed": 15,
}
"""
    cruise OEW length [m],
    cruise MTOW length [m],
    loitering time [s], 
    #TO/landing [#], 
    # cruise height [ft], 
    # cruise speed [m/s]
"""

V_stall = 10  # Stall speed in m/s (example value, adjust as needed)
W_S = 110.25  # Wing loading in N/m²
W_P = 0.163  # Power loading in N/W
stat_miles_to_m = 1609.34  # Conversion factor from statute miles to meters
c_p = 0.7
m_s_to_mph = 2.23694  # Conversion factor from m/s to mph
FF_startup = 0.999  # (adjusted to 0.999 from 0.998 as it is electric)
FF_taxi = 1
std_climb = 4000  # ft
FF_descent = 1 - (1 - 0.998) / (std_climb / mission_profile["cruise_h"])
FF_climb = 1 - (1 - 0.995) / (
    std_climb / mission_profile["cruise_h"]
)  # Fuel flow during climb
n_climb = 3
n_descent = 3


db_filter_max_payload = 4  # kg, maximum payload for database filter
db_filter_max_range = 40  # km, maximum range for database filter
db = get_data_from_database()


# It is assumed that the landing, taxi, shutdown phase is already accounted for by the startup and taxi fuel flow rates.


def payload_mass_to_mtow(payload_mass, R):
    """Convert payload mass to maximum takeoff weight (MTOW) in kg.

    Args:
        payload_mass (float): Payload mass in kg.
        R (float): Range in km.

    Returns:
        float: Maximum takeoff weight (MTOW) in kg.
    """
    [A, B], C = get_regression_plane(db, db_filter_max_payload, db_filter_max_range)
    mtow = A * payload_mass + B * R / 1000 + C  # Example conversion factor
    return mtow  # Example conversion factor


def calc_cruise_FF():
    R_maxPL = mission_profile["cruise_MTOW"]
    FF_maxPL = 1 / np.exp(
        R_maxPL / stat_miles_to_m * 1 / (375 * η_prop / c_p * L_over_D_cruise)
    )
    R_zeroPL = mission_profile["cruise_OEW"]
    FF_0PL = 1 / np.exp(
        R_zeroPL / stat_miles_to_m * 1 / (375 * η_prop / c_p * L_over_D_cruise)
    )
    FF_cruise = FF_maxPL * FF_0PL
    return FF_cruise  # Fuel flow in kg/s


def calc_loiter_FF():
    E_loiter = mission_profile["loitering_time"] / 3600
    V_s = V_stall * m_s_to_mph  # Convert stall speed to mph
    FF_loiter = 1 / np.exp(E_loiter / (375 / V_s) / (η_prop / c_p) / L_over_D_cruise)
    return FF_loiter  # Fuel flow in kg/s


# def calc_CD_cruise(S, W):
#     CD_cruise = W / (0.5 * ρ * V_cruise**2 * S) / L_over_D_cruise  # Drag coefficient during cruise

#     return CD_cruise


def calc_battery_mass_cruise(R, MTOW):

    W_bat_cruise = (MTOW * R) / (
        η_prop * η_elec * L_over_D_cruise * battery_energy_density / g
    )

    return W_bat_cruise  # Battery mass in kg


def FF_others_to_FF_cruise_ratio_used():
    """Calculate the ratio of cruise fuel flow to total fuel flow.

    Returns:
        float: Ratio of cruise fuel flow to total fuel flow.
    """
    FF_cruise = calc_cruise_FF()
    FF_loiter = calc_loiter_FF()
    FF_others = (
        FF_startup * FF_taxi * FF_climb**n_climb * FF_descent**n_descent * FF_loiter
    )  # Other fuel flow rates

    return (1 - FF_others) / (
        1 - FF_cruise
    )  # Ratio of cruise fuel flow to total fuel flow


MTOW = payload_mass_to_mtow(
    payload_mass, R=mission_profile["cruise_MTOW"]
)  # Convert payload mass to MTOW

W_bat_cruise_maxPL = calc_battery_mass_cruise(
    mission_profile["cruise_MTOW"], MTOW
)  # Calculate battery mass for cruise

W_bat_cruise_0PL = calc_battery_mass_cruise(
    mission_profile["cruise_OEW"], MTOW
)  # Calculate battery mass for cruise at 0 payload

W_bat_cruise = W_bat_cruise_0PL + W_bat_cruise_maxPL  # Total battery mass for cruise

ratio_FF_others_to_FF_cruise = (
    FF_others_to_FF_cruise_ratio_used()
)  # Calculate ratio of cruise fuel flow to total fuel flow

W_bat_total = ((1 + ratio_FF_others_to_FF_cruise) * W_bat_cruise) / (
    1 - battery_lowest_limit
)  # Total battery mass

S = MTOW * g / W_S  # Wing area in m²

P_max = MTOW * g / W_P  # Power in W

V_cruise = mission_profile["cruise_speed"]
Cl_cruise = MTOW * g / (0.5 * ρ * V_cruise**2 * S)  # Lift coefficient during cruise


print(f"MTOW: {MTOW} kg")
print(f"Battery mass for cruise: {W_bat_cruise} kg")
print(f"Ratio of cruise fuel flow to total fuel flow: {ratio_FF_others_to_FF_cruise}")
print(f"Total battery mass: {W_bat_total} kg")
print(f"Wing area: {S} m²")
print(f"Takeoff Power: {P_max} W")
print(f"Lift coefficient during cruise: {Cl_cruise}")


import matplotlib.pyplot as plt

# Calculate the fuel fractions for each phase
FF_cruise = calc_cruise_FF()
FF_loiter = calc_loiter_FF()
FF_others = FF_startup * FF_taxi * FF_climb**n_climb * FF_descent**n_descent * FF_loiter
# print(f"Fuel flow during cruise: {FF_cruise} kg/s")
# print(f"Fuel flow during loiter: {FF_loiter} kg/s")
# print(f"Fuel flow during startup: {FF_startup} kg/s")
# print(f"Fuel flow during taxi: {FF_taxi} kg/s")
# print(f"Fuel flow during climb: {FF_climb} kg/s")
# print(f"Fuel flow during descent: {FF_descent} kg/s")
# print(f"Fuel flow during others: {FF_others} kg/s")
# Normalize the fuel fractions to represent their contribution to total energy consumption
total_FF = FF_cruise * FF_others
energy_breakdown = {
    "Startup+Landing+Shutdown": (1 - FF_startup) / total_FF,
    "Taxi": (1 - FF_taxi) / FF_others,
    "Climb": (1 - (FF_climb**n_climb)) / total_FF,
    "Cruise": (1 - FF_cruise) / total_FF,
    "Descent": (1 - (FF_descent**n_descent)) / total_FF,
    "Loiter": (1 - FF_loiter) / total_FF,
}

# Create the pie chart
labels = energy_breakdown.keys()
sizes = energy_breakdown.values()
colors = ["gold", "lightblue", "lightgreen", "orange", "pink", "purple"]
explode = (0.1, 0, 0, 0.1, 0, 0)  # Highlight Startup and Cruise phases

plt.figure(figsize=(8, 8))
plt.pie(
    sizes,
    labels=labels,
    autopct="%1.1f%%",
    startangle=140,
    colors=colors,
    explode=explode,
)
plt.title("Energy Consumption Breakdown per Flight Mission")
plt.savefig("energy_consumption_breakdown.png", dpi=300, bbox_inches="tight")
plt.close()

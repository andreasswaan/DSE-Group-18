import numpy as np

A = 2.616  # Jan regression factor for MTOW calculation
B = 0.236  # Jan regression factor for MTOW calculation
C = -0.717  # Jan regression factor for MTOW calculation

V_s = 10  # Stall speed in m/s (example value, adjust as needed)
W_S = 110.25 # Wing loading in N/m²
W_P = 0.163 # Power loading in N/W

η_prop = 0.85  # Propeller efficiency
η_elec = 0.95  # Motor efficiency
L_over_D_cruise = 10.0  # Lift-to-drag ratio during cruise
payload_mass = 3 # Payload weight in kg
V_cruise = 15.0  # Cruise speed in m/s
R_maxPL = 5000 # Range at max payload in m
R_0PL = 10000 # Range at 0 payload in m 
E_loiter = 300/3600 # Time for loitering in seconds
e_bat = 250*3600  # Energy density of the battery in J/kg
g = 9.80665 # Acceleration due to gravity in m/s^2
ρ = 1.225  # Air density in kg/m^3
stat_miles_to_m = 1609.34  # Conversion factor from statute miles to meters
η_p = 0.7
c_p = 0.7
m_s_to_mph = 2.23694 # Conversion factor from m/s to mph
V_stall = V_s
FF_startup = 0.999 # (adjusted to 0.999 from 0.998 as it is electric)
FF_taxi = 1
std_climb = 4000
climb_altitude = 500 # Climb altitude in m
FF_descent = 1-(1-0.998)/(std_climb/climb_altitude)
FF_climb = 1-(1-0.995)/(std_climb/climb_altitude) # Fuel flow during climb
n_climb = 3
n_descent = 3

# It is assumed that the landing, taxi, shutdown phase is already accounted for by the startup and taxi fuel flow rates.


def payload_mass_to_mtow(payload_mass, R):
    """Convert payload mass to maximum takeoff weight (MTOW) in kg.
    
    Args:
        payload_mass (float): Payload mass in kg.
        
    Returns:
        float: Maximum takeoff weight (MTOW) in kg.
    """
    mtow = A * payload_mass + B * R + C # Example conversion factor
    return mtow  # Example conversion factor

def calc_cruise_FF():

    FF_maxPL = 1/np.exp(R_maxPL/stat_miles_to_m * 1/(375*η_p/c_p*L_over_D_cruise))
    FF_0PL = 1/np.exp(R_0PL/stat_miles_to_m * 1/(375*η_p/c_p*L_over_D_cruise))
    FF_cruise = FF_maxPL * FF_0PL
    return FF_cruise  # Fuel flow in kg/s

def calc_loiter_FF():
    V_s = V_stall * m_s_to_mph  # Convert stall speed to mph
    FF_loiter = 1/np.exp(E_loiter /(375/V_s)/(η_p/c_p)/L_over_D_cruise)
    return FF_loiter  # Fuel flow in kg/s

# def calc_CD_cruise(S, W):
#     CD_cruise = W / (0.5 * ρ * V_cruise**2 * S) / L_over_D_cruise  # Drag coefficient during cruise

#     return CD_cruise

def calc_battery_mass_cruise(R, MTOW):
    
    W_bat_cruise = (MTOW*R)/(η_prop * η_elec * L_over_D_cruise * e_bat/g)

    return W_bat_cruise  # Battery mass in kg

def FF_others_to_FF_cruise_ratio_used():
    """Calculate the ratio of cruise fuel flow to total fuel flow.
    
    Returns:
        float: Ratio of cruise fuel flow to total fuel flow.
    """
    FF_cruise = calc_cruise_FF()
    FF_loiter = calc_loiter_FF()
    FF_others = FF_startup * FF_taxi * FF_climb**n_climb * FF_descent**n_descent * FF_loiter  # Other fuel flow rates
    
    return (1-FF_others)/(1-FF_cruise)  # Ratio of cruise fuel flow to total fuel flow


MTOW = payload_mass_to_mtow(payload_mass, R=13)  # Convert payload mass to MTOW
print(f"MTOW: {MTOW} kg")

W_bat_cruise_maxPL = calc_battery_mass_cruise(R_maxPL, MTOW)  # Calculate battery mass for cruise
W_bat_cruise_0PL = calc_battery_mass_cruise(R_0PL, MTOW)  # Calculate battery mass for cruise at 0 payload
W_bat_cruise = W_bat_cruise_0PL + W_bat_cruise_maxPL  # Total battery mass for cruise
print(f"Battery mass for cruise: {W_bat_cruise} kg")

ratio_FF_others_to_FF_cruise = FF_others_to_FF_cruise_ratio_used()  # Calculate ratio of cruise fuel flow to total fuel flow
print(f"Ratio of cruise fuel flow to total fuel flow: {ratio_FF_others_to_FF_cruise}")

W_bat_total = (1+ratio_FF_others_to_FF_cruise) * W_bat_cruise  # Total battery mass
print(f"Total battery mass: {W_bat_total} kg")

S = MTOW * g/ W_S  # Wing area in m²
print(f"Wing area: {S} m²")

P_max = MTOW * g/ W_P  # Power in W
print(f"Takeoff Power: {P_max} W")

Cl_cruise = MTOW * g / (0.5 * ρ * V_cruise**2 * S)  # Lift coefficient during cruise
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
    "Startup+Landing+Shutdown": (1-FF_startup) / total_FF,
    "Taxi": (1-FF_taxi)/FF_others,
    "Climb": (1-(FF_climb**n_climb)) / total_FF,
    "Cruise": (1-FF_cruise) / total_FF,
    "Descent": (1-(FF_descent**n_descent)) / total_FF,
    "Loiter": (1-FF_loiter) / total_FF
}

# Create the pie chart
labels = energy_breakdown.keys()
sizes = energy_breakdown.values()
colors = ['gold', 'lightblue', 'lightgreen', 'orange', 'pink', 'purple']
explode = (0.1, 0, 0, 0.1, 0, 0)  # Highlight Startup and Cruise phases

plt.figure(figsize=(8, 8))
plt.pie(sizes, labels=labels, autopct='%1.1f%%', startangle=140, colors=colors, explode=explode)
plt.title("Energy Consumption Breakdown per Flight Mission")
plt.savefig("energy_consumption_breakdown.png", dpi=300, bbox_inches='tight')
plt.close()
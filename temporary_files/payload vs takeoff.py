import matplotlib.pyplot as plt
import numpy as np

# Constants
V_takeoff = 10  # Takeoff velocity in m/s
cruise_height = 100  # Cruise height in m
Cd = 2
A_drone= 0.45**2+0.3**2
n_rotors=4
rho = 1.225  # Air density in kg/m^3
V = 10  # Velocity in m/s
cruise_time = 60*60
pl=2.5
energy_density = 253*3600
TO_landing_time=60
prop_efficiency = 0.85
electric_efficiency = 0.95
ToverWmax_takeoff = 2  # thrust-to-weight ratio at takeoff (placeholder value)

# Data
Payload_weights = [5, 3, 1, 1.5, 5, 10, 1.5, 1, 2, 4]
takeoff_weights = [16.5, 13, 2.8, 8, 18.5, 20, 5.8, 4.25, 6, 6.7]

# Fit a line of best fit
coefficients = np.polyfit(Payload_weights, takeoff_weights, 1)  # Linear fit
"""
line_of_best_fit = np.poly1d(coefficients)
# Generate x values for the line
x = np.linspace(min(Payload_weights), max(Payload_weights), 100)
y = line_of_best_fit(x)

# Calculate R²
predicted = line_of_best_fit(Payload_weights)
ss_total = np.sum((takeoff_weights - np.mean(takeoff_weights))**2)
ss_residual = np.sum((takeoff_weights - predicted)**2)
r_squared = 1 - (ss_residual / ss_total)

# Plot the data
plt.scatter(Payload_weights, takeoff_weights, color='blue', label='Data Points')
plt.scatter(Payload_weights[-1], takeoff_weights[-1], color='green', label='Data Points')
plt.plot(x, y, color='red', label='Line of Best Fit')

# Add R² value to the plot
plt.text(0.05, 0.95, f"$R^2 = {r_squared:.2f}$", transform=plt.gca().transAxes, fontsize=12, verticalalignment='top')

# Add labels and legend
plt.xlabel('Payload Weight (kg)')
plt.ylabel('Takeoff Weights (kg)')
plt.legend(loc='lower right')  # Move the legend to the bottom-right corner

# Show the plot
plt.show()"""
mtow=(coefficients[0]*pl+coefficients[1])
oew=mtow-pl
def calculate_drag(Cd,A,speed):
    return 0.5 * Cd * A * rho * speed**2

def calculate_power_takeoff(V_takeoff):
    Thrust = ToverWmax_takeoff * mtow * 9.81 + calculate_drag(Cd, A_drone, V_takeoff)
    P_takeoff = Thrust * V_takeoff / prop_efficiency / electric_efficiency
    return P_takeoff

def size_props(P_takeoff):
    vert_prop_total_area = (ToverWmax_takeoff * mtow*9.81)**3 / (2 * rho * P_takeoff**2)
    vert_prop_area = vert_prop_total_area / n_rotors
    vert_prop_diameter = 2 * np.sqrt(vert_prop_area / np.pi)  # m, diameter of propellers
    return vert_prop_total_area, vert_prop_diameter

A_prop, vert_prop_diameter = size_props(calculate_power_takeoff(V_takeoff))
print("Vertical propeller total area:", A_prop, "m^2")
print("Vertical propeller diameter:", vert_prop_diameter, "m")

def take_off_landing_energy(mass):
    energy_takeoff_hover = 2 * (mass*9.81) ** 1.5 / (np.sqrt(2 * rho * A_prop)) * cruise_height/ V_takeoff
    energy_takeoff_climb = 9.81*mass * cruise_height
    energy_takeoff_drag = calculate_drag(Cd, A_drone, V_takeoff) * cruise_height 
    energy_takeoff = (energy_takeoff_hover + energy_takeoff_climb + energy_takeoff_drag)/electric_efficiency/prop_efficiency
    return energy_takeoff

def calculate_p_cruise(mass, speed):
    p_hover=(mass*9.81)**(3/2)/(np.sqrt(2*rho*A_prop))/prop_efficiency/electric_efficiency
    D = 0.5 * Cd * A_drone * rho * speed**2
    alpha = np.arctan(D / (mass * 9.81))
    p_cruise = p_hover / np.cos(alpha)
    return p_cruise 

p_cruise_mtow = calculate_p_cruise(mtow, V)
p_cruise_oew= calculate_p_cruise(oew, V)
def calculate_battery_mass(mission_profile, graph=False):
    cruiseOEW_length, cruiseMTOW_length, loiter_time, TO_landing = mission_profile
    p_hover_mtow = calculate_p_cruise(mtow, 0)
    
    # Energy calculations for each phase
    energy_takeoff_landing = take_off_landing_energy(mtow)
    energy_cruise_MTOW = p_cruise_mtow * cruiseMTOW_length / V
    energy_cruise_OEW = p_cruise_oew * cruiseOEW_length / V
    energy_loiter = loiter_time * p_hover_mtow
    
    # Total energy
    energy_total = (energy_cruise_MTOW + energy_cruise_OEW + energy_loiter + TO_landing * energy_takeoff_landing)
    battery_mass = energy_total / energy_density /0.7 # in kg
    
    # Create a pie chart
    if graph:
        energy_phases = [energy_takeoff_landing, energy_cruise_MTOW, energy_cruise_OEW, energy_loiter]
        labels = ['Takeoff/Landing', 'Cruise (MTOW)', 'Cruise (OEW)', 'Loiter']
        plt.figure(figsize=(8, 6))
        plt.pie(energy_phases, labels=labels, autopct='%1.1f%%', startangle=140, colors=['gold', 'blue', 'green', 'red'])
        plt.title('Energy Consumption per Mission Phase')
        plt.show()
    
    return battery_mass, energy_total/1000000

mission_profile = [0,15000,5*60,3] # [cruiseOEW_length, cruiseMTOW_length, TO/landing]
battery_mass, energy_total=calculate_battery_mass(mission_profile, graph=False)

mission_profile_average = [10000, 5000, 0, 6]
battery_mass_average, energy_total_average=calculate_battery_mass(mission_profile_average, graph=True)
if battery_mass_average>battery_mass:
    print("Average mission profile requires more battery mass")
print("mtow:", mtow, "kg")
print("p_cruise:", p_cruise_mtow, "W")
print("Battery mass:", battery_mass, "kg")
print("Energy total:", energy_total, "MJ")
print("energy total average:", energy_total_average, "MJ")

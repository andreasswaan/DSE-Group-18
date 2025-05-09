import os
import numpy as np
import matplotlib.pyplot as plt
from globals import main_dir

# Constants (everything is not so educated guess!!!)
rho = 1.225  # Air density at sea level (kg/m³)
g = 9.80665     # Gravitational acceleration (m/s²)
V_s = 20.63262  # Stall speed (m/s)
C_L_max_landing = 2.0  # Maximum lift coefficient (landing configuration)
eta_p = 0.8  # Propeller efficiency
c = 5.0      # Climb rate (m/s)
A = 9        # Aspect ratio
e = 0.7      # Oswald efficiency factor
C_D0 = 0.0335  # Zero-lift drag coefficient
V_climb = 70  # Climb speed (m/s)
V_cruise = 30 # Cruise speed (m/s)
TOP = 100     # Take-off parameter (from Raymer/statistics)

# Wing loading range (N/m²)
W_S = np.linspace(100, 3000, 100)

# --- Constraints Calculation ---
# 1. Stall Speed Constraint (W/S ≤ 0.5 * ρ * V_s² * C_L_max)
W_S_max_stall = 0.5 * rho * V_s**2 * C_L_max_landing

# 2. Take-off Constraint (W/P = (W/S) / (TOP * C_L_TO))
C_L_TO = 1.8 # Take-off lift coefficient
W_P_takeoff = W_S / (TOP * C_L_TO)

# 3. Cruise Constraint (W/P = 0.9/0.8 eta_p / (C_D0 * 0.5 * rho * V_cruise**3/(0.8W/S)+0.8W/S * 1/(np.pi * A * e * 0.5 * rho * V_cruise)))
term_cruise1 = C_D0 * 0.5 * rho * V_cruise**3/(0.8 * W_S)
term_cruise2 = 0.8 * W_S * 1/(np.pi * A * e * 0.5 * rho * V_cruise)
W_P_cruise = 0.9/0.8 * eta_p / (term_cruise1 + term_cruise2)

# 4. Climb Rate Constraint (W/P ≥ [1/η_p] * [1/V * (c/g + 0.5*ρ*V²*C_D0/(W/S) + (2/(ρ*V²*e*π*A))*(W/S)])
term_climb1 = c / g
term_climb2 = 0.5 * rho * V_climb**2 * C_D0 / W_S
term_climb3 = (2 / (rho * V_climb**2 * e * np.pi * A)) * W_S
W_P_climb = (1 / eta_p) * (1 / V_climb) * (term_climb1 + term_climb2 + term_climb3)

# --- Plotting ---
plt.figure(figsize=(10, 6))

# Stall constraint (vertical line)
plt.axvline(x=W_S_max_stall, color='r', linestyle='--',
            label=f'Stall Limit (W/S ≤ {W_S_max_stall:.1f} N/m²)')

# Take-off constraint
plt.plot(W_S, W_P_takeoff, 'b-', label='Take-off Constraint (TOP)')

# Climb constraint
plt.plot(W_S, W_P_climb, 'g-', label='Climb Constraint')

# Feasible region (shading)
plt.fill_between(W_S, 0, np.minimum(W_P_takeoff, W_P_climb),
                 where=(W_S <= W_S_max_stall),
                 color='lightgreen', alpha=0.3, label='Feasible Region')

# Design point (highest W/S, highest W/P in feasible region)
design_W_S = W_S_max_stall * 0.9  # Example: 90% of stall limit
design_W_P = np.minimum(
    np.interp(design_W_S, W_S, W_P_takeoff),
    np.interp(design_W_S, W_S, W_P_climb)
)
plot_path = os.path.join(main_dir, 'fixed_wing_plots\\WP_WS_diagram.png')
os.makedirs(os.path.dirname(plot_path), exist_ok=True)
plt.scatter(design_W_S, design_W_P, color='k', s=100, label='Design Point')
plt.annotate(f'Design Point:\nW/S={design_W_S:.1f} N/m²\nW/P={design_W_P:.1f} N/W',
             (design_W_S, design_W_P), xytext=(10, 10), textcoords='offset points')

# Formatting
plt.xlabel('Wing Loading (W/S) [N/m²]')
plt.ylabel('Power Loading (W/P) [N/W]')
plt.title('W/P vs. W/S Diagram for Propeller Aircraft')
plt.grid(True)
plt.legend()
plt.savefig(plot_path, dpi=300, bbox_inches='tight')
# plt.ylim(0, 100)  # Adjust y-axis limit based on your data
# plt.xlim(0, 3000)
plt.close()
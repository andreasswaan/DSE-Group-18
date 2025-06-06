import numpy as np
import matplotlib.pyplot as plt

# Physical constants
g = 9.81  # m/s²

# Design parameters
V_cruise = 12  # m/s
V_dive = 1.5 * V_cruise  # conservative estimate (V_never_exceed and add factor for V_dive)
V_range = np.linspace(0, V_dive, 500)

# Load factor limits
n_pos_limit = 3.5  # positive load factor limit
n_neg_limit = -1.5  # negative load factor limit

# Stall speed (level flight at n=1)
V_stall = 8  # m/s (estimate)

# Maneuvering envelope
n_stall_pos = (V_range / V_stall) ** 2
n_stall_pos[V_range > V_stall * np.sqrt(n_pos_limit)] = np.nan

n_stall_neg = -(V_range / V_stall) ** 2
n_stall_neg[V_range > V_stall * np.sqrt(-n_neg_limit)] = np.nan

# Gust envelope parameters (simplified)
rho = 1.225  # kg/m³
a = 5.7  # lift curve slope [1/rad]
S = 0.5  # wing area in m²
W = 5 * g  # weight in N (5 kg)
U_de = [7.5, 15, 22.5]  # gust velocities in m/s

gust_lines = []
for u in U_de:
    delta_n = (rho * V_range * u * a) / (2 * W / S)
    gust_lines.append(1 + delta_n)
    gust_lines.append(1 - delta_n)

# No-slip condition as additional boundary
mu_static = 0.4
n_noslip = mu_static / np.sqrt(1 + mu_static**2)  # max lateral n
n_noslip_line = np.ones_like(V_range) * n_noslip

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(V_range, n_stall_pos, 'b-', label="Positive stall boundary")
plt.plot(V_range, n_stall_neg, 'b-', label="Negative stall boundary")
plt.hlines(n_pos_limit, 0, V_dive, colors='r', linestyles='--', label='Structural limit (+3.5g)')
plt.hlines(n_neg_limit, 0, V_dive, colors='r', linestyles='--', label='Structural limit (-1.5g)')
plt.vlines(V_dive, n_neg_limit, n_pos_limit, colors='k', linestyles='--', label="Dive speed")

# Gust lines
for i, u in enumerate(U_de):
    plt.plot(V_range, gust_lines[2*i], 'g--', alpha=0.6, label=f"Gust +{u} m/s" if i == 0 else None)
    plt.plot(V_range, gust_lines[2*i+1], 'g--', alpha=0.6)

# No slip condition
plt.hlines(n_noslip, 0, V_dive, colors='purple', linestyles='-.', label=f'No-slip limit (~{n_noslip:.2f}g)')

plt.title("Maneuvering and Gust Envelope for Pizza Drone")
plt.xlabel("True Airspeed (m/s)")
plt.ylabel("Load Factor (n)")
plt.ylim(-2, 4)
plt.xlim(0, V_dive + 2)
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

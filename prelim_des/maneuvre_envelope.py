from __future__ import annotations
import os
from typing import TYPE_CHECKING
import numpy as np
import matplotlib.pyplot as plt


from prelim_des.drone import Drone
from prelim_des.mission import Mission
from prelim_des.performance import Performance
from constants import ρ, g
from globals import main_dir

def plot_maneuver_and_gust_envelope(
    drone: Drone, U_de=None, plot=True
):
    """
    Plot the maneuvering and gust envelope for a given aircraft.
    """
    mass = drone.MTOW[0]
    W = mass * g
    rho = ρ
    S = drone.wing.S[0]
    CL_max = drone.aero.CL_max
    a = drone.aero.CL_slope_aircraft
    mu_static = drone.structure.mu_static
    n_pos_limit = drone.structure.calc_n_max()
    n_neg_limit = drone.structure.calc_n_min()
    V_cruise = drone.perf.V_cruise

    # Reference speeds
    V_B = getattr(drone, "V_B", 0.7 * V_cruise)  # Use attribute or estimate
    V_dive = 1.5 * V_cruise

    # Gust velocities at each reference speed (example values, adjust as needed)
    U_gusts = [12.5, 10.0, 5.0]  # [at V_B, V_cruise, V_dive]

    # Calculate delta_n at each reference speed
    V_refs = [V_B, V_cruise, V_dive]
    gust_points = []
    for V, U_g in zip(V_refs, U_gusts):
        delta_n = (rho * V * U_g * a) / (2 * W / S)
        gust_points.append((V, 1 + delta_n))
    # Mirror for negative
    gust_points_neg = [(V, 1 - ((rho * V * U_g * a) / (2 * W / S))) for V, U_g in zip(V_refs, U_gusts)]

    # Connect the gust points to form the envelope
    V_env = [0] + [pt[0] for pt in gust_points] + [pt[0] for pt in gust_points[::-1]]
    n_env = [1] + [pt[1] for pt in gust_points] + [pt[1] for pt in gust_points_neg[::-1]]

    # Plot the maneuvering envelope as before (stall, structural limits, etc.)
    V_range = np.linspace(0, V_dive, 500)
    V_stall = np.sqrt((2 * W) / (rho * S * CL_max))
    n_stall_pos = (V_range / V_stall) ** 2
    n_stall_pos[V_range > V_stall * np.sqrt(n_pos_limit)] = np.nan
    n_stall_neg = -(V_range / V_stall) ** 2
    n_stall_neg[V_range > V_stall * np.sqrt(-n_neg_limit)] = np.nan
    
    n_max_gust = np.nanmax(n_env)
    n_max_maneuver = np.nanmax(n_stall_pos)
    n_max = max(n_max_gust, n_max_maneuver)
    
    if plot:
        plt.figure(figsize=(10, 6))

        # Draw lines from (0,1) to each positive gust point
        for V, n in gust_points:
            plt.plot([0, V], [1, n], 'g--', lw=2)
        # Draw lines from (0,1) to each negative gust point
        for V, n in gust_points_neg:
            plt.plot([0, V], [1, n], 'g--', lw=2)


        plt.plot(V_range, n_stall_pos, 'b-', label="Stall boundary (positive)")
        plt.plot(V_range, n_stall_neg, 'b-', label="Stall boundary (negative)")
        plt.hlines(n_pos_limit, 0, V_dive, colors='r', linestyles='--', label=f'CS Structural limit (+{n_pos_limit}g)')
        plt.hlines(n_neg_limit, 0, V_dive, colors='r', linestyles='--', label=f'CS Structural limit ({n_neg_limit}g)')
        plt.vlines(V_dive, n_neg_limit, n_pos_limit, colors='k', linestyles='--', label="Dive speed")
        plt.plot(V_env, n_env, 'g-', lw=2, label="Gust Envelope")       

        # --- Add this block for n_max ---
        # Find the maximum n from both envelopes (ignore NaNs)
    
        plt.hlines(n_max, 0, V_dive, colors='orange', linestyles='-', linewidth=2, label='n_max')
        # Annotate the n_max line
        plt.text(V_dive * 0.95, n_max + 0.05, "n_max", color='orange', fontsize=11, ha='right', va='bottom', fontweight='bold')

        plt.title("Maneuvering and Gust Envelope for Pizza Drone")
        plt.xlabel("True Airspeed (m/s)")
        plt.ylabel("Load Factor (n)")
        plt.xlim(0, V_dive + 2)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(main_dir, "prelim_des", "maneuver_gust_envelope.svg"), format='svg', dpi=300)
        plt.show()
        
    return n_max
            
# Example usage:
if __name__ == "__main__":
    mission = Mission("DRCCRCCRCCD")
    drone = Drone()
    perf = Performance(drone, mission)
    drone.perf = perf
    drone.class_1_weight_estimate()

    drone.wing.S = perf.wing_area(drone.OEW)
    drone.class_2_weight_estimate()

    # drone.iterative_weight_estimate(plot=True, tolerance=0.01)
    n_max = plot_maneuver_and_gust_envelope(drone)
    print(f"Maximum load factor (n_max): {n_max:.2f}")
from __future__ import annotations
import os
from typing import TYPE_CHECKING


if TYPE_CHECKING:
    from prelim_des.drone import Drone
import numpy as np
import matplotlib.pyplot as plt

from prelim_des.mission import Mission
from prelim_des.performance import Performance
from prelim_des.constants import ρ, g
from globals import main_dir


def plot_maneuver_and_gust_envelope(drone: Drone, U_de=None, plot=False):
    """
    Plot the maneuvering and gust envelope for a given aircraft.
    """
    mass = float(drone.MTOW)
    W = mass * g
    rho = ρ
    S = float(drone.wing.S)
    CL_max = drone.aero.CL_max
    a = drone.aero.CL_slope_aircraft
    mu_static = drone.structure.mu_static
    n_pos_limit = drone.structure.calc_n_max()
    n_neg_limit = drone.structure.calc_n_min()
    V_cruise = drone.perf.V_cruise

    # Reference speeds
    V_B = getattr(drone, "V_B", 0.9 * V_cruise)  # Use attribute or estimate
    V_dive = 1.5 * V_cruise

    # Gust velocities at each reference speed (example values, adjust as needed)
    # U_gusts = [12.5, 10.0, 5.0]  # [at V_B, V_cruise, V_dive]
    U_gusts = [5.0, 4.70, 3.0]  # [at V_B, V_cruise, V_dive]

    # Calculate delta_n at each reference speed
    V_refs = [V_B, V_cruise, V_dive]
    gust_points = []
    for V, U_g in zip(V_refs, U_gusts):
        delta_n = (rho * V * U_g * a) / (2 * W / S)
        gust_points.append((V, 1 + delta_n))
    # Mirror for negative
    gust_points_neg = [
        (V, 1 - ((rho * V * U_g * a) / (2 * W / S))) for V, U_g in zip(V_refs, U_gusts)
    ]

    # Connect the gust points to form the envelope
    # Collect the gust envelope points in order (upper, then lower in reverse)
    V_env = (
        [0]
        + [pt[0] for pt in gust_points]
        + [pt[0] for pt in gust_points_neg[::-1]]
        + [0]
    )
    n_env = (
        [1]
        + [pt[1] for pt in gust_points]
        + [pt[1] for pt in gust_points_neg[::-1]]
        + [1]
    )

    # Plot the maneuvering envelope as before (stall, structural limits, etc.)
    V_range = np.linspace(0, V_dive, 500)
    V_stall = np.sqrt((2 * W) / (rho * S * CL_max))
    n_stall_pos = (V_range / V_stall) ** 2
    n_stall_pos[V_range > V_stall * np.sqrt(n_pos_limit)] = np.nan
    n_stall_neg = -((V_range / V_stall) ** 2)
    n_stall_neg[V_range > V_stall * np.sqrt(-n_neg_limit)] = np.nan

    n_max_gust = np.nanmax(n_env)
    n_max_maneuver = np.nanmax(n_stall_pos)
    n_max = max(n_max_gust, n_max_maneuver)

    if plot:
        plt.figure(figsize=(10, 6))

        # Draw lines from (0,1) to each positive gust point
        for V, n in gust_points:
            plt.plot([0, V], [1, n], "g--", lw=2)
        # Draw lines from (0,1) to each negative gust point
        for V, n in gust_points_neg:
            plt.plot([0, V], [1, n], "g--", lw=2)

        plt.plot(V_range, n_stall_pos, "b-", label="Stall boundary")
        plt.plot(
            V_range,
            n_stall_neg,
            "b-",
        )  # label="Stall boundary (negative)")
        plt.hlines(
            n_pos_limit,
            0,
            V_dive,
            colors="r",
            linestyles="--",
            label=f"CS Structural limit",
            # label=f"CS Structural limit (+{n_pos_limit}g)",
        )
        plt.hlines(
            n_neg_limit,
            0,
            V_dive,
            colors="r",
            linestyles="--",
            # label=f"CS Structural limit ({n_neg_limit}g)",
        )
        plt.vlines(
            V_dive,
            n_neg_limit,
            n_pos_limit,
            colors="k",
            linestyles="--",
            # label="Dive speed",
        )
        # Plot the filled gust envelope
        plt.fill(
            V_env,
            n_env,
            color="green",
            alpha=0.15,
        )  # label="Gust Envelope")
        # Optionally, plot the outline as a solid line
        plt.plot(V_env, n_env, "g-", lw=2, label="Gust Envelope")

        # --- Add this block for n_max ---
        # Find the maximum n from both envelopes (ignore NaNs)

        plt.hlines(
            n_max,
            0,
            V_dive,
            colors="orange",
            linestyles="-",
            linewidth=2,
            label="n_max",
        )
        # Annotate the n_max line
        # plt.text(
        #     V_dive * 0.95,
        #     n_max + 0.05,
        #     "n_max",
        #     color="orange",
        #     fontsize=14,
        #     ha="right",
        #     va="bottom",
        #     fontweight="bold",
        # )

        # Find indices for transitions
        idx_upper = np.argmax(V_range > V_stall * np.sqrt(n_pos_limit))
        if idx_upper == 0:
            idx_upper = len(V_range)
        idx_lower = np.argmax(V_range > V_stall * np.sqrt(-n_neg_limit))
        if idx_lower == 0:
            idx_lower = len(V_range)

        # 1. Fill between stall boundaries up to upper limit
        plt.fill_between(
            V_range[:idx_upper],
            n_stall_pos[:idx_upper],
            n_stall_neg[:idx_upper],
            color="blue",
            alpha=0.10,
            # label="Maneuvering Envelope",
        )

        # 2. Fill between structural upper and stall lower up to lower limit
        if idx_upper < idx_lower:
            plt.fill_between(
                V_range[idx_upper:idx_lower],
                n_pos_limit,
                n_stall_neg[idx_upper:idx_lower],
                color="blue",
                alpha=0.10,
            )

        # 3. Fill between structural limits for the rest
        if idx_lower < len(V_range):
            plt.fill_between(
                V_range[idx_lower:],
                n_stall_pos[idx_lower:],
                n_neg_limit,
                color="blue",
                alpha=0.10,
            )

        # Mark V_B, V_C, and V_dive on the x-axis
        plt.axvline(V_B, color="purple", linestyle=":", linewidth=2)
        plt.text(
            V_B - 0.5,
            plt.ylim()[0] + 6.5,
            r"$V_B$",
            color="purple",
            fontsize=15,
            ha="center",
            va="bottom",
            fontweight="bold",
        )

        plt.axvline(V_cruise, color="brown", linestyle=":", linewidth=2)
        plt.text(
            V_cruise - 0.5,
            plt.ylim()[0] + 6.5,
            r"$V_C$",
            color="brown",
            fontsize=15,
            ha="center",
            va="bottom",
            fontweight="bold",
        )

        plt.axvline(V_dive, color="black", linestyle=":", linewidth=2)
        plt.text(
            V_dive - 0.5,
            plt.ylim()[0] + 6.5,
            r"$V_{D}$",
            color="black",
            fontsize=15,
            ha="center",
            va="bottom",
            fontweight="bold",
        )

        plt.xlabel("True Airspeed [m/s]", fontsize=16, labelpad=10)
        plt.ylabel("Load Factor n [-]", fontsize=16, labelpad=10)
        plt.tick_params(labelsize=14)
        plt.xlim(0, V_dive + 1)
        plt.ylim(-2, n_max + 2)
        plt.grid(True, alpha=0.5)
        plt.legend(fontsize=14, loc="best")
        plt.tight_layout()
        plt.savefig(
            os.path.join(main_dir, "prelim_des", "plots", "maneuver_gust_envelope.pdf"),
            format="pdf",
            dpi=300,
            bbox_inches="tight",
        )
        plt.close()

    return n_max


# Example usage:
if __name__ == "__main__":
    from prelim_des.drone import Drone

    mission = Mission("DRCCRCCD")
    drone = Drone()
    perf = Performance(drone, mission)
    drone.perf = perf
    drone.class_1_weight_estimate()

    drone.wing.S = perf.wing_area(drone.OEW)
    drone.class_2_weight_estimate()

    # drone.iterative_weight_estimate(plot=True, tolerance=0.01)
    n_max = plot_maneuver_and_gust_envelope(drone, plot=True)
    print(f"Maximum load factor (n_max): {n_max:.2f}")

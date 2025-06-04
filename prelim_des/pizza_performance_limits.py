import numpy as np
import matplotlib.pyplot as plt
from prelim_des.constants import g
from prelim_des.utils.import_toml import load_toml


def max_ax_for_bank_angle(theta_deg, toml):
    """
    For a given bank angle (deg), return:
      - max ax before slip (m/s²)
      - max ax before folding (m/s²)
    """
    mu_static = toml["config"]["payload"]["static_friction_coeff"]
    box_width = toml["config"]["payload"]["box_width"]
    pizza_diameter = 0.40
    pizza_radius = pizza_diameter / 2

    theta = np.radians(theta_deg)
    # Slip: ax² + (g sinθ)² <= (μg cosθ)²
    slip_limit = np.sqrt(
        (mu_static * g * np.cos(theta)) ** 2 - (g * np.sin(theta)) ** 2
    )
    slip_limit = np.where(slip_limit > 0, slip_limit, 0)

    # Folding: R sin(phi) <= (R - margin)
    support_margin = (box_width - pizza_diameter) / 2
    phi_fold = np.arcsin((pizza_radius - support_margin) / pizza_radius)
    ax_fold = g * np.tan(phi_fold)
    return slip_limit, ax_fold


def max_bank_angle_for_ax(ax, toml):
    """
    For a given ax (m/s²), return:
      - max bank angle before slip (deg)
      - max bank angle before folding (deg)
    """
    mu_static = toml["config"]["payload"]["static_friction_coeff"]
    box_width = toml["config"]["payload"]["box_width"]
    pizza_diameter = 0.40
    pizza_radius = pizza_diameter / 2

    # Slip: tanθ <= μ
    slip_angle = np.degrees(np.arctan(mu_static))
    # Folding: sin(phi) = (R - margin)/R, phi = arctan(ax/g)
    support_margin = (box_width - pizza_diameter) / 2
    phi_fold = np.arcsin((pizza_radius - support_margin) / pizza_radius)
    ax_fold = g * np.tan(phi_fold)
    # For a given ax, bank angle at equilibrium: θ = arctan(ax/g)
    theta = np.degrees(np.arctan(ax / g))
    # Folding limit: only valid if ax <= ax_fold
    theta_folding = np.where(ax <= ax_fold, theta, np.nan)
    # Slip limit: only valid if theta <= slip_angle
    theta_slip = np.where(theta <= slip_angle, theta, np.nan)
    return theta_slip, theta_folding


def plot_pizza_performance_envelope(toml):
    # Existing plotting code (unchanged)
    box_width = toml["config"]["payload"]["box_width"]
    pizza_diameter = 0.40
    pizza_radius = pizza_diameter / 2
    mu_static = toml["config"]["payload"]["static_friction_coeff"]

    theta_vals = np.radians(np.linspace(0, 45, 300))
    ax_vals = np.linspace(0, 5, 300)
    theta_grid, ax_grid = np.meshgrid(theta_vals, ax_vals)
    no_slip_limit = np.sqrt(
        ax_grid**2 + (g * np.sin(theta_grid)) ** 2
    ) <= mu_static * g * np.cos(theta_grid)
    phi_grid = np.arctan2(ax_grid, g)
    support_margin = (box_width - pizza_diameter) / 2
    effective_offset = pizza_radius * np.sin(phi_grid)
    folding_limit = effective_offset <= (pizza_radius - support_margin)

    plt.figure(figsize=(10, 6))
    plt.contourf(
        np.degrees(theta_grid),
        ax_grid,
        no_slip_limit,
        levels=[0, 0.5, 1],
        colors=["red", "green"],
        alpha=0.6,
    )
    plt.contour(
        np.degrees(theta_grid),
        ax_grid,
        folding_limit,
        levels=[0.5],
        colors="blue",
        linestyles="--",
        linewidths=2,
    )
    plt.xlabel("Bank angle θ (degrees)")
    plt.ylabel("Horizontal acceleration $a_x$ (m/s²)")
    plt.title("Pizza Performance Envelope: No Slip and Folding Risk")
    green_patch = plt.Rectangle((0, 0), 1, 1, color="green", label="Safe (No Slip)")
    red_patch = plt.Rectangle((0, 0), 1, 1, color="red", label="Slip Risk")
    blue_line = plt.Line2D(
        [0], [0], color="blue", linestyle="--", label="Folding Limit"
    )
    plt.legend(handles=[green_patch, red_patch, blue_line])
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Lateral acceleration plot
    a_lat_g = np.linspace(0, 2, 500)
    a_lat = a_lat_g * g
    bank_angle_deg = np.degrees(np.arctan(a_lat / g))
    a_lat_slip_limit = mu_static * g
    bank_angle_slip_limit = np.degrees(np.arctan(mu_static))

    plt.figure(figsize=(10, 6))
    plt.plot(
        a_lat_g,
        bank_angle_deg,
        label="Bank angle for given lateral acceleration",
        color="C0",
    )
    plt.axhline(
        bank_angle_slip_limit, color="red", linestyle="--", label="Slip limit (μ·g)"
    )
    plt.fill_between(
        a_lat_g,
        bank_angle_deg,
        bank_angle_slip_limit,
        where=bank_angle_deg >= bank_angle_slip_limit,
        color="red",
        alpha=0.2,
        label="Slip risk zone",
    )
    plt.xlabel("Lateral acceleration (g)")
    plt.ylabel("Bank angle (deg)")
    plt.title("Pizza Stability During Banking Maneuver")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    toml = load_toml()
    plot_pizza_performance_envelope(toml)
    slip, fold = max_ax_for_bank_angle(20, toml)
    print(f"Max ax for 20 deg bank: slip={slip:.2f} m/s², folding={fold:.2f} m/s²")
    slip_angle, fold_angle = max_bank_angle_for_ax(3, toml)
    print(
        f"Max bank angle for ax=3 m/s²: slip={slip_angle:.2f} deg, folding={fold_angle:.2f} deg"
    )

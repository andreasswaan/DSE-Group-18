import numpy as np

from prelim_des.drone import Drone

from .structure_materials import toml


# === LOAD DISTRIBUTIONS ===

def elliptical_lift_distribution(y: float, drone: Drone) -> float:
    """
    Computes lift per unit span (N/m) at spanwise position y from centerline.
    Assumes elliptical distribution. small change hello

    Parameters:
        y (float): Position along span (from root, 0 ≤ y ≤ b/2)

    Returns:
        float: Lift per unit span at y (N/m)
    """
    b = float(drone.wing.span)  # Use the drone's wing span
    CL_max = drone.aero.CL_max
    V_max = toml["config"]["mission"]["max_velocity"]
    L_total = float(drone.aero.lift(V_max, CL_max))  # Total lift at max velocity
    return (4 * L_total / (np.pi * b)) * np.sqrt(1 - (2 * y / b) ** 2)


# def constant_weight_distribution(
#     y: float, W_total: float, drone: Drone
# ) -> float:
#     """
#     Computes weight per unit span (N/m) at spanwise position y from centerline.
#     Assumes constant weight distribution along the wing.

#     Parameters:
#         y (float): Position along span (from root, 0 ≤ y ≤ b/2)
#         b (float): Full wingspan
#         W_total (float): Total weight supported by the wing (N)

#     Returns:
#         float: Weight per unit span at y (N/m)
#     """

#     return W_total / (b / 2)  # Divide by half-span (modelling half wing)


def constant_drag_distribution(drone: Drone) -> float:
    """
    Returns drag per unit span (N/m) at spanwise position y.
    Assumes constant drag distribution along the wing.
    """
    b = float(drone.wing.span)
    D_total = float(drone.aero.drag(toml["config"]["mission"]["max_velocity"]))

    return D_total / (b / 2)  # Divide by half-span (modelling half wing)

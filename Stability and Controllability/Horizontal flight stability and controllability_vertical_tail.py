import math
import matplotlib.pyplot as plt
import numpy as np


def vertical_tail_sizing_area(
    L_fuselage, V_g, S_lat, X_cg, sideslip_angle, V, aspect_ratio
):
    return (
        (2 * X_cg - L_fuselage)
        * V_g**2
        * S_lat
        / (math.pi * sideslip_angle * V**2 * (L_fuselage - X_cg))
    ) / aspect_ratio


L_fuselage = 10  # m
V_g = 5  # m/s
drone_thickness = 0.4  # m
S_lat = drone_thickness * L_fuselage  # m^2
X_cg = 7  # m
sideslip_angle = 5 / 180 * math.pi  # rad
V = 15  # cruise speed
aspect_ratio = 3
v_taper_ratio = 

S_result = vertical_tail_sizing_area(
    L_fuselage, V_g, S_lat, X_cg, sideslip_angle, V, aspect_ratio
)
b = math.sqrt(S_result * aspect_ratio)
c = S_result / b

print("b=", b, "m", "\n", "c=", c, "m", "\n", "S=", S_result, "m^2")

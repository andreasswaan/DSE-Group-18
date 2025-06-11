import math
import matplotlib.pyplot as plt
import numpy as np


def vertical_tail_sizing(
    L_fuselage, V_g, X_cg, V, aspect_ratio, v_taper_ratio,drone_thickness
):
    S_lat = drone_thickness * L_fuselage
    S = (
        (2 * X_cg - L_fuselage)
        * V_g**2
        * S_lat
        / (math.pi * V**2 * (L_fuselage - X_cg))
    ) / aspect_ratio
    b = math.sqrt(S * aspect_ratio)
    c_small = 2 * v_taper_ratio / (1 + v_taper_ratio) * math.sqrt(S / aspect_ratio)
    c_big = 2 / (1 + v_taper_ratio) * math.sqrt(S / aspect_ratio)
    if b <= 0 or c_small <= 0 or c_big <= 0:
        b = 0
        c_small = 0
        c_big = 0
        print("error: vertical tail model failure")
    return c_small, c_big, b


L_fuselage = 10  # m
V_g = 5  # m/s
drone_thickness = 0.4  # m
X_cg = 7  # m
V = 15  # cruise speed
aspect_ratio = 5
v_taper_ratio = 0.5

c_1, c_2, b = vertical_tail_sizing(
    L_fuselage, V_g, X_cg, V, aspect_ratio, v_taper_ratio, drone_thickness
)

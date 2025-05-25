from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from mission import Mission
from prelim_des.drone import Drone
from prelim_des.performance import Performance


mission = Mission("DRCCRCCRCCD")
# print("Mission phases:", mission.phases_str)
# print("Mission phase objects:", mission.legs_dict)

drone = Drone()
perf = Performance(drone, mission)
drone.perf = perf
# drone.class_1_weight_estimate()
# print("Drone MTOW:", drone.MTOW)
# print("Drone OEW:", drone.OEW)
# drone.wing.S = perf.wing_area(drone.OEW)
# print("Wing area (S):", drone.wing.S)

# drone.class_2_weight_estimate()
# print("Drone MTOW after class 2 estimate:", drone.MTOW)
# print("Drone OEW after class 2 estimate:", drone.OEW)

drone.iterative_weight_estimate(plot=True, tolerance=0.0001)

# print("Wing parameters after iterative weight estimate:")
# print(f"Wing area (S): {drone.wing.S:.2f} m²")
# print(f"Aspect ratio (AR): {drone.wing.geom_AR:.2f}")
# print(f"Mean aerodynamic chord (MAC): {drone.wing.MAC:.2f} m")
# print(f"Root chord (c_root): {drone.wing.c_root:.2f} m")
# print(f"Wing span (b): {drone.wing.span:.2f} m")
# print(f"Taper ratio (TR): {drone.wing.taper:.2f}")
# print(f"xLemac: {drone.wing.xLEMAC:.2f} m")
# print(f"yLemac: {drone.wing.yLEMAC:.2f} m")
# print(f"Quarter chord sweep (Λ): {drone.wing.Λ_c4:.2f} degrees")



drone.wing.plot_planform(saveplot=True)


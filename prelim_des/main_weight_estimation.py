from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from mission import Mission
from prelim_des.drone import Drone
from prelim_des.performance import Performance
import logging
import utils.define_logging  # do not remove this line, it sets up logging configuration


logging.info("Starting drone performance estimation...")


# mission = Mission("DRCCRCCRCCD")
mission = Mission("DRCCRCD")
# print("Mission phases:", mission.phases_str)
# print("Mission phase objects:", mission.legs_dict)

drone = Drone()
perf = Performance(drone, mission)
drone.perf = perf

drone.class_1_weight_estimate(del_mech=False)
print("Drone MTOW:", drone.MTOW)
# print("Drone OEW:", drone.OEW)

drone.wing.S = perf.wing_area(drone.OEW)
# print("Wing area (S):", drone.wing.S)

drone.class_2_weight_estimate(transition=True)
# print("Drone MTOW after class 2 estimate:", drone.MTOW)
# print("Drone OEW after class 2 estimate:", drone.OEW)

drone.iterative_weight_estimate(
    transition=True, plot=False, max_iterations=100, tolerance=0.01
)

print("Drone MTOW after iterative estimate:", drone.MTOW)
print(
        "Drone mass breakdown:",
        "Battery weight:",
        drone.propulsion.battery.weight,
        "Wing weight:",
        drone.wing.weight,
        "Fuselage weight:",
        drone.fuselage.weight,
        "Tail weight:",
        drone.tail.weight,
        "Landing gear weight:",
        drone.landing_gear.weight,
        "Propulsion weight:",
        drone.propulsion.weight(drone.perf.mission_energy(transition=True)[0]),
        "Mission energy (J):",
        drone.perf.mission_energy(transition=True)[0], 
)
# print(f"Wing surface area: {drone.wing.S}")

# drone.wing.plot_planform(save_plot=True)
print(f"Aspect ratio: {drone.wing.geom_AR}")
# print(f"Mean aerodynamic chord (MAC): {drone.wing.MAC}")

# print(drone.wing.c_root, drone.wing.c_tip, drone.wing.S)
# print(drone.wing.span)
# print(drone.wing.x_ac_lemac)
# print(drone.wing.c_root, drone.wing.c_tip, drone.wing.S)
# print(drone.wing.span)
# print(drone.wing.x_ac_lemac)

# drone.perf.cruise_noise(plot=True)

drone.perf.payload_range_diagram()

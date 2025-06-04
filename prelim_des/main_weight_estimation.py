from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from mission import Mission
from prelim_des.drone import Drone
from prelim_des.performance import Performance
import logging
import utils.define_logging  # do not remove this line, it sets up logging configuration

logging.info("Starting drone performance estimation...")


mission = Mission("DRCCRCCRCCD")
# print("Mission phases:", mission.phases_str)
# print("Mission phase objects:", mission.legs_dict)

drone = Drone()
perf = Performance(drone, mission)
drone.perf = perf
drone.class_1_weight_estimate()
# print("Drone MTOW:", drone.MTOW)
# print("Drone OEW:", drone.OEW)
drone.wing.S = perf.wing_area(drone.OEW)
# print("Wing area (S):", drone.wing.S)

drone.class_2_weight_estimate()
# print("Drone MTOW after class 2 estimate:", drone.MTOW)
# print("Drone OEW after class 2 estimate:", drone.OEW)

drone.iterative_weight_estimate(plot=True, tolerance=0.01)
print("Drone MTOW after iterative estimate:", drone.MTOW)
print(f"Wing surface area: {drone.wing.S}")

drone.wing.plot_planform(save_plot=True)
print(f"Aspect ratio: {drone.wing.geom_AR}")

# drone.perf.cruise_noise(plot=True)

drone.perf.payload_range_diagram()

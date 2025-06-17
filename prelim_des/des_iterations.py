from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from prelim_des.mission import Mission
from prelim_des.drone import Drone
from prelim_des.performance import Performance


mission = Mission("DRCCRCD")

drone = Drone()
perf = Performance(drone, mission)
drone.perf = perf

drone.class_1_weight_estimate(del_mech=True)
print("Drone MTOW:", drone.MTOW)

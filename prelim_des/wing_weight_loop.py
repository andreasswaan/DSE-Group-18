from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from mission import Mission
from prelim_des.drone import Drone
from prelim_des.performance import Performance
import logging
import utils.define_logging  # do not remove this line, it sets up logging configuration
from prelim_des.idealized_structure import run_structure_analysis

mission = Mission("DRCCRCCRCCD")
# print("Mission phases:", mission.phases_str)
# print("Mission phase objects:", mission.legs_dict)

drone = Drone()
perf = Performance(drone, mission)
drone.perf = perf
drone.MTOW = 12

drone.perf.wing_area_iterations(22)

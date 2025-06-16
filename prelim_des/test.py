from prelim_des.drone import Drone
from prelim_des.mission import Mission
from prelim_des.performance import Performance
from prelim_des.idealized_structure import StructuralAnalysis
mission = Mission("DRCCRCCRCCD")
drone = Drone()
perf = Performance(drone, mission)
drone.perf = perf
drone.class_1_weight_estimate()

drone.wing.S = perf.wing_area(drone.OEW)
drone.class_2_weight_estimate()

structural_analysis = StructuralAnalysis(drone,1,"wing")

print(structural_analysis.run_wing_analysis())
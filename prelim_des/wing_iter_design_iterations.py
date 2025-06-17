from __future__ import annotations
import os
import datetime
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
# print("Drone MTOW:", drone.MTOW)

drone.wing.S = perf.wing_area(drone.OEW)
drone.class_2_weight_estimate(transition=True)

t0 = datetime.datetime.now()

mtow, oew, mtow_history, S_history, n_iterations, component_dict = drone.iterative_weight_estimate(
    transition=True, iter_wing_planform=True, plot=False, max_iterations=100, tolerance=0.001
)

t1 = datetime.datetime.now()
print(f"\nTime taken for iterative weight estimate: {t1 - t0}\n")

plot_path = os.path.join("prelim_des", "iterations", "plots")
os.makedirs(plot_path, exist_ok=True)
data_path = os.path.join("prelim_des", "iterations", "data")
os.makedirs(data_path, exist_ok=True)

plt.figure(figsize=(8, 6))
plt.plot(
    np.arange(1, len(mtow_history) + 1),
    mtow_history,
    marker="o",
    color="#1f77b4",
    label="MTOW Convergence",
)
plt.xlabel("Iteration", fontsize=16, labelpad=10)
plt.ylabel("Total Weight [kg]", fontsize=16, labelpad=10)
plt.tick_params(labelsize=14)
plt.ylim(np.min(mtow_history) * 0.9, np.max(mtow_history) * 1.1)
plt.legend(fontsize=14, loc="upper right")
plt.grid(True)
plt.tight_layout()
plt.savefig(
    os.path.join(plot_path, "iter_wing_mtow_convergence.pdf"),
    dpi=300,
    format="pdf",
    bbox_inches="tight",
)
plt.close()

# Save results in a dictionary and store as npz
results_dict = {
    "mtow_history": np.array(mtow_history),
    "S_history": np.array(S_history),
    "n_iterations": n_iterations,
    "final_mtow": mtow,
    "final_oew": oew,
    "component_weights": {
        "wing_weight_history": np.array(component_dict["wing_weight_history"]),
        "fuselage_weight_history": np.array(component_dict["fuselage_weight_history"]),
        "landing_gear_weight_history": np.array(component_dict["landing_gear_weight_history"]),
        "propulsion_weight_history": np.array(component_dict["propulsion_weight_history"]),
        "tail_weight_history": np.array(component_dict["tail_weight_history"]),
        "sensor_weight_history": np.array(component_dict["sensor_weight_history"]),
    },
}

np.savez(
    os.path.join(data_path, "iter_wing_convergence_results_insulated.npz"), **results_dict
)

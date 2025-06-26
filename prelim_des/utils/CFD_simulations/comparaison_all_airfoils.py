import os
import matplotlib.pyplot as plt
import numpy as np
from prelim_des.utils.CFD_simulations.ten_sweep import (
    Lift_10_sweep,
    Cl_10_sweep,
    Cd_10_sweep,
    Drag_10_sweep,
)
from prelim_des.utils.CFD_simulations.no_sweep import (
    Lift_no_sweep,
    Drag_no_sweep,
    Cl_no_sweep,
    Cd_no_sweep,
)
from prelim_des.utils.CFD_simulations.tc120 import (
    Lift_tc_120,
    Drag_tc_120,
    Cd_tc_120,
    Cl_tc_120,
)
from prelim_des.utils.CFD_simulations.tc320 import (
    Lift_tc_320,
    Drag_tc_320,
    Cl_tc_320,
    Cd_tc_320,
)
from prelim_des.utils.CFD_simulations.rc440 import (
    Lift_rc_440,
    Drag_rc_440,
    Cl_rc_440,
    Cd_rc_440,
)
from prelim_des.utils.CFD_simulations.rc640 import (
    Lift_rc_640,
    Drag_rc_640,
    Cd_rc_640,
    Cl_rc_640,
)
from prelim_des.utils.CFD_simulations.basic import (
    Lift_basic,
    Drag_basic,
    Cd_basic,
    Cl_basic,
    AOA,
)

show_plots = False
root = os.path.dirname(os.path.abspath(__file__))

# Lift plot
plt.figure(figsize=(10, 10))
plt.plot(AOA, Lift_10_sweep, label="10° Sweep", marker="P", c="c", markersize=12)
plt.plot(AOA, Lift_no_sweep, label="No Sweep", marker="v", c="c", markersize=12)
plt.plot(AOA, Lift_tc_320, label="TC:320 mm", marker="P", c="y", markersize=12)
plt.plot(AOA, Lift_tc_120, label="TC:120 mm", marker="v", c="y", markersize=12)
plt.plot(AOA, Lift_rc_640, label="RC:640 mm", marker="P", c="m", markersize=12)
plt.plot(AOA, Lift_rc_440, label="RC:440 mm", marker="v", c="m", markersize=12)
plt.plot(AOA, Lift_basic, label="Basic", marker="o", c="black")
plt.xlabel("Angle of Attack (α) [deg]", fontsize=18, labelpad=10)
plt.ylabel("Lift [N]", fontsize=18, labelpad=10)
plt.tick_params(labelsize=14)
plt.legend(fontsize=18)
plt.grid(True)
plt.tight_layout()
plt.savefig(
    os.path.join(root, "figs", "lift_comparaison.svg"),
    dpi=300,
    bbox_inches="tight",
    format="svg",
)
if show_plots:
    plt.show()

# Cl plot
plt.figure(figsize=(10, 10))
plt.plot(AOA, Cl_10_sweep, label="10° Sweep", marker="P", c="c", markersize=12)
plt.plot(AOA, Cl_no_sweep, label="No Sweep", marker="v", c="c", markersize=12)
plt.plot(AOA, Cl_tc_320, label="TC:320 mm", marker="P", c="y", markersize=12)
plt.plot(AOA, Cl_tc_120, label="TC:120 mm", marker="v", c="y", markersize=12)
plt.plot(AOA, Cl_rc_640, label="RC:640 mm", marker="P", c="m", markersize=12)
plt.plot(AOA, Cl_rc_440, label="RC:440 mm", marker="v", c="m", markersize=12)
plt.plot(AOA, Cl_basic, label="Basic", marker="o", c="black")
plt.xlabel("Angle of Attack (α) [deg]", fontsize=18, labelpad=10)
plt.ylabel("Cl [-]", fontsize=18, labelpad=10)
plt.tick_params(labelsize=14)
plt.legend(fontsize=18)
plt.grid(True)
plt.tight_layout()
plt.savefig(
    os.path.join(root, "figs", "Cl_comparaison.svg"),
    dpi=300,
    bbox_inches="tight",
    format="svg",
)
if show_plots:
    plt.show()

# Cd plot
plt.figure(figsize=(10, 10))
plt.plot(AOA, Cd_10_sweep, label="10° Sweep", marker="P", c="c", markersize=12)
plt.plot(AOA, Cd_no_sweep, label="No Sweep", marker="v", c="c", markersize=12)
plt.plot(AOA, Cd_tc_320, label="TC:320 mm", marker="P", c="y", markersize=12)
plt.plot(AOA, Cd_tc_120, label="TC:120 mm", marker="v", c="y", markersize=12)
plt.plot(AOA, Cd_rc_640, label="RC:640 mm", marker="P", c="m", markersize=12)
plt.plot(AOA, Cd_rc_440, label="RC:440 mm", marker="v", c="m", markersize=12)
plt.plot(AOA, Cd_basic, label="Basic", marker="o", c="black")
plt.xlabel("Angle of Attack (α) [deg]", fontsize=18, labelpad=10)
plt.ylabel("Cd [-]", fontsize=18, labelpad=10)
plt.tick_params(labelsize=14)
plt.legend(fontsize=18)
plt.grid(True)
plt.tight_layout()
plt.savefig(
    os.path.join(root, "figs", "Cd_comparaison.svg"),
    dpi=300,
    bbox_inches="tight",
    format="svg",
)
if show_plots:
    plt.show()

# Drag plot
plt.figure(figsize=(10, 10))
plt.plot(AOA, Drag_10_sweep, label="10° Sweep", marker="P", c="c", markersize=12)
plt.plot(AOA, Drag_no_sweep, label="No Sweep", marker="v", c="c", markersize=12)
plt.plot(AOA, Drag_tc_320, label="TC:320 mm", marker="P", c="y", markersize=12)
plt.plot(AOA, Drag_tc_120, label="TC:120 mm", marker="v", c="y", markersize=12)
plt.plot(AOA, Drag_rc_640, label="RC:640 mm", marker="P", c="m", markersize=12)
plt.plot(AOA, Drag_rc_440, label="RC:440 mm", marker="v", c="m", markersize=12)
plt.plot(AOA, Drag_basic, label="Basic", marker="o", c="black")
plt.xlabel("Angle of Attack (α) [deg]", fontsize=18, labelpad=10)
plt.ylabel("Drag [N]", fontsize=18, labelpad=10)
plt.tick_params(labelsize=16)
plt.legend(fontsize=18)
plt.grid(True)
plt.tight_layout()
plt.savefig(
    os.path.join(root, "figs", "Drag_comparaison.svg"),
    dpi=300,
    bbox_inches="tight",
    format="svg",
)
if show_plots:
    plt.show()

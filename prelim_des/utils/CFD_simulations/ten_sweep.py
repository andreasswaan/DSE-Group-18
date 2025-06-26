import numpy as np

Root_Chord = 0.54
Tip_Chord = 0.22
S = 1.156
LE_sweep = 10
B = 3
AR = 7.785467128
Speed = 15
rho = 1.225
e = 0.8

AOA = [0, 3, 6, 9, 12]

Lift_10_sweep = np.array([55.17, 94.646, 132.024, 159.955, 166.706])
Cl_10_sweep = Lift_10_sweep * 2 / (rho * Speed**2 * S)

Drag_10_sweep = np.array([10.905, 12.619, 15.927, 20.879, 28.829])
Cd_10_sweep = Drag_10_sweep * 2 / (rho * Speed**2 * S)


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    plt.plot(AOA, Lift_10_sweep, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Lift [N]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Cl_10_sweep, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Coefficient of Lift [-]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Drag_10_sweep, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Drag [N]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Cd_10_sweep, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Coefficient of Drag [-]", fontsize=12)
    plt.grid(True)
    plt.show()

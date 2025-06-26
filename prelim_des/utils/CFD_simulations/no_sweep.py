import numpy as np

Root_Chord = 0.54
Tip_Chord = 0.22
S = 1.156
LE_sweep = 0
B = 3
AR = 7.785467128
Speed = 15
rho = 1.225
e = 0.8

AOA = [0, 3, 6, 9, 12]

Lift_no_sweep = np.array([55.675, 93.836, 127.532, 149.981, 155.176])
Cl_no_sweep = Lift_no_sweep * 2 / (rho * Speed**2 * S)

Drag_no_sweep = np.array([11.275, 12.925, 16.757, 22.383, 28.843])
Cd_no_sweep = Drag_no_sweep * 2 / (rho * Speed**2 * S)


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    plt.plot(AOA, Lift_no_sweep, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Lift [N]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Cl_no_sweep, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Coefficient of Lift [-]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Drag_no_sweep, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Drag [N]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Cd_no_sweep, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Coefficient of Drag [-]", fontsize=12)
    plt.grid(True)
    plt.show()

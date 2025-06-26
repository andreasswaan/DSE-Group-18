import numpy as np

Root_Chord = 0.54
Tip_Chord = 0.32
S = 1.301
LE_sweep = 3.1
B = 3
AR = 7.785467128
Speed = 15
rho = 1.225
e = 0.8

AOA = [0, 3, 6, 9, 12]

Lift_tc_320 = np.array([62.337, 105.334, 145.492, 175.019, 177.656])
Cl_tc_320 = Lift_tc_320 * 2 / (rho * Speed**2 * S)

Drag_tc_320 = np.array([11.999, 13.97, 17.9, 24.076, 32.419])
Cd_tc_320 = Drag_tc_320 * 2 / (rho * Speed**2 * S)


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    plt.plot(AOA, Lift_tc_320, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Lift [N]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Cl_tc_320, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Coefficient of Lift [-]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Drag_tc_320, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Drag [N]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Cd_tc_320, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Coefficient of Drag [-]", fontsize=12)
    plt.grid(True)
    plt.show()

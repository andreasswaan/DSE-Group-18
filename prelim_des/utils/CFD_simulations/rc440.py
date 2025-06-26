import numpy as np

Root_Chord = 0.44
Tip_Chord = 0.22
S = 1.001

LE_sweep = 3.1
B = 3
AR = 7.785467128
Speed = 15
rho = 1.225
e = 0.8

AOA = [0, 3, 6, 9, 12]

Lift_rc_440 = np.array([49.979, 82.632, 112.625, 128.366, 127.936])
Cl_rc_440 = Lift_rc_440 * 2 / (rho * Speed**2 * S)

Drag_rc_440 = np.array([11.031, 12.752, 15.707, 21.02, 27.667])
Cd_rc_440 = Drag_rc_440 * 2 / (rho * Speed**2 * S)


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    plt.plot(AOA, Lift_rc_440, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Lift [N]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Cl_rc_440, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Coefficient of Lift [-]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Drag_rc_440, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Drag [N]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Cd_rc_440, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Coefficient of Drag [-]", fontsize=12)
    plt.grid(True)
    plt.show()

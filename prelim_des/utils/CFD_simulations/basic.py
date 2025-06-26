import numpy as np

Root_Chord = 0.54
Tip_Chord = 0.22
S = 1.156
LE_sweep = 3.1
B = 3
AR = 7.785467128
Speed = 15
rho = 1.225
e = 0.8

AOA = [0, 3, 6, 9, 12]

Lift_basic = np.array([56.323, 94.038, 129.646, 151.061, 155.983])
Cl_basic = Lift_basic * 2 / (rho * Speed**2 * S)

Drag_basic = np.array([11.288, 13.027, 16.48, 21.777, 29.006])
Cd_basic = Drag_basic * 2 / (rho * Speed**2 * S)


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    plt.plot(AOA, Lift_basic, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Lift [N]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Cl_basic, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Coefficient of Lift [-]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Drag_basic, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Drag [N]", fontsize=12)
    plt.grid(True)
    plt.show()

    plt.plot(AOA, Cd_basic, marker="o")
    plt.xlabel("Angle of Attack (α) [deg]", fontsize=12)
    plt.ylabel("Coefficient of Drag [-]", fontsize=12)
    plt.grid(True)
    plt.show()

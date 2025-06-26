import numpy as np
import matplotlib.pyplot as plt

thicknesses = [0.002, 0.003, 0.005]
principal_stress = [13.2, 9.486, 4.481292517]

conv_t = [1, 0.8, 0.6, 0.4]
conv_shear = [7.2, 8.6, 9.1, 9.486]
# Plot original data points

if False:
    plt.plot(thicknesses, principal_stress, label="Shear Stress", marker="o")
    plt.xlabel("Thickness [m]", fontsize=14, labelpad=5)
    plt.ylabel("Shear Stress [Pa]", fontsize=14, labelpad=5)
    plt.tick_params(labelsize=12)
    plt.legend()
    plt.grid(True)
    plt.show()

if True:
    plt.plot(conv_t, conv_shear, label="Max Shear Stress", marker="o")
    plt.xlabel("Mesh element size [mm]", fontsize=14, labelpad=5)
    plt.ylabel("Shear Stress [Pa]", fontsize=14, labelpad=5)
    plt.tick_params(labelsize=12)
    plt.ylim(bottom=0, top=11)
    plt.legend()
    plt.grid(True)
    plt.show()

t = 0.002


def calculate_stress_ansys(t: float):
    if t <= 0.003:
        thicknesses_1 = thicknesses[0:2]
        principal_stress_1 = principal_stress[0:2]

        # Perform linear regression: y = m*x + c
        coeffs = np.polyfit(thicknesses_1, principal_stress_1, 1)
        m, c = coeffs
        return t * m + c
    else:
        thicknesses_2 = thicknesses[1::]
        principal_stress_2 = principal_stress[1::]
        # Perform linear regression: y = m*x + c
        coeffs = np.polyfit(thicknesses_2, principal_stress_2, 1)
        m, c = coeffs
        return t * m + c


print(calculate_stress_ansys(t))
print(calculate_stress_ansys(t + 0.003))

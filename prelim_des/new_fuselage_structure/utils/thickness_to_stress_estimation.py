import numpy as np

thicknesses = [0.002, 0.003, 0.005]
principal_stress = [5.516919218, 9.486, 4.481292517]

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

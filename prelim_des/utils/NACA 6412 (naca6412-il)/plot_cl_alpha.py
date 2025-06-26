import os
import numpy as np
import pandas as pd

root = os.path.dirname(os.path.abspath(__file__))
airfoil_data_path = root + "\\xf-naca6412-il-1000000.csv"
with open(airfoil_data_path, "r") as f:
    lines = f.readlines()
# Find the line where the actual data starts (header: 'Alpha,Cl,Cd,Cdp,Cm,Top_Xtr,Bot_Xtr')
for idx, line in enumerate(lines):
    if line.strip().startswith("Alpha,Cl"):
        data_start = idx
        break

airfoil_DF = pd.read_csv(airfoil_data_path, skiprows=data_start)
print(airfoil_DF)

import matplotlib.pyplot as plt

plt.plot(airfoil_DF["Alpha"], airfoil_DF["Cl"])
plt.xlabel("Angle Of Attack (α) [deg]", fontsize=12)
plt.ylabel("Coefficient of lift (Cl) [-]", fontsize=12)
plt.grid(True)
plt.show()

import os
import numpy as np
import pandas as pd

root = os.path.dirname(os.path.abspath(__file__))
airfoil_path = "\\NACA airfoil database"
NACA_type = "\\NACA 4 digit airfoils"

minimum_cl = 1.5

possible_airfoils = []


def make_airfoil_selection(airfoil):
    print(f"Folder: {airfoil}")
    airfoil_folder_path = root + airfoil_path + NACA_type + "\\" + airfoil
    print(f"Processing airfoil: {airfoil_folder_path}")
    airfoil_data_path = airfoil_folder_path + "\\" + os.listdir(airfoil_folder_path)[1]
    print(airfoil_data_path)
    # Skip the header lines and read only the data table
    with open(airfoil_data_path, "r") as f:
        lines = f.readlines()
    # Find the line where the actual data starts (header: 'Alpha,Cl,Cd,Cdp,Cm,Top_Xtr,Bot_Xtr')
    for idx, line in enumerate(lines):
        if line.strip().startswith("Alpha,Cl"):
            data_start = idx
            break

    airfoil_DF = pd.read_csv(airfoil_data_path, skiprows=data_start)
    print(airfoil_DF)
    if airfoil_DF["Cl"].max() > minimum_cl:
        print(f"Airfoil {airfoil} meets the minimum Cl requirement of {minimum_cl}.")
        possible_airfoils.append(airfoil)


for folder in os.listdir(root + airfoil_path + NACA_type):

    if folder.startswith("Symmetrical airfoils"):
        for folder1 in os.listdir(
            root + airfoil_path + NACA_type + "\\Symmetrical airfoils"
        ):
            make_airfoil_selection(folder + "\\" + folder1)

    else:
        make_airfoil_selection(folder)

print("Possible airfoils that meet the criteria:")
for airfoil in possible_airfoils:
    print(airfoil)

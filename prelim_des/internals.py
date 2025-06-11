# Dictionaries for all internal components. Mass [kg] + (x,y,z) coordinates [mm] of individual CGs.
# Constant components are part of the OEW
# Variable components may or may not be present at a given moment (i.e. pizzas and batteries)
# Variable components can be triggered to be "active" or "inactive" in the simulation, affecting the CG.
# Structural elements are retreived from idealized_structure
# TODO: finish this
layout_const = {
    # PAYLOAD
    "insulation": {"m": 0.500, "x": 750.0, "y": 0.0, "z": 170.0},
    # ELEC
    "cpu": {"m": 0.070, "x": 95.0, "y": 0.0, "z": 0.0},  # TODO: placeholder mass
    # SENSOR TODO: placebolder masses
    "cam1": {"m": 0.050, "x": 55.0, "y": 0.0, "z": 0.0},  # forward
    "cam2": {"m": 0.050, "x": 1300.0, "y": 0.0, "z": 0.0},  # backward
    "cam3": {"m": 0.050, "x": 100.0, "y": 230.0, "z": 0.0},  # left
    "cam4": {"m": 0.050, "x": 100.0, "y": -230.0, "z": 0.0},  # right
    "cam5": {"m": 0.050, "x": 115.0, "y": 0.0, "z": 0.0},  # up
    "cam6": {"m": 0.050, "x": 100.0, "y": 0.0, "z": 0.0},  # down
    "gnss": {"m": 0.300, "x": 200.0, "y": 0.0, "z": 300.0},
    # TODO: payload sensors
}
layout_var = {
    # PIZZA
    "pizzaf1": {"m": 0.5, "x": 440.0, "y": 0.0, "z": 70.0, "active": 1},
    "pizzaf2": {"m": 0.5, "x": 440.0, "y": 0.0, "z": 140.0, "active": 0},
    "pizzab1": {"m": 0.5, "x": 960.0, "y": 0.0, "z": 70.0, "active": 0},
    "pizzab2": {"m": 0.5, "x": 960.0, "y": 0.0, "z": 140.0, "active": 0},
    "pizzab3": {"m": 0.5, "x": 960.0, "y": 0.0, "z": 210.0, "active": 0},
    # POWER
    "bat1": {"m": 1.974, "x": 245.0, "y": 50.0, "z": 250.0, "active": 1},
    "bat2": {"m": 1.974, "x": 245.0, "y": -50.0, "z": 250.0, "active": 1},
    "bat3": {"m": 1.974, "x": 1300.0, "y": 75.0, "z": 130.0, "active": 1},
    "bat4": {"m": 1.974, "x": 1300.0, "y": -75.0, "z": 130.0, "active": 0},
}
layout_struc = {  # TODO: get data from structures
    "fuselage": {"m": 1.0, "x": 750.0, "y": 0.0, "z": 150.0},
    "wing": {"m": 1.0, "x": 750.0, "y": 290.0, "z": 0.0},
    "tail": {"m": 1.0, "x": 1200.0, "y": 0.0, "z": 350.0},
}


def get_cg_internals(comps: dict) -> dict:
    """
    Get the center of gravity (CG) of all internal components.

    Args:
        comps (dict): Dictionary containing the components and their masses and CG positions.

    Returns:
        dict: Dictionary with the total mass and CG.
    """
    total_m = 0.0
    total_mx = 0.0
    total_my = 0.0
    total_mz = 0.0

    for comp in comps.values():
        if ("active" in comp.keys() and comp["active"]) or "active" not in comp.keys():
            total_m += comp["m"]
            total_mx += comp["m"] * comp["x"]
            total_my += comp["m"] * comp["y"]
            total_mz += comp["m"] * comp["z"]

    cg_internal = {
        "m": total_m,
        "x": total_mx / total_m if total_m != 0 else 0.0,
        "y": total_my / total_m if total_m != 0 else 0.0,
        "z": total_mz / total_m if total_m != 0 else 0.0,
    }

    return cg_internal


cg_internals_struc = get_cg_internals(layout_struc)
cg_internals_const = get_cg_internals(layout_const)
print(cg_internals_const)
# Don't forget to change which variable items are "active" before getting its CG.
cg_internals_var = get_cg_internals(layout_var)
print(cg_internals_var)


if __name__ == "__main__":
    cg_internals_const = get_cg_internals(layout_const)
    # print(f"Components list: {layout_const}")
    # print("Center of Gravity for Internals:")
    # print(f"Mass: {cg_internals_const['m']:.2f} kg")
    # print(f"CG Position: ({cg_internals_const['x']:.2f}, {cg_internals_const['y']:.2f}, {cg_internals_const['z']:.2f}) mm")

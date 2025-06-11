# Dictionaries for all internal components. Mass [kg] + (x,y,z) coordinates [mm] of individual CGs.
# Constant components are part of the OEW
# Variable components may or may not be present at a given moment (i.e. pizzas and batteries)
# TODO: finish this
layout_const = {
    # PAYLOAD
    "pizza_holder1": {"m": 1.5, "x": 0.0, "y": 0.0, "z": 0.0},
    # ELEC
    "cpu1": {"m": 1.5, "x": 0.0, "y": 0.0, "z": 0.0},
    # SENSOR
}
layout_var = {
    # PIZZA
    "stack1": {"m": 1.5, "x": 0.0, "y": 0.0, "z": 0.0},
    "stack2": {"m": 1.5, "x": 0.0, "y": 0.0, "z": 0.0},
    # POWER
    "bat1": {"m": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
    "bat2": {"m": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
    "bat3": {"m": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
    "bat4": {"m": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
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


if __name__ == "__main__":
    cg_internals_const = get_cg_internals(layout_const)
    print(f"Components list: {layout_const}")
    print("Center of Gravity for Internals:")
    print(f"Mass: {cg_internals_const['m']:.2f} kg")
    print(
        f"CG Position: ({cg_internals_const['x']:.2f}, {cg_internals_const['y']:.2f}, {cg_internals_const['z']:.2f}) mm"
    )

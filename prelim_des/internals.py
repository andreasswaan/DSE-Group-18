from typing import TYPE_CHECKING
from CG_calculation_structures import (
    compute_fuselage_cg_xz,
    compute_wing_cg_xz,
    compute_tail_cg_xz,
)
import CG_calculation_structures

if TYPE_CHECKING:
    from idealized_structure import FuselageStructure, WingStructure, TailStructure


def get_cg(comps: dict) -> dict:
    """
    Get the center of gravity (CG) of a dictionary of components.

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

    cg_dict = {
        "m": total_m,
        "x": total_mx / total_m if total_m != 0 else 0.0,
        "y": total_my / total_m if total_m != 0 else 0.0,
        "z": total_mz / total_m if total_m != 0 else 0.0,
    }

    return cg_dict


def get_cg_groups(
    comps_const: dict,
    fuselage: FuselageStructure,
    wing: WingStructure,
    tail: TailStructure,
) -> dict:
    """
    Get the center of gravity (CG) of all constant components.

    Args:
        comps_const (dict): Dictionary containing the constant components and their masses and CG positions.
        fuselage (FuselageStructure): Fuselage structure.
        wing (WingStructure): Wing structure.
        tail (TailStructure): Tail structure.

    Returns:
        dict: Dictionary with the total mass and CG of constant components.
    """

    # Define structural components
    fuse_cg_x, fuse_cg_z = compute_fuselage_cg_xz(fuselage)
    wing_cg_x, wing_cg_z = compute_wing_cg_xz(wing)
    tail_cg_x, tail_cg_z = compute_tail_cg_xz(tail)
    h_fuse = fuselage.fuselage_height

    layout_struc = {  # TODO get masses
        "fuselage": {
            "m": 1.0,
            "x": fuse_cg_x,
            "y": 0.0,
            "z": fuse_cg_z + h_fuse / 2,
        },
        "wing": {"m": 1.0, "x": wing_cg_x, "y": 0.0, "z": wing_cg_z + h_fuse / 2},
        "tail": {"m": 1.0, "x": tail_cg_x, "y": 0.0, "z": tail_cg_z + h_fuse / 2},
    }

    # fuselage, with constant components, and tail
    fuselage_total = {
        **layout_struc["fuselage"],
        **layout_struc["tail"],
        **comps_const,
    }

    cg_groups = {
        "fuselage": get_cg(fuselage_total),
        "wing": layout_struc["wing"],
    }

    return cg_groups


# Dictionary for all internal variable components. Mass [kg] + (x,y,z) coordinates [mm] of individual CGs.
# Variable components may or may not be present at a given moment (i.e. pizzas and batteries)
layout_var = {
    # PIZZA
    "pizzaf1": {"m": 0.5, "x": 440.0, "y": 0.0, "z": 70.0},
    "pizzaf2": {"m": 0.5, "x": 440.0, "y": 0.0, "z": 140.0},
    "pizzaf3": {"m": 0.5, "x": 440.0, "y": 0.0, "z": 210.0},
    "pizzab1": {"m": 0.5, "x": 960.0, "y": 0.0, "z": 70.0},
    "pizzab2": {"m": 0.5, "x": 960.0, "y": 0.0, "z": 140.0},
    "pizzab3": {"m": 0.5, "x": 960.0, "y": 0.0, "z": 210.0},
    # POWER
    "bat1": {"m": 1.974, "x": 245.0, "y": 50.0, "z": 250.0},
    "bat2": {"m": 1.974, "x": 245.0, "y": -50.0, "z": 250.0},
    "bat3": {"m": 1.974, "x": 1300.0, "y": 75.0, "z": 130.0},
    "bat4": {"m": 1.974, "x": 1300.0, "y": -75.0, "z": 130.0},
}

# Define other internal components that are constant
layout_const = {
    # PAYLOAD
    "insulation": {"m": 0.500, "x": 750.0, "y": 0.0, "z": 170.0},
    # ELEC
    "cpu": {"m": 0.077, "x": 95.0, "y": 0.0, "z": 0.0},
    # SENSOR TODO: placeholder masses
    "cam1": {"m": 0.050, "x": 55.0, "y": 0.0, "z": 0.0},  # forward
    "cam2": {"m": 0.050, "x": 1300.0, "y": 0.0, "z": 0.0},  # backward
    "cam3": {"m": 0.050, "x": 100.0, "y": 230.0, "z": 0.0},  # left
    "cam4": {"m": 0.050, "x": 100.0, "y": -230.0, "z": 0.0},  # right
    "cam5": {"m": 0.050, "x": 115.0, "y": 0.0, "z": 0.0},  # up
    "cam6": {"m": 0.050, "x": 100.0, "y": 0.0, "z": 0.0},  # down
    "gnss": {"m": 0.030, "x": 200.0, "y": 0.0, "z": 300.0},
    # TODO: payload sensors
    # POWER TODO add motors
}

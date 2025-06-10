from __future__ import annotations
from typing import TYPE_CHECKING
import numpy as np

if TYPE_CHECKING:
    from prelim_des.drone import Drone
from constants import ρ
from prelim_des.utils.import_toml import load_toml
import utils.define_logging  # do not remove this line, it sets up logging configuration
import logging
from utils.CFD_simulation_results import (
    average_cl_alpha,
    AOA,
    tip_chord_cl_alpha,
    root_chord_cl_alpha,
)


toml = load_toml()


class Aerodynamics:

    S: float

    def __init__(
        self,
        drone: Drone,
    ):
        """
        Initialize the Aerodynamics class with default parameters.

        Parameters:
        drone (Drone): The drone object to which this aerodynamics model belongs.
        """
        logging.debug("Initializing Aerodynamics class...")

        self.CL_max = toml["config"]["aero"]["CL_max"]
        self.CL_cruise = toml["config"]["aero"]["CL_cruise"]
        self.CD_0 = toml["config"]["aero"]["CD0"]
        self.CD_flat_plate = toml["config"]["aero"]["CD_flat_plate"]
        self.oswald_efficiency = toml["config"]["aero"]["oswald_efficiency"]
        self.drone = drone
        self.CL_slope_aircraft = toml["config"]["aero"]["CL_slope_aircraft"]

    @property
    def CD(self) -> float:
        return self.CD_0 + (
            self.CL_cruise**2
            / (np.pi * self.oswald_efficiency * self.drone.wing.geom_AR)
        )

    def base_cl_cruise(self, alpha):
        """
        Estimate the cl of the drone based on alpha (deg).

        Parameters:
        alpha (float): The angle of attack (deg).

        Returns:
        float: The cl.
        """
        if alpha > 12:
            raise ValueError("the angle of attack is to large", alpha)
        if alpha < 0:
            raise ValueError("the angle of attack is to little", alpha)

        avg_cl_alpha = average_cl_alpha()
        cl_at_req_alpha = np.interp(alpha, AOA, avg_cl_alpha)
        return cl_at_req_alpha

    @property
    def cl_alpha(self):
        root_chord = self.drone.wing.c_root
        tip_chord = self.drone.wing.c_tip

        cl_alpha_tip = tip_chord_cl_alpha(tip_chord)
        cl_alpha_root = root_chord_cl_alpha(root_chord)
        cl_alpha = []
        for i in range(len(AOA)):
            cl_alpha.append(np.average([cl_alpha_tip[i], cl_alpha_root[i]]))

        return cl_alpha

    def lift(self, V: float, CL) -> float:
        """
        Estimate the lift force on the drone at a given velocity.

        Parameters:
        velocity (float): The velocity of the drone in m/s.

        Returns:
        float: The lift force in Newtons.
        """
        # Placeholder: replace with real aerodynamic model
        return 0.5 * ρ * V**2 * self.drone.wing.S * CL

    def drag(self, V: float, TO_or_LD: bool = False) -> float:
        """
        Estimate the drag force on the drone at a given velocity.

        Parameters:
        velocity (float): The velocity of the drone in m/s.
        flat_plate (bool): If True, use the flat plate drag coefficient, otherwise use the standard drag coefficient.

        Returns:
        float: The drag force in Newtons.
        """
        if TO_or_LD:
            return (
                0.5
                * ρ
                * V**2
                * (self.drone.wing.S + self.drone.structure.top_view_area)
                * self.CD_flat_plate
            )  # Drag force in N
        else:
            return 0.5 * ρ * V**2 * self.drone.wing.S * self.CD  # Drag force in N

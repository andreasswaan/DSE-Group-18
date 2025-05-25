from __future__ import annotations
from typing import TYPE_CHECKING
import numpy as np
if TYPE_CHECKING:
    from prelim_des.drone import Drone
from constants import ρ
from prelim_des.utils.import_toml import load_toml

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
        self.CL_max = toml["config"]["aero"]["CL_max"]
        self.CL_cruise = toml["config"]["aero"]["CL_cruise"]
        self.CD_0 = toml["config"]["aero"]["CD0"]
        self.CD_flat_plate = toml["config"]["aero"]["CD_flat_plate"]
        self.oswald_efficiency = toml["config"]["aero"]["oswald_efficiency"]
        self.drone = drone

    @property
    def CD(self) -> float:
        return (
            self.CD_0
            + (self.CL_cruise**2 / (np.pi * self.oswald_efficiency * self.drone.wing.geom_AR))
    
        )

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
            return 0.5 * ρ * V**2 * (self.drone.wing.S + self.drone.structure.top_view_area) * self.CD_flat_plate  # Drag force in N
        else:
            return 0.5 * ρ * V**2 * self.drone.wing.S * self.CD # Drag force in N

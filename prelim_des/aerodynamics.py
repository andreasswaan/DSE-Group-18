import numpy as np
from drone import Drone
from constants import ρ


class Aerodynamics:
    def __init__(
        self,
        CL_max: float = 2.0,
        CL_cruise: float = 1.5,
        CD_0: float = 0.05,
        oswald_efficiency: float = 0.8,
        S: float = 0.5,
    ):
        """
        Initialize the Aerodynamics class with default parameters.

        Parameters:
        CL_max (float): Maximum lift coefficient.
        CL_cruise (float): Lift coefficient at cruise.
        C_D0 (float): Zero-lift drag coefficient.
        oswald_efficiency (float): Oswald efficiency factor.
        S (float): Wing surface area in m^2.
        """
        self.CL_max = CL_max
        self.CL_cruise = CL_cruise
        self.CD_0 = CD_0
        self.oswald_efficiency = oswald_efficiency
        self.S = S
        self.drone = Drone()

    @property
    def CD(self) -> float:
        return (
            self.CD_0
            + (self.CL_max / (np.pi * self.oswald_efficiency * self.drone.wing.geom_AR))
            ** 2
        )

    def estimate_drag(self, speed: float) -> float:
        """
        Estimate the drag force on the drone at a given speed.

        Parameters:
        speed (float): The speed of the drone in m/s.

        Returns:
        float: The drag force in Newtons.
        """
        return 0.5 * ρ * speed**2 * self.S * self.CD  # Drag force in N

    def estimate_lift(self, speed: float, CL) -> float:
        """
        Estimate the lift force on the drone at a given speed.

        Parameters:
        speed (float): The speed of the drone in m/s.

        Returns:
        float: The lift force in Newtons.
        """
        # Placeholder: replace with real aerodynamic model
        return 0.5 * ρ * speed**2 * self.S * CL

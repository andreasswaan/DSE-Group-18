import numpy as np
from constants import ρ, g
from prelim_des.elems import Wing

class Performance:
    def __init__(self, L_over_D_cruise=10.0, T_over_D_cruise=1.0, T_over_W_takeoff=2.0):
        """
        Initialize the Performance class with default parameters.	
        L_over_D_cruise (float): Lift-to-drag ratio at cruise.
        T_over_D_cruise (float): Thrust-to-drag ratio at cruise.
        T_over_W_takeoff (float): Thrust-to-weight ratio at takeoff.
        """	
        self.L_over_D_cruise = L_over_D_cruise
        self.T_over_D_cruise = T_over_D_cruise
        self.T_over_W_takeoff = T_over_W_takeoff


    def calculate_wing_surface_area(self, weight, V_cruise):
        """Calculate required wing surface area for given weight and cruise speed."""
        S = weight / (0.5 * ρ * V_cruise**2 * Wing.CL_cruise)
        return S

    def calculate_thrust_cruise(self, D):
        """Calculate thrust required during cruise."""
        T = D * self.T_over_D_cruise  # Assuming level flight, thrust equals drag
        return T

    def calculate_power_cruise(self, T, V_cruise):
        """Calculate power required during cruise."""
        P_cruise = T * V_cruise  # Power = Thrust * Velocity
        return P_cruise

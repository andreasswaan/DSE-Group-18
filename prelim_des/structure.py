from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from prelim_des.drone import Drone
import numpy as np
from prelim_des.constants import g
from prelim_des.utils.import_toml import load_toml

toml = load_toml()
class Structure:
    def __init__(self, drone: Drone):
        self.drone = drone

    def calc_n_max(self):
        """
        Calculate the maximum load factor (n_max) for the drone.
        This is a placeholder method and should be replaced with a real calculation.
        """
        # Placeholder: replace with real structural analysis
        return 3.2 # Based on https://research.tudelft.nl/files/144857482/6.2022_1485.pdf
    def calc_n_ult(self):
        """
        Calculate the ultimate load factor (n_ult) for the drone.
        This is a placeholder method and should be replaced with a real calculation.
        """
        # Placeholder: replace with real structural analysis
        return 1.5 * self.calc_n_max()
    def fuselage_length(self):
        """
        Calculate the fuselage length.
        This is a placeholder method and should be replaced with a real calculation.
        """
        # Placeholder: replace with real fuselage length calculation
        payload_length = toml['config']['payload']['box_length'] * 2.1     # Assuming there are two pizzas stacked next to each other
        l_fus = max(payload_length, self.drone.wing.c_root)
        return l_fus
    def fuselage_diameter(self):
        """
        Calculate the fuselage diameter.
        This is a placeholder method and should be replaced with a real calculation.
        """
        # Placeholder: replace with real fuselage diameter calculation
        payload_width = max(toml['config']['payload']['box_width'] * 1.1, 
                            toml['config']['payload']['box_height'] 
                            * toml['config']['payload']['n_box']
                            * 1.1)  # Assuming some clearance around the payload
        return payload_width
    def fuselage_max_perimeter(self):
        """
        Calculate the maximum fuselage perimeter.
        This is a placeholder method and should be replaced with a real calculation.
        """
        # Placeholder: replace with real fuselage perimeter calculation
        return np.pi * self.fuselage_diameter()

    @property
    def top_view_area(self):
        """
        Fuselage top view area [m2]
        Placeholder method to calculate the top view area of the fuselage.
        """
        return self.fuselage_diameter() * self.fuselage_length()  # Assuming a cylindrical fuselage for simplicity
    
    def delivery_mechanism_weight(self):
        """
        Calculate the weight of the delivery mechanism.
        This is a placeholder method and should be replaced with a real calculation.
        """
        # Placeholder: replace with real delivery mechanism weight calculation
        return toml['config']['payload']['del_mech_weight']  # kg

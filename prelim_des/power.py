import numpy as np
from constants import ρ, g
from prelim_des.utils.unit_converter import ImperialConverter

class PropulsionSystem:
    def __init__(self):
        self.battery = Battery()
        self.motor = Motor()
        self.propeller = VertPropeller()

    def size(self, energy_required, peak_power):
        self.battery.size(energy_required)
        self.motor.size(peak_power)
        self.propeller.size(peak_power)

    def estimate_weight(self):
        return (
            self.battery.estimate_weight() +
            self.motor.estimate_weight() +
            self.propeller.estimate_weight()
        )

class Battery:
    def __init__(self, energy_density: float = 250, min_battery: float = 0.2):  # Wh/kg
        self.energy_required = None
        self.energy_density = energy_density
        self.min_battery = min_battery

    def size(self, energy_required):
        self.energy_required = energy_required

    def battery_weight(self):
        return self.energy_required / self.energy_density
    
class Motor:
    def __init__(self, specific_power: float, peak_power: float, η_elec: float = 0.95):  # kW/kg
        self.peak_power = peak_power
        self.specific_power = specific_power
        self.η_elec = η_elec

    def size(self, peak_power):
        self.peak_power = peak_power

    def estimate_weight(self):
        return self.peak_power / self.specific_power
    
    
class VertPropeller:
    def __init__(self, diameter: float = ImperialConverter.len_in_m(12), n_blades: int = 8, η_prop: float = 0.85):
        """
        Initialize the VertPropeller class with default parameters.
        Parameters:
        diameter (float): Diameter of the propeller in meters.
        n_blades (int): Number of blades.
        η_prop (float): Propeller efficiency.
        """
        self.η_prop = η_prop
        self.diameter = diameter
        self.n_blades = n_blades
    
    @property
    def area(self):
        return np.pi * (self.diameter / 2) ** 2 * self.n_blades
    
    def power(self, thrust: float) -> float:
        """
        Calculate the power required for the propeller.
        Parameters:
        thrust (float): Thrust in Newtons.
        Returns:
        float: Power in Watts.
        """
        return thrust ** 1.5 / np.sqrt(2 * ρ * self.area * self.η_prop)
    

class HorPropeller(VertPropeller):
    def __init__(self, diameter: float = ImperialConverter.len_in_m(18), n_blades: int = 2):
        super().__init__(diameter=diameter, n_blades=n_blades)


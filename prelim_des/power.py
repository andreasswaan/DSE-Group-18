import numpy as np
from constants import ρ, g
from prelim_des.utils.unit_converter import ImperialConverter
from prelim_des.drone import Drone
from prelim_des.utils.import_toml import load_toml
from prelim_des.constants import g
from prelim_des.power import Battery, Motor, HorPropeller, VertPropeller

toml = load_toml()

class PropulsionSystem:
    def __init__(self, drone: Drone, battery: Battery, motor: Motor, hor_prop: HorPropeller, ver_prop: VertPropeller):
        self.drone = drone
        self.battery = battery
        self.motor = motor
        self.hor_prop = hor_prop
        self.ver_prop = ver_prop

    def estimate_weight(self, energy_required: float = None) -> float:
        return (
            self.battery.weight(energy_required)
            + self.motor.weight()
            + self.ver_prop.weight()
            + self.hor_prop.weight()
        )

class Battery:
    def __init__(self):
        self.min_batt_lvl = toml["config"]["battery"]["min_batt_lvl"]
        self.batt_energy_ratio = toml["config"]["battery"]["batt_energy_ratio"]
        self.energy_density = toml["config"]["battery"]["energy_density"]  # Wh/kg

    def weight(self, energy_required: float) -> float:
        return energy_required / (self.energy_density * self.batt_energy_ratio)
    
class Motor:
    def __init__(self, specific_power: float, peak_power: float, η_elec: float = 0.95):  # kW/kg
        self.peak_power = peak_power
        self.specific_power = specific_power
        self.η_elec = η_elec

    def n_motors(self, power: float) -> int:
        """
        Placeholder method to calculate the number of motors required based on power.
        Parameters:
        power (float): Power in Watts.
        Returns:
        int: Number of motors required.
        """
        if power <= 0:
            raise ValueError("Power must be greater than zero.")
        return 8
    
    def weight(self):
        """
        Placeholder method to estimate motor weight based on specific power.
        Returns:
        float: Estimated weight of the motor in kg.
        """
        return 0.16 * self.n_motors() # Placeholder value, should be replaced with a real calculation
    
    
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
    
    def n_vert_prop(self) -> int:
        """
        Placeholder method to calculate the number of vertical propellers required.
        Returns:
        int: Number of vertical propellers.
        """
        return 8
    
    def weight(self) -> float:
        """
        Placeholder method to estimate the weight of the vertical propeller.
        Returns:
        float: Estimated weight in kg.
        """
        return 0.012 * self.n_vert_prop()
    

class HorPropeller(VertPropeller):
    def __init__(self, diameter: float = ImperialConverter.len_in_m(18), n_blades: int = 2):
        super().__init__(diameter=diameter, n_blades=n_blades)
    
    def n_hor_prop(self) -> int:
        """
        Placeholder method to calculate the number of horizontal propellers required.
        Returns:
        int: Number of horizontal propellers.
        """
        return 2
    
    def weight(self) -> float:
        """
        Placeholder method to estimate the weight of the horizontal propeller.
        Returns:
        float: Estimated weight in kg.
        """
        return 0.012 * self.n_hor_prop()
    


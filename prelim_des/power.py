from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from prelim_des.drone import Drone
import numpy as np
from constants import ρ, g
from prelim_des.utils.unit_converter import ImperialConverter, TimeConverter
from prelim_des.utils.import_toml import load_toml
from prelim_des.constants import g
import utils.define_logging  # do not remove this line, it sets up logging configuration
import logging


toml = load_toml()


class PropulsionSystem:
    def __init__(self, drone: Drone):
        logging.debug("Initializing PropulsionSystem class...")

        self.drone = drone
        self.battery = Battery()
        self.motor = Motor()
        self.hor_prop = HorPropeller()
        self.ver_prop = VertPropeller()
        self.η_elec = toml["config"]["motor"]["eff_elec"]
        self.η_prop = toml["config"]["propeller"]["eff_prop"]

    def weight(self, energy_required: float = None) -> float:
        return (
            self.battery.calc_weight(energy_required)
            + self.motor.weight()
            + self.ver_prop.weight()
            + self.hor_prop.weight()
        )


class Battery:
    weight: float  # Placeholder for battery weight
    def __init__(self):
        logging.debug("Initializing Battery class...")

        self.min_batt_lvl = toml["config"]["battery"]["min_batt_lvl"]
        self.batt_energy_ratio = toml["config"]["battery"]["batt_energy_ratio"]
        self.energy_density = toml["config"]["battery"][
            "energy_density"
        ] * TimeConverter.hours_to_sec(
            1
        )  # J/kg

    def calc_weight(self, energy_required: float) -> float:
        weight = energy_required / (self.energy_density * self.batt_energy_ratio)
        if weight < 0:
            logging.error("Calculated battery weight is negative. Check energy requirements.")
            raise ValueError("Battery weight cannot be negative.")
        self.weight = weight
        return weight


class Motor:

    def __init__(self):
        logging.debug("Initializing Motor class...")

        pass

    def n_motors(self) -> int:
        """
        Placeholder method to calculate the number of motors required based on power.
        Returns:
        int: Number of motors required.
        """
        n_vert = 8
        n_hor = 2
        return n_vert + n_hor  # Total number of motors (8 vertical + 2 horizontal)

    def weight(self):
        """
        Placeholder method to estimate motor weight based on specific power.
        Returns:
        float: Estimated weight of the motor in kg.
        """
        return (
            0.16 * self.n_motors()
        )  # Placeholder value, should be replaced with a real calculation
    
    def max_thrust(self) -> float:
        """
        Placeholder method to calculate the maximum thrust produced by the motor.
        Returns:
        float: Maximum thrust including all motors in Newtons.
        """
        T_motor_AS2820_KV880_max_thrust = 24.153779 # N
        return T_motor_AS2820_KV880_max_thrust * self.n_motors()  # N


class VertPropeller:
    def __init__(
        self,
        diameter: float = ImperialConverter.len_in_m(12),
        n_blades: int = 8,
        η_prop: float = 0.85,
    ):
        """
        Initialize the VertPropeller class with default parameters.
        Parameters:
        diameter (float): Diameter of the propeller in meters.
        n_blades (int): Number of blades.
        η_prop (float): Propeller efficiency.
        """
        logging.debug("Initializing VertPropeller class...")

        self.η_prop = η_prop
        self.diameter = diameter
        self.n_blades = n_blades

    @property
    def area(self):
        return np.pi * (self.diameter / 2) ** 2 * self.n_blades

    def power(self, thrust: float, FM=toml["config"]["propeller"]["FM"]) -> float:
        """
        Calculate the power required for the propeller.
        Parameters:
        thrust (float): Thrust in Newtons.
        Returns:
        float: Power in Watts.
        """
        return thrust**1.5 / np.sqrt(2 * ρ * self.area * self.η_prop) / FM

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
    def __init__(
        self, diameter: float = ImperialConverter.len_in_m(18), n_blades: int = 2
    ):
        logging.debug("Initializing HorPropeller class...")

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

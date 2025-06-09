from __future__ import annotations
from typing import TYPE_CHECKING
import logging
import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib import colors

if TYPE_CHECKING:
    from prelim_des.drone import Drone
    from prelim_des.mission import Mission
from constants import ρ, g
from globals import main_dir
from prelim_des.utils.import_toml import load_toml
import utils.define_logging  # do not remove this line, it sets up logging configuration
from utils.unit_converter import TimeConverter, ImperialConverter
from matplotlib.patches import Patch

toml = load_toml()


# Professional color palette
OEW_COLOR = "#6C7A89"  # Slate gray-blue
PIZZA_COLOR = "#F4A300"  # Ochre (muted orange)
BATTERY_COLOR = "#3CB371"  # Medium sea green
MTOW_COLOR = "#222222"  # Almost black
POINT_COLOR = "#222222"  # Same as MTOW for points


class Performance:
    def __init__(self, drone: Drone, mission: Mission):
        """
        Initialize the Performance class with default parameters.
        Parameters:
        drone (Drone): The drone object to which this performance model belongs.
        mission (Mission): The mission object to which this performance model belongs.
        """
        logging.debug("Initializing Performance class...")

        self.L_over_D_cruise = toml["config"]["performance"]["L_over_D_cruise"]
        self.T_over_D_cruise = toml["config"]["performance"]["T_over_D_cruise"]
        self.T_over_W_takeoff = toml["config"]["performance"]["T_over_W_takeoff"]
        self.drone = drone
        self.mission = mission
        self.V_cruise = toml["config"]["mission"]["cruise_speed"]

    def wing_area(self, mass):
        """Calculate required wing surface area for given weight and cruise speed."""
        S = (
            mass * g / (0.5 * ρ * self.V_cruise**2 * self.drone.aero.CL_cruise)
        )  # Using lift equation with horizontal equilibrium
        return S

    def cruise_thrust(self, D):
        """Calculate thrust required during cruise."""
        T = D * self.T_over_D_cruise  # Assuming level flight, thrust equals drag
        return T

    def power(self, T, V):
        """Calculate power required during cruise."""
        P = T * V  # Power = Thrust * Velocity
        return P

    def takeoff_thrust(self, weight, V_TO, use_T_over_W=False):
        """Calculate thrust required during takeoff."""
        if use_T_over_W:
            T = weight * g * self.T_over_W_takeoff
        else:
            T = weight * g + self.drone.aero.drag(
                V_TO, TO_or_LD=True
            )  # Assuming thrust equals weight plus drag at takeoff speed
        return T

    def landing_thrust(self, weight, V_LD, use_T_over_W=False):
        """Calculate thrust required during landing."""
        if use_T_over_W:
            T = weight * g * self.T_over_W_takeoff
        else:
            T = weight * g - self.drone.aero.drag(V_LD, TO_or_LD=True)
        return T

    def hover_thrust(self, weight):
        """Calculate thrust required during loiter."""
        T = weight * g
        return T

    def cruise_energy(self, range):

        energy_cruise = (
            self.drone.MTOW
            * range
            / (
                self.drone.propulsion.η_elec
                * self.drone.propulsion.η_prop
                * self.L_over_D_cruise
                / g
            )
        )  # Energy required for cruise

        return energy_cruise

    def cruise_range(self, energy):
        """
        Calculate the range during cruise based on available energy.
        Parameters:
        energy (float): Available energy in Joules.
        Returns:
        float: Range in meters.
        """
        range_cruise = (
            energy
            * self.drone.propulsion.η_elec
            * self.drone.propulsion.η_prop
            * self.L_over_D_cruise
            / (self.drone.MTOW * g)
        )
        return range_cruise

    def leg_energy(self, leg):
        """
        Calculate the energy required for a single leg of the mission.
        """
        PL_mass = leg["payload_mass"]

        cruise_energy = self.cruise_energy(leg["distance"])
        cruise_power = cruise_energy / (
            leg["distance"] / leg["cruise_speed"]
        )  # Power = Energy / Time

        takeoff_thrust = self.takeoff_thrust(self.drone.OEW + PL_mass, leg["TO_speed"], use_T_over_W=False)
        takeoff_power = self.drone.propulsion.ver_prop.power(takeoff_thrust)
        take_off_energy = takeoff_power * (leg["TO_time"])

        landing_thrust = self.landing_thrust(self.drone.OEW + PL_mass, leg["LD_speed"], use_T_over_W=False)
        landing_power = self.drone.propulsion.ver_prop.power(landing_thrust)
        landing_energy = landing_power * (leg["LD_time"])

        hover_thrust = self.hover_thrust(self.drone.OEW + PL_mass)
        hover_power = self.drone.propulsion.ver_prop.power(hover_thrust)
        hover_energy = hover_power * leg["loitering_time"]

        leg_energy = cruise_energy + take_off_energy + landing_energy + hover_energy
        print(f"Energy breakdown: Cruise Energy: {cruise_energy[0]:.2f} J, "
              f"Takeoff Energy: {take_off_energy[0]:.2f} J, "
              f"Landing Energy: {landing_energy[0]:.2f} J, "
              f"Hover Energy: {hover_energy[0]:.2f} J",
              f"Leg Mass: {self.drone.OEW + PL_mass} kg")

        return leg_energy, cruise_power, takeoff_power, landing_power, hover_power

    def mission_energy(self):
        """
        Calculate the total energy required for the mission.
        """
        mission_energy = 0
        for leg in self.mission.legs_dict:

            try:
                leg_energy, cruise_power, takeoff_power, landing_power, hover_power = (
                    self.leg_energy(leg)
                )
                mission_energy += leg_energy
                # Store or process the energy and power values as needed
            except Exception as e:
                logging.error(f"Error calculating energy for leg {leg}: {e}")
                raise ValueError(f"Error calculating energy for leg {leg}: {e}")

        mission_energy = mission_energy / (
            1 - self.drone.propulsion.battery.min_batt_lvl
        )  # Adjust for minimum battery level

        return mission_energy

    def cruise_noise(self, baseline_noise=95, plot=False):
        """
        Calculate the noise level during cruise.
        Parameters:
        baseline_noise (float): Baseline noise level in dB at 1 m distance.
        Returns:
        float: Noise level during cruise in dB.
        """
        # Assuming a simple model where noise increases with speed
        h = toml["config"]["mission"]["cruise_height"]
        height_array = np.linspace(1, h, 100)
        noise = 10 * np.log10(10 ** (baseline_noise / 10) / height_array**2)

        if plot:
            plt.figure(figsize=(10, 6))
            plt.plot(height_array, noise, label="Cruise Noise Level")
            plt.title("Cruise Noise Level vs Height")
            plt.xlabel("Height (m)")
            plt.ylabel("Noise Level (dB)")
            plt.grid()
            plt.legend()
            plt.tight_layout()
            plot_path = os.path.join(main_dir, "prelim_des", "plots")
            plt.savefig(os.path.join(plot_path, "cruise_noise.png"), dpi=300)
        return noise

    def payload_range_diagram(self, n_battery_max=8, E_total=5e6):
        """
        Plot a structured payload-range curve with mass breakdown.
        """
        n_pizza_max = toml["config"]["payload"]["n_box"]
        pizza_mass = toml["config"]["payload"]["box_weight"]

        # E_density = TimeConverter.hours_to_sec(
        #     toml["config"]["battery"]["energy_density"]
        # )  # J/kg

        # E_battery = E_total / n_battery_max
        # battery_mass = E_battery / E_density
        min_bat_lvl = self.drone.propulsion.battery.min_batt_lvl
        max_battery_mass = float(self.drone.propulsion.battery.weight)
        mass_per_battery = max_battery_mass / n_battery_max
        energy_per_battery = (
            self.mission_energy()[0] * (1 - min_bat_lvl) / n_battery_max
        )
        OEW_base = self.drone.OEW - max_battery_mass

        # Phase 1: Add batteries while keeping max pizzas
        phase1_ranges = []
        phase1_pizza = []
        phase1_battery = []

        for b in range(n_battery_max + 1):
            total_mass = OEW_base + n_pizza_max * pizza_mass + b * mass_per_battery
            if (
                total_mass
                > OEW_base + n_pizza_max * pizza_mass + n_battery_max * mass_per_battery
            ):
                break
            # Calculate available energy for cruise (proportional to battery mass)
            E_available = b * energy_per_battery  # Adjust for minimum battery level

            # Use cruise_range calculation for range
            if total_mass > 0 and E_available > 0:
                self.drone.MTOW = total_mass  # Temporarily set MTOW for calculation
                r = float(self.cruise_range(E_available) / 1000)  # convert to km
            else:
                r = 0
            phase1_ranges.append(r)
            phase1_pizza.append(n_pizza_max * pizza_mass)
            phase1_battery.append(b * mass_per_battery)

        # Phase 2: Remove pizzas at full battery
        phase2_ranges = []
        phase2_pizza = []
        phase2_battery = []

        for p in range(n_pizza_max - 1, -1, -1):
            total_mass = OEW_base + p * pizza_mass + n_battery_max * mass_per_battery
            # Calculate available energy for cruise (proportional to battery mass)
            E_available = n_battery_max * energy_per_battery

            if total_mass > 0 and E_available > 0:
                self.drone.MTOW = total_mass  # Temporarily set MTOW for calculation
                r = float(self.cruise_range(E_available) / 1000)  # convert to km
            else:
                r = 0
            phase2_ranges.append(r)
            phase2_pizza.append(p * pizza_mass)
            phase2_battery.append(n_battery_max * mass_per_battery)

        # Combine phases
        ranges = np.array(phase1_ranges + phase2_ranges)
        pizza_masses = np.array(phase1_pizza + phase2_pizza)
        battery_masses = np.array(phase1_battery + phase2_battery)
        oews = np.full_like(ranges, OEW_base)
        mtows = oews + pizza_masses + battery_masses

        # Plot
        plt.figure(figsize=(10, 6))

        # OEW line
        plt.axhline(OEW_base, color="gray", linestyle="--", linewidth=1.2, label="OEW")

        # MTOW line at point B
        mtow_B = OEW_base + n_pizza_max * pizza_mass + n_battery_max * mass_per_battery
        plt.axhline(
            mtow_B, color="black", linestyle="--", linewidth=1.2, label="MTOW (at B)"
        )

        # Stacked breakdown (plot battery mass on top) as stepwise bands
        plt.fill_between(ranges, 0, oews, color=OEW_COLOR, label="OEW", step="pre")
        plt.fill_between(
            ranges,
            oews,
            oews + pizza_masses,
            color=PIZZA_COLOR,
            label="Pizza Mass",
            step="pre",
        )
        plt.fill_between(
            ranges,
            oews + pizza_masses,
            oews + pizza_masses + battery_masses,
            color=BATTERY_COLOR,
            label="Battery Mass",
            step="pre",
        )
        plt.step(
            ranges, mtows, where="pre", color=MTOW_COLOR, linewidth=1.5, label="MTOW"
        )
        plt.scatter(ranges, mtows, color=POINT_COLOR, zorder=5)

        # Annotate key points
        # Point A: first point
        plt.annotate(
            "A",
            xy=(ranges[0], mtows[0]),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=10,
            fontweight="bold",
            color="darkred",
            arrowprops=dict(arrowstyle="->", lw=1, color="gray"),
        )

        # Point B: end of phase 1
        B_index = n_battery_max
        plt.annotate(
            "B",
            xy=(ranges[B_index], mtows[B_index]),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=10,
            fontweight="bold",
            color="darkred",
            arrowprops=dict(arrowstyle="->", lw=1, color="gray"),
        )

        # Point C: last point
        plt.annotate(
            "C",
            xy=(ranges[-1], mtows[-1]),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=10,
            fontweight="bold",
            color="darkred",
            arrowprops=dict(arrowstyle="->", lw=1, color="gray"),
        )

        plt.xlabel("Range [km]")
        plt.ylabel("Total Weight [kg]")
        plt.title("Payload-Range Diagram with Mass Breakdown")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

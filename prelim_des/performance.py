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

    def wing_area(self, mass, V_cruise=toml["config"]["mission"]["cruise_speed"]):
        """Calculate required wing surface area for given weight and cruise speed."""
        S = (
            mass * g / (0.5 * ρ * V_cruise**2 * self.drone.aero.CL_cruise)
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

    def landing_thrust(self, weight, V_LD):
        """Calculate thrust required during landing."""
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

    def leg_energy(self, leg):
        """
        Calculate the energy required for a single leg of the mission.
        """
        PL_mass = leg["payload_mass"]

        cruise_energy = self.cruise_energy(leg["distance"])
        cruise_power = cruise_energy / (
            leg["distance"] / leg["cruise_speed"]
        )  # Power = Energy / Time

        takeoff_thrust = self.takeoff_thrust(self.drone.OEW + PL_mass, leg["TO_speed"])
        takeoff_power = self.drone.propulsion.ver_prop.power(takeoff_thrust)
        take_off_energy = takeoff_power * (leg["TO_time"])

        landing_thrust = self.landing_thrust(self.drone.OEW + PL_mass, leg["LD_speed"])
        landing_power = self.drone.propulsion.ver_prop.power(landing_thrust)
        landing_energy = landing_power * (leg["LD_time"])

        hover_thrust = self.hover_thrust(self.drone.OEW + PL_mass)
        hover_power = self.drone.propulsion.ver_prop.power(hover_thrust)
        hover_energy = hover_power * leg["loitering_time"]

        leg_energy = cruise_energy + take_off_energy + landing_energy + hover_energy
        # print(f"Energy breakdown: Cruise Energy: {cruise_energy[0]:.2f} J, "
        #       f"Takeoff Energy: {take_off_energy[0]:.2f} J, "
        #       f"Landing Energy: {landing_energy[0]:.2f} J, "
        #       f"Hover Energy: {hover_energy[0]:.2f} J")

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



    # def payload_range_diagram(self, n_battery_max=8, E_total=5e6, plot=True):
    #     """
    #     Plot payload-range diagram for discrete payloads and battery combinations,
    #     with color-coded markers based on number of batteries.
    #     """
    #     payloads = []
    #     ranges = []
    #     battery_counts = []
        
    #     n_pizza_max = toml["config"]["payload"]["n_box"]
    #     pizza_mass = toml["config"]["payload"]["box_weight"]
    #     E_density = TimeConverter.hours_to_sec(toml["config"]["battery"]["energy_density"])  # J/kg

    #     E_battery = E_total / n_battery_max
    #     battery_mass = E_battery / E_density
    #     max_battery_mass = battery_mass * n_battery_max

    #     for n_pizzas in range(n_pizza_max + 1):
    #         for n_batteries in range(1, n_battery_max + 1):
    #             total_payload_mass = n_pizzas * pizza_mass
    #             total_battery_mass = n_batteries * battery_mass
    #             OEW = self.drone.OEW - max_battery_mass
    #             MTOW = OEW + total_payload_mass + total_battery_mass

    #             if MTOW > self.drone.MTOW:
    #                 continue

    #             E_total_combo = n_batteries * E_battery

    #             dummy_leg = {
    #                 "payload_mass": total_payload_mass,
    #                 "distance": 0.1,
    #                 "cruise_speed": 5,
    #                 "cruise_h": ImperialConverter.len_ft_m(200),
    #                 "TO_speed": 2,
    #                 "LD_speed": 3,
    #                 "loitering_time": 30.0,
    #             }
    #             dummy_leg["TO_time"] = dummy_leg["cruise_h"] / dummy_leg["TO_speed"]
    #             dummy_leg["LD_time"] = dummy_leg["cruise_h"] / dummy_leg["LD_speed"]

    #             _, _, takeoff_power, landing_power, hover_power = self.leg_energy(dummy_leg)

    #             E_fixed = (
    #                 takeoff_power * dummy_leg["TO_time"]
    #                 + landing_power * dummy_leg["LD_time"]
    #                 + hover_power * dummy_leg["loitering_time"]
    #             )

    #             E_cruise = E_total_combo - E_fixed
    #             if E_cruise <= 0:
    #                 continue

    #             L_over_D = self.L_over_D_cruise
    #             eta = self.drone.propulsion.η_elec * self.drone.propulsion.η_prop
    #             range_km = E_cruise * eta * L_over_D / (MTOW * g) / 1000  # km

    #             payloads.append(total_payload_mass)
    #             ranges.append(range_km)
    #             battery_counts.append(n_batteries)

    #     if plot:
    #         plt.figure(figsize=(8, 5))
    #         cmap = plt.cm.rainbow
    #         norm = colors.BoundaryNorm(np.arange(0.5, 9.5, 1), cmap.N)
    #         # scatter = plt.scatter(ranges, payloads, c=battery_counts, cmap='viridis', marker='o')
    #         scatter = plt.scatter(
    #             ranges,
    #             payloads,
    #             c=battery_counts,
    #             cmap=cmap,
    #             norm=norm,
    #             marker='o',
    #             edgecolor='black',
    #         )
    #         cbar = plt.colorbar(scatter)
    #         cbar.set_ticks(np.arange(1, n_battery_max + 1))
    #         cbar.set_label("Number of Batteries")
            
    #         plt.xlabel("Range [km]")
    #         plt.ylabel("Payload [kg]")
    #         plt.title("Payload-Range Diagram (Colored by Battery Count)")
    #         plt.grid(True)
    #         plt.tight_layout()
    #         plt.show()

    #     return ranges, payloads, battery_counts


    def payload_range_diagram(self, n_battery_max=8, E_total=5e6, plot=True):
        """
        Plot a structured payload-range curve:
        Phase 1: Max payload, increase batteries
        Phase 2: Max batteries, decrease payload
        Annotates key points: A (max payload), B (mid), C (max range)
        """
        # payloads = []
        total_weights = []
        ranges = []

        n_pizza_max = toml["config"]["payload"]["n_box"]
        pizza_mass = toml["config"]["payload"]["box_weight"]
        E_density = TimeConverter.hours_to_sec(toml["config"]["battery"]["energy_density"])  # J/kg

        E_battery = E_total / n_battery_max
        battery_mass = E_battery / E_density
        max_battery_mass = battery_mass * n_battery_max

        OEW_base = self.drone.OEW - max_battery_mass

        point_labels = {}

        # Phase 1: Increase batteries at max pizzas
        n_pizzas = n_pizza_max
        for n_batteries in range(0, n_battery_max + 1):
            total_payload_mass = n_pizzas * pizza_mass
            total_battery_mass = n_batteries * battery_mass
            OEW = OEW_base
            MTOW = OEW + total_payload_mass + total_battery_mass

            if MTOW > self.drone.MTOW:
                continue

            E_total_combo = n_batteries * E_battery

            dummy_leg = {
                "payload_mass": total_payload_mass,
                "distance": 0.1,
                "cruise_speed": 5,
                "cruise_h": ImperialConverter.len_ft_m(200),
                "TO_speed": 2,
                "LD_speed": 3,
                "loitering_time": 30.0,
            }
            dummy_leg["TO_time"] = dummy_leg["cruise_h"] / dummy_leg["TO_speed"]
            dummy_leg["LD_time"] = dummy_leg["cruise_h"] / dummy_leg["LD_speed"]

            _, _, takeoff_power, landing_power, hover_power = self.leg_energy(dummy_leg)

            E_fixed = (
                takeoff_power * dummy_leg["TO_time"]
                + landing_power * dummy_leg["LD_time"]
                + hover_power * dummy_leg["loitering_time"]
            )

            E_cruise = E_total_combo - E_fixed
            range_km = 0 if E_cruise <= 0 else (
                E_cruise * self.drone.propulsion.η_elec * self.drone.propulsion.η_prop * self.L_over_D_cruise
                / (MTOW * g) / 1000
            )[0]

            # payloads.append(total_payload_mass)
            total_weights.append(MTOW)
            ranges.append(range_km)

            if n_batteries == 0:
                point_labels['A'] = (range_km, total_payload_mass)
            elif n_batteries == n_battery_max:
                point_labels['B'] = (range_km, total_payload_mass)

        # Phase 2: Decrease pizzas at max batteries
        n_batteries = n_battery_max
        for n_pizzas in range(n_pizza_max - 1, -1, -1):  # avoid duplicate
            total_payload_mass = n_pizzas * pizza_mass
            total_battery_mass = n_batteries * battery_mass
            OEW = OEW_base
            MTOW = OEW + total_payload_mass + total_battery_mass

            if MTOW > self.drone.MTOW:
                continue

            E_total_combo = n_batteries * E_battery

            dummy_leg = {
                "payload_mass": total_payload_mass,
                "distance": 0.1,
                "cruise_speed": 5,
                "cruise_h": ImperialConverter.len_ft_m(200),
                "TO_speed": 2,
                "LD_speed": 3,
                "loitering_time": 30.0,
            }
            dummy_leg["TO_time"] = dummy_leg["cruise_h"] / dummy_leg["TO_speed"]
            dummy_leg["LD_time"] = dummy_leg["cruise_h"] / dummy_leg["LD_speed"]

            _, _, takeoff_power, landing_power, hover_power = self.leg_energy(dummy_leg)

            E_fixed = (
                takeoff_power * dummy_leg["TO_time"]
                + landing_power * dummy_leg["LD_time"]
                + hover_power * dummy_leg["loitering_time"]
            )

            E_cruise = E_total_combo - E_fixed
            range_km = 0 if E_cruise <= 0 else (
                E_cruise * self.drone.propulsion.η_elec * self.drone.propulsion.η_prop * self.L_over_D_cruise
                / (MTOW * g) / 1000
            )[0]

            # payloads.append(total_payload_mass)
            total_weights.append(MTOW)
            ranges.append(range_km)

            if n_pizzas == 0:
                point_labels['C'] = (range_km, total_payload_mass)

        if plot:
            plt.figure(figsize=(8, 5))

            # Combine both phases with color tags
            phase_data = []

            # Rebuild the data (needed to attach phase color and label)
            # Phase 1: Increase batteries
            n_pizzas = n_pizza_max
            for n_batteries in range(0, n_battery_max + 1):
                total_payload_mass = n_pizzas * pizza_mass
                total_battery_mass = n_batteries * battery_mass
                OEW = OEW_base
                MTOW = OEW + total_payload_mass + total_battery_mass

                if MTOW > self.drone.MTOW:
                    continue

                E_total_combo = n_batteries * E_battery

                dummy_leg["payload_mass"] = total_payload_mass
                _, _, takeoff_power, landing_power, hover_power = self.leg_energy(dummy_leg)

                E_fixed = (
                    takeoff_power * dummy_leg["TO_time"]
                    + landing_power * dummy_leg["LD_time"]
                    + hover_power * dummy_leg["loitering_time"]
                )

                E_cruise = E_total_combo - E_fixed
                range_km = 0 if E_cruise <= 0 else (
                    E_cruise * self.drone.propulsion.η_elec * self.drone.propulsion.η_prop * self.L_over_D_cruise
                    / (MTOW * g) / 1000
                )[0]

                label = None
                if n_batteries == 0:
                    label = 'A'
                elif n_batteries == n_battery_max:
                    label = 'B'

                if n_batteries == 0 and n_pizzas == 0:
                    label = 'OEW'

                phase_data.append((range_km, MTOW, 'batteries', label))

            # Phase 2: Decrease pizzas
            n_batteries = n_battery_max
            for n_pizzas in range(n_pizza_max - 1, -1, -1):
                total_payload_mass = n_pizzas * pizza_mass
                total_battery_mass = n_batteries * battery_mass
                OEW = OEW_base
                MTOW = OEW + total_payload_mass + total_battery_mass

                if MTOW > self.drone.MTOW:
                    continue

                E_total_combo = n_batteries * E_battery

                dummy_leg["payload_mass"] = total_payload_mass
                _, _, takeoff_power, landing_power, hover_power = self.leg_energy(dummy_leg)

                E_fixed = (
                    takeoff_power * dummy_leg["TO_time"]
                    + landing_power * dummy_leg["LD_time"]
                    + hover_power * dummy_leg["loitering_time"]
                )

                E_cruise = E_total_combo - E_fixed
                range_km = 0 if E_cruise <= 0 else (
                    E_cruise * self.drone.propulsion.η_elec * self.drone.propulsion.η_prop * self.L_over_D_cruise
                    / (MTOW * g) / 1000
                )[0]

                label = 'C' if n_pizzas == 0 else None
                phase_data.append((range_km, MTOW, 'pizzas', label))

            # Sort and unpack
            phase_data.sort(key=lambda x: x[0])
            ranges_sorted = [x[0] for x in phase_data]
            weights_sorted = [x[1] for x in phase_data]
            colors = ['blue' if x[2] == 'batteries' else 'green' for x in phase_data]

            # Plot
            plt.scatter(ranges_sorted, weights_sorted, c=colors, label='Flight envelope', zorder=2)
            plt.step(ranges_sorted, weights_sorted, color='gray', linestyle='--', zorder=1)

            # Annotate key points
            for range_km, weight, phase, label in phase_data:
                if label:
                    plt.annotate(
                        f'{label}',
                        xy=(range_km, weight),
                        xytext=(5, 5),
                        textcoords='offset points',
                        fontsize=10,
                        fontweight='bold',
                        color='darkred',
                        arrowprops=dict(arrowstyle='->', lw=1, color='gray')
                    )

            plt.xlabel("Range [km]")
            plt.ylabel("Total Weight [kg]")
            plt.ylim([min(weights_sorted) * 0.98, self.drone.MTOW * 1.02])
            plt.title("Payload-Range Diagram")

            plt.legend(handles=[
                Patch(color='blue', label='Increasing batteries'),
                Patch(color='green', label='Decreasing pizzas'),
            ])

            plt.grid(True)
            plt.tight_layout()
            plt.show()


        return ranges, MTOW, #payloads
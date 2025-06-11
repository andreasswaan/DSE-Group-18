import os
import matplotlib.pyplot as plt
from globals import main_dir
from prelim_des.power import PropulsionSystem
from prelim_des.elems import Wing, Fuselage, LandingGear
from prelim_des.structure import Structure
from prelim_des.aerodynamics import Aerodynamics
from prelim_des.constants import *
from prelim_des.performance import Performance
from prelim_des.utils.import_toml import load_toml
from src.initial_sizing.database.read_database import get_MTOW_from_payload
import utils.define_logging  # do not remove this line, it sets up logging configuration
import logging

toml = load_toml()


class Drone:
    aero: Aerodynamics
    wing: Wing
    structure: Structure
    propulsion: PropulsionSystem
    perf: Performance

    MTOW: float
    OEW: float

    def __init__(self):
        logging.debug("Initializing Drone class...")
        # Subsystems
        self.aero = Aerodynamics(self)
        self.wing = Wing(self)
        self.fuselage = Fuselage(self)
        self.landing_gear = LandingGear(self)
        self.structure = Structure(self)
        self.propulsion = PropulsionSystem(self)

    def class_1_weight_estimate(self):
        """
        Estimate the weight of the drone using a class 1 weight estimate.
        This method uses the database to calculate the empty operating weight (OEW) and maximum takeoff weight (MTOW).
        """
        self.max_payload = (
            toml["config"]["payload"]["n_box"] * toml["config"]["payload"]["box_weight"]
        )
        design_payload = self.structure.delivery_mechanism_weight() + self.max_payload
        self.MTOW = get_MTOW_from_payload(design_payload)
        self.OEW = self.MTOW - self.max_payload
        if self.OEW <= 0:
            logging.error(
                "OEW cannot be zero or negative. Check the payload and MTOW values."
            )
            raise ValueError(
                "OEW cannot be zero or negative. Check the payload and MTOW values."
            )
        return self.MTOW, self.OEW

    def class_2_weight_estimate(self, transition=False, print=False):
        """
        Estimate the weight of the drone using a class 2 weight estimate.
        """
        if self.wing.S is None:
            logging.error(
                "Wing area (S) must be defined before performing class 2 weight estimate."
            )
            raise ValueError(
                "Wing area (S) must be defined before performing class 2 weight estimate."
            )

        mission_energy = self.perf.mission_energy(transition)
        self.OEW = (
            self.wing.roskam_weight()
            + self.fuselage.weight()
            + self.landing_gear.weight()
            + self.propulsion.weight(mission_energy)
        )
        if print:
            print(
                f"Mission Energy: {mission_energy[0]:.2f} J"
                if mission_energy is not None
                else "Mission Energy: Not calculated"
            )
            print(
                f"Battery Weight: {self.propulsion.battery.calc_weight(energy_required=mission_energy)[0]:.2f} kg"
                if hasattr(self.propulsion, "battery")
                else "Battery Weight: Not available"
            )
            print(
                f"Motor Weight: {self.propulsion.motor.weight():.2f} kg"
                if hasattr(self.propulsion, "motor")
                else "Motor Weight: Not available"
            )
            print(
                f"Propeller Weight: {self.propulsion.ver_prop.weight():.2f} kg"
                if hasattr(self.propulsion, "ver_prop")
                else "Propeller Weight: Not available"
            )
            print(
                f"Component Weights: Wing = {self.wing.roskam_weight():.2f} kg, "
                f"Fuselage = {self.fuselage.weight():.2f} kg, "
                f"Landing Gear = {float(self.landing_gear.weight()):.2f} kg, "
                f"Propulsion = {self.propulsion.weight(mission_energy)[0]:.2f} kg"
            )

        self.max_payload = (
            toml["config"]["payload"]["n_box"] * toml["config"]["payload"]["box_weight"]
        )
        self.MTOW = self.OEW + self.max_payload
        logging.info(
            f"Class 2 Weight Estimate: MTOW = {self.MTOW[0]:.2f} kg, OEW = {self.OEW[0]:.2f} kg"
        )

        return self.MTOW, self.OEW

    def iterative_weight_estimate(
        self, transition=False, max_iterations=100, tolerance=0.01, plot=False
    ):
        """
        Perform an iterative weight estimate until convergence.
        """
        if (
            not hasattr(self, "MTOW")
            or not hasattr(self, "OEW")
            or self.MTOW is None
            or self.OEW is None
        ):
            self.class_1_weight_estimate()
        mtow_history = [self.MTOW]
        self.wing.S = self.perf.wing_area(self.MTOW)
        S_history = [self.wing.S]
        for i in range(max_iterations):
            MTOW_prev = self.MTOW
            self.class_2_weight_estimate(transition)
            mtow_history.append(self.MTOW)
            self.wing.S = self.perf.wing_area(self.MTOW)
            S_history.append(self.wing.S)
            if abs(self.MTOW - MTOW_prev) < tolerance * MTOW_prev:
                print(
                    f"Converged after {i + 1} iterations: MTOW = {self.MTOW[0]:.2f} kg, OEW = {self.OEW[0]:.2f} kg"
                )
                break
        else:
            logging.error(
                "Weight estimate did not converge within the maximum number of iterations."
            )
            raise ValueError(
                "Weight estimate did not converge within the maximum number of iterations."
            )

        if plot:
            plot_path = os.path.join(main_dir, "prelim_des", "plots", "weight_estimate")
            os.makedirs(plot_path, exist_ok=True)

            plt.figure(figsize=(8, 5))
            plt.plot(mtow_history, marker="o")
            plt.xticks(range(len(mtow_history)))
            plt.xlabel("Iteration Number [-]")
            plt.ylabel("MTOW [kg]")
            plt.title("MTOW Convergence after Class II Weight Estimation")
            plt.grid(True, linestyle="--", alpha=0.7)
            plt.tight_layout()
            plt.savefig(os.path.join(plot_path, "mtow_convergence.png"), dpi=300)
            plt.close()

            plt.figure(figsize=(8, 5))
            plt.plot(S_history, marker="o")
            plt.xticks(range(len(S_history)))
            plt.xlabel("Iteration Number [-]")
            plt.ylabel("Wing Area [m²]")
            plt.title("Wing Area Convergence after Class II Weight Estimation")
            plt.grid(True, linestyle="--", alpha=0.7)
            plt.tight_layout()
            plt.savefig(os.path.join(plot_path, "wing_area_convergence.png"), dpi=300)
            plt.close()

        return self.MTOW, self.OEW

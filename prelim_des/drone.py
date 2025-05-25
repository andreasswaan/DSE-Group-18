import matplotlib.pyplot as plt
from prelim_des.power import PropulsionSystem
from prelim_des.elems import Wing, Fuselage, LandingGear
from prelim_des.structure import Structure
from prelim_des.aerodynamics import Aerodynamics
from prelim_des.constants import *
from prelim_des.performance import Performance
from prelim_des.utils.import_toml import load_toml
from src.initial_sizing.database.read_database import get_MTOW_from_payload

toml = load_toml()

class Drone:
    aero:Aerodynamics
    wing:Wing
    structure:Structure
    propulsion:PropulsionSystem
    perf:Performance
    
    MTOW: float
    OEW: float
    
    def __init__(self):

        # Subsystems
        self.aero = Aerodynamics(self)
        self.wing = Wing(self)
        self.fuselage = Fuselage(self)
        self.landing_gear = LandingGear(self)
        self.structure = Structure(self)
        self.propulsion = PropulsionSystem(self)
        # self.perf = Performance(self)

    def class_1_weight_estimate(self):
        """
        Estimate the weight of the drone using a class 1 weight estimate.
        This method uses the database to calculate the empty operating weight (OEW) and maximum takeoff weight (MTOW).
        """
        self.max_payload = toml['config']['payload']['n_box'] * toml['config']['payload']['box_weight']
        self.MTOW = get_MTOW_from_payload(self.max_payload)
        self.OEW = self.MTOW - self.max_payload
        if self.OEW <= 0:
            raise ValueError("OEW cannot be zero or negative. Check the payload and MTOW values.")
        return self.MTOW, self.OEW

    def class_2_weight_estimate(self):
        """
        Estimate the weight of the drone using a class 2 weight estimate.
        """
        if self.wing.S is None:
            raise ValueError("Wing area (S) must be defined before performing class 2 weight estimate.")
        
        mission_energy = self.perf.mission_energy()
        self.OEW = (
            self.wing.weight()
            + self.fuselage.weight()
            + self.landing_gear.weight()
            + self.propulsion.weight(mission_energy)
        )
        print(f"Component Weights: Wing = {self.wing.weight()[0]:.2f} kg, "
              f"Fuselage = {self.fuselage.weight()[0]:.5f} kg, "
                f"Landing Gear = {self.landing_gear.weight()[0]:.2f} kg, "
                f"Propulsion = {self.propulsion.weight(mission_energy)[0]:.2f} kg")
        self.max_payload = toml['config']['payload']['n_box'] * toml['config']['payload']['box_weight']
        self.MTOW = self.OEW + self.max_payload
        print(f"Class 2 Weight Estimate: MTOW = {self.MTOW[0]:.2f} kg, OEW = {self.OEW[0]:.2f} kg")
        
        return self.MTOW, self.OEW
    
    def iterative_weight_estimate(self, max_iterations=100, tolerance=0.01, plot=False):
        """
        Perform an iterative weight estimate until convergence.
        """
        if self.MTOW is None or self.OEW is None:
            self.class_1_weight_estimate()
        mtow_history = [self.MTOW]
        self.wing.S = self.perf.wing_area(self.MTOW)
        for i in range(max_iterations):
            MTOW_prev = self.MTOW
            self.class_2_weight_estimate()
            mtow_history.append(self.MTOW)
            self.wing.S = self.perf.wing_area(self.MTOW)
            if abs(self.MTOW - MTOW_prev) < tolerance * MTOW_prev:
                break
        else:
            raise ValueError("Weight estimate did not converge within the maximum number of iterations.")
        
        if plot:
            plt.figure(figsize=(8, 5))
            plt.plot(mtow_history, marker='o')
            plt.xlabel("Iteration")
            plt.ylabel("MTOW")
            plt.title("MTOW Convergence")
            plt.grid(True, linestyle='--', alpha=0.7)
            plt.tight_layout()
            plt.show()
        
        return self.MTOW, self.OEW
    


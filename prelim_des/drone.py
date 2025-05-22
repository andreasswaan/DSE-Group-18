from prelim_des.power import PropulsionSystem
from prelim_des.elems import Wing
from prelim_des.structure import Structure
from prelim_des.constants import *

class Drone:
    def __init__(self, config, mission):
        self.config = config
        self.mission = mission
        self.payload_mass = config['payload_mass']

        # Subsystems
        self.wing = Wing(config['wing'])
        self.structure = Structure()
        self.propulsion = PropulsionSystem()

    def estimate_thrust_for_phase(self, phase):
        # Placeholder: replace with real aerodynamic model
        if phase.mode == 'hover':
            return g * (self.payload_mass + 5)  # hover = full lift
        elif phase.mode == 'cruise':
            return 0.3 * g * self.payload_mass  # lower for cruise
        else:
            return 0.5 * g * self.payload_mass

    def estimate_power_for_phase(self, phase, thrust):
        prop_efficiency = 0.7
        speed = max(1, phase.speed)  # prevent zero
        power = thrust * speed / prop_efficiency / 1000  # kW
        return power

    def mission_sizing(self):
        energy, peak_power = self.mission.total_energy_and_peak_power(self)
        self.propulsion.size(energy_required=energy, peak_power=peak_power)

    def class_2_weight_estimate(self):
        return {
            "wing": self.wing.estimate_weight(),
            "propulsion": self.propulsion.estimate_weight(),
            "structure": self.structure.estimate_weight(),
        }
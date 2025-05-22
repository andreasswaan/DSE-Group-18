class MissionPhase:
    def __init__(self, name, duration_s, speed_mps, altitude_m, mode):
        self.name = name
        self.duration_s = duration_s
        self.speed = speed_mps
        self.altitude = altitude_m
        self.mode = mode  # 'hover', 'cruise', etc.

class Mission:
    def __init__(self, phases):
        self.phases = phases

    def total_energy_and_peak_power(self, drone):
        total_energy = 0
        peak_power = 0

        for phase in self.phases:
            thrust = drone.estimate_thrust_for_phase(phase)
            power = drone.estimate_power_for_phase(phase, thrust)

            energy = power * phase.duration_s / 3600  # Wh
            total_energy += energy
            peak_power = max(peak_power, power)

        return total_energy, peak_power
class PropulsionSystem:
    def __init__(self):
        self.battery = Battery()
        self.motor = Motor()
        self.propeller = Propeller()

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
    def __init__(self, energy_density=220):  # Wh/kg
        self.energy_required = None
        self.energy_density = energy_density

    def size(self, energy_required):
        self.energy_required = energy_required

    def estimate_weight(self):
        return self.energy_required / self.energy_density
    
class Motor:
    def __init__(self, specific_power=5):  # kW/kg
        self.peak_power = None
        self.specific_power = specific_power

    def size(self, peak_power):
        self.peak_power = peak_power

    def estimate_weight(self):
        return self.peak_power / self.specific_power
    
    
class Propeller:
    def __init__(self):
        self.diameter = None

    def size(self, power):
        self.diameter = (power / 5) ** 0.5  # placeholder scaling

    def estimate_weight(self):
        return self.diameter * 2  # placeholder
    


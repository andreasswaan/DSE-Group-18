from prelim_des.drone import Drone
from prelim_des.mission import Mission
from constants import ρ, g
from prelim_des.utils.import_toml import load_toml

toml = load_toml()

class Performance:
    def __init__(self, drone: Drone, mission: Mission):
        """
        Initialize the Performance class with default parameters.	
        Parameters:
        drone (Drone): The drone object to which this performance model belongs.
        mission (Mission): The mission object to which this performance model belongs.
        """	
        self.L_over_D_cruise = toml["config"]["performance"]["L_over_D_cruise"]
        self.T_over_D_cruise = toml["config"]["performance"]["T_over_D_cruise"]
        self.T_over_W_takeoff = toml["config"]["performance"]["T_over_W_takeoff"]
        self.drone = drone
        self.mission = mission

    def wing_area(self, weight, V_cruise=toml["config"]["mission"]["cruise_speed"]):
        """Calculate required wing surface area for given weight and cruise speed."""
        S = weight / (0.5 * ρ * V_cruise**2 * self.drone.aero.CL_cruise)  # Using lift equation with horizontal equilibrium
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
            T = weight * g + self.drone.aero.drag(V_TO, TO_or_LD=True)  # Assuming thrust equals weight plus drag at takeoff speed
        return T
    
    def landing_thrust(self, weight, V_LD):
        """Calculate thrust required during landing."""
        T = weight * g - self.drone.aero.drag(V_LD, TO_or_LD=True)
        return T
    
    def hover_thrust(self, weight, V_loiter):
        """Calculate thrust required during loiter."""
        T = weight * g
        return T
    
    def cruise_energy(self, range):
        η_elec = toml["config"]["propulsion"]["η_elec"]
        η_prop = toml["config"]["propulsion"]["η_prop"]
        L_over_D_cruise = toml["config"]["performance"]["L_over_D_cruise"]
        energy_cruise = range / (η_elec * η_prop * g * L_over_D_cruise)  # Energy required for cruise
        
        return energy_cruise
    
    def leg_energy(self, leg):
        """
        Calculate the energy required for a single leg of the mission.
        """
        PL_mass = leg['payload_mass']
        
        cruise_energy = self.cruise_energy(leg['distance'])
        cruise_power = cruise_energy / (leg['distance'] / leg['cruise_speed'])  # Power = Energy / Time
        
        takeoff_thrust = self.takeoff_thrust(self.drone.OEW + PL_mass, leg['TO_speed'])
        takeoff_power = self.drone.propulsion.ver_prop.power(takeoff_thrust)
        take_off_energy = takeoff_power * (leg['TO_time'])
        
        landing_thrust = self.landing_thrust(self.drone.OEW + PL_mass, leg['LD_speed'])
        landing_power = self.drone.propulsion.ver_prop.power(landing_thrust)
        landing_energy = landing_power * (leg['LD_time'])
        
        hover_thrust = self.hover_thrust(self.drone.OEW + PL_mass, leg['loiter_speed'])
        hover_power = self.drone.propulsion.ver_prop.power(hover_thrust)
        hover_energy = hover_power * leg['loitering_time']
        
        leg_energy = cruise_energy + take_off_energy + landing_energy + hover_energy
        return leg_energy, cruise_power, takeoff_power, landing_power, hover_power
    
    def mission_energy(self):
        """
        Calculate the total energy required for the mission.
        """
        mission_energy = 0
        for leg in self.mission.legs_dict:
            
            try:
                leg_energy, cruise_power, takeoff_power, landing_power, hover_power = self.leg_energy(leg)
                mission_energy += leg_energy
                # Store or process the energy and power values as needed
            except Exception as e:
                print(f"Error calculating energy for leg {leg}: {e}")
        
        return mission_energy
                
            
            
        
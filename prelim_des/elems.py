from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from prelim_des.drone import Drone
import numpy as np
from constants import g
from prelim_des.utils.import_toml import load_toml
from prelim_des.utils.unit_converter import ImperialConverter

toml = load_toml()


class Wing:
    """Class for a wing."""
    S:float
    mac:float
    def __init__(
        self,
        drone: Drone
    ):
        self.drone = drone
        self.span = toml["config"]["wing"]["wing_span"]
        self.thick_over_chord = toml["config"]["wing"]["thickness_over_chord"]
        self.Γ = toml["config"]["wing"]["dihedral"]
        self.Λ_c4 = np.deg2rad(toml["config"]["wing"]["quarter_chord_sweep"])
    
    @property
    def c_root(self):
        """Wing surface [m2]"""
        return 2 * self.S / (self.span * (1 + self.taper))

    @property
    def c_tip(self):
        """Tip chord length [m]"""
        return self.taper * self.c_root

    @property
    def geom_AR(self):
        """Geometric aspect ratio [-]"""
        return self.span**2 / self.S

    def sweep(self, x):
        """Sweep angle [rad] at a given chord fraction [-]"""
        sweep0 = self.Λ_c4
        x0 = 0.25
        return np.arctan(
            np.tan(sweep0) + (x0 - x) * (2 * self.c_root / self.span) * (1 - self.taper)
        )

    def chord(self, y):
        """Chord length at a spanwise position y [m]"""
        return self.c_root + (self.c_tip - self.c_root) * y / (self.span / 2)

    def y_chord(self, c):
        """Spanwise coordinate [m] of chord of length c [m]"""
        return (self.c_root - c) / (self.c_root - self.c_tip) * self.span / 2

    def x_chord(self, c):
        """X-coordinate (from root LE) of chord of length c [m]"""
        return self.y_chord(c) * np.tan(self.sweep(0))

    @property
    def MAC(self):
        """Mean aerodynamic chord length [m]"""
        # Assuming MAC = MGC
        return (
            (2 / 3) * self.c_root * (1 + self.taper + self.taper**2) / (1 + self.taper)
        )

    @property
    def yLEMAC(self):
        """Spanwise position (y-coord) of MAC [m]"""
        return self.y_chord(self.MAC)

    @property
    def xLEMAC(self):
        """Longitudinal position (x-coord) of MAC [m], relative to root LE"""
        return self.x_chord(self.MAC)

    @property
    def taper(self):
        """Longitudinal position (x-coord) of MAC [m], relative to root LE"""
        return 0.2 * (2 - self.Λ_c4 * np.pi / 180)

    def CLalpha(self, mach, eff=0.95):
        """DATCOM method to estimate wing lift slope.

        :param mach: Mach number [-]
        :type mach: float
        :param eff: Airfoil efficiency coefficient, defaults to 0.95
        :type eff: float, optional
        """
        beta = np.sqrt(1 - mach**2)
        return (
            2
            * np.pi
            * self.geom_AR
            / (
                2
                + np.sqrt(
                    4
                    + (self.geom_AR * beta / eff) ** 2
                    * (1 + np.tan(self.sweep(0.5)) ** 2 / beta**2)
                )
            )
        )

    def area_int(self, y0, y1):
        """Area integrator two span positions. It computes the wing area between
        span coord y=y0 and y=y1.

        :param y0: First span coordinate [m]
        :type y0: float
        :param y1: Second span coordinate [m]
        :type y1: float
        :return: Area [m2]
        :rtype: float
        """
        assert y0 >= 0 and y1 >= 0 and y1 > y0, "Specify correct bounds!"
        return (self.chord(y0) + self.chord(y1)) / 2 * (y1 - y0)

    def weight(self):
        """Estimate the wing weight using Cessna method from Roskam V: 
        https://research.tudelft.nl/files/144857482/6.2022_1485.pdf.

        :return: Wing weight [kg]
        :rtype: float
        """
        # Roskam Chapter 5 page 67
        n_ult = self.drone.structure.calc_n_ult()
        roskam_w = ImperialConverter.mass_lbs_kg(
            0.04674 * (ImperialConverter.mass_kg_lbs(self.drone.MTOW))**0.397 
            * (ImperialConverter.area_m2_ft2(self.S))**0.360 
            * n_ult**0.397 * self.geom_AR**1.712)
        
        # Placeholder estimate for the wing weight based
        return (0.4 * self.S + 0.06 * self.drone.MTOW)


class Fuselage:
    def __init__(self, drone: Drone):
        """Fuselage class."""
        self.drone = drone

    @property
    def length(self):
        """Fuselage length [m]"""
        return self.drone.structure.fuselage_length()
    
    @property
    def diameter(self):
        """Fuselage diameter [m]"""
        return self.drone.structure.fuselage_diameter()
    @property
    def perimeter(self):
        """Fuselage perimeter [m]"""
        return self.drone.structure.fuselage_max_perimeter()
    
    def weight(self):
        """Estimate the fuselage weight using Cessna method from Roskam V: 
        https://research.tudelft.nl/files/144857482/6.2022_1485.pdf.

        :return: Fuselage weight [kg]
        :rtype: float
        """
        # Roskam Chapter 5 page 76
        N_pax = toml["config"]["payload"]["n_pax"]
        n_box = toml["config"]["payload"]["n_box"]
        N_pax = max(N_pax, n_box)
        
        return ImperialConverter.mass_lbs_kg(
            14.86 * ImperialConverter.mass_kg_lbs(self.drone.MTOW**0.144)
            * (self.length/self.perimeter)**0.778 
            * ImperialConverter.len_m_ft(self.length)**0.383 
            * (N_pax**0.455)) * (0.5/100) + self.drone.structure.delivery_mechanism_weight()
        # Assuming that a pax weighs 100kg and that a pizza weighs 300 g.
    

class LandingGear:
    def __init__(self, drone: Drone):
        """Landing gear class."""
        self.drone = drone
        # self.n_legs = toml["config"]["landing_gear"]["n_legs"]
        # self.wheel_diameter = toml["config"]["landing_gear"]["wheel_diameter"]
        # self.wheel_width = toml["config"]["landing_gear"]["wheel_width"]
    @property
    def l_s_m(self):
        """Length of the landing gear strut [m]"""
        # Placeholder: replace with real calculation
        return toml["config"]["landing_gear"]["shock_strut_length_MG"]
    def weight(self):
        """Estimate the landing gear weight using Cessna method from Roskam V: 
        Placeholder method used: Update with better method once design is more clear.
        """
        # Roskam Chapter 5 page 81
        n_ult = self.drone.structure.calc_n_ult()
        box_weight = toml["config"]["payload"]["box_weight"]  # kg
        design_landing_weight = self.drone.MTOW - box_weight  # kg
        wheel_tire_assembly_weight = 0.1  # kg, placeholder value
        
        # ImperialConverter.mass_lbs_kg(
            # 0.013 * ImperialConverter.mass_kg_lbs(self.drone.MTOW) + 
            # 0.146 * ImperialConverter.mass_kg_lbs(design_landing_weight**0.417) * n_ult**0.95 
            # * ImperialConverter.len_m_ft(self.l_s_m)**0.183 
            # + ImperialConverter.mass_kg_lbs(wheel_tire_assembly_weight)
        # Placeholder estimate for the landing gear weight based on MTOW and wheel assembly weight, the design landing weight term has been omitted
        return (
            ImperialConverter.mass_lbs_kg(
            0.013 * ImperialConverter.mass_kg_lbs(self.drone.MTOW) +  
            + ImperialConverter.mass_kg_lbs(wheel_tire_assembly_weight))
        )
        


# class HorizontalTail(Wing):
#     def __init__(self,
#                 cr,
#                 ct,
#                 b,
#                 sweep_lst,
#                 mac = None,
#                 Delta_AR = 0,
#                 config = 'T-tail',
#                 motion_type = 'full_moving',
#                 CLmin=None,
#                 ):
#         '''Horizontal tail surface class.

#         :param cr: Root chord length [m]
#         :type cr: float
#         :param ct: Tip chord length [m]
#         :type ct: float
#         :param b: Span length [m]
#         :type b: float
#         :param sweep_lst: Combination of sweep angle [rad] and chord \
#         fraction [-] in tuple (Lambda, c)
#         :type sweep_lst: Tuple[float,float]
#         :param mac: Assumed mean aerodynamic chord length [m], defaults to None
#         :type mac: float, optional
#         :param Delta_AR: Change in effective AR, as a fraction of the \
#         geometric AR, defaults to 0
#         :type Delta_AR: int, optional
#         :param config: Tail configuration, any of 'T-tail', 'canard', \
#         'fuselage-mounted' or 'fin-mounted'
#         :type config: str
#         :param motion_type: Tail configuration motion capabilities. Any \
#         of 'full_moving', 'adjustable' and 'fixed'
#         :type motion_type: str
#         :param CLmin: Minimum CL (most negative CL), defaults to None
#         :type CLmin: float, optional
#         '''

#         super().__init__(cr, ct, b, sweep_lst, mac, Delta_AR)

#         if config.lower() in ['t-tail', 'canard', 'fuselage-mounted', 'fin-mounted']:
#             self.config = config
#         else:
#             self.config = None
#             raise ValueError('Tail configuration not recognised')
#         if motion_type in ['full_moving', 'adjustable', 'fixed']:
#             self.motion_type = motion_type
#         else:
#             self.motion_type = None
#             raise ValueError('Tail motion not recongised')

#         self._CLmin = CLmin

#     @property
#     def Vh_V(self):
#         self.config = self.config.lower()
#         if self.config == 't-tail' or self.config == 'canard':
#             return 1
#         elif self.config == 'fuselage-mounted':
#             return 0.85
#         elif self.config == 'fin-mounted':
#             return 0.95
#         else:
#             raise ValueError('Tail configuration not recognised')


#     @property
#     def CLmin(self):
#         if self._CLmin is None:
#             if self.motion_type == 'full_moving':
#                 return -1
#             elif self.motion_type == 'adjustable':
#                 return -0.8
#             elif self.motion_type == 'fixed':
#                 return -0.35*self.AR**(1/3)
#             else:
#                 raise ValueError('Tail motion not recongised')
#         else:
#             return self._CLmin

#     def CLalpha(self, mach, eff=0.95):
#         '''DATCOM method to estimate wing lift slope.

#         :param mach: Mach number [-]
#         :type mach: float
#         :param eff: Airfoil efficiency coefficient, defaults to 0.95
#         :type eff: float, optional
#         '''
#         return super().CLalpha(mach*self.Vh_V, eff)

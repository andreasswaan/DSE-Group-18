import numpy as np
from constants import g


class Wing:
    '''Class for a wing.'''
    def __init__(self,
                 cr,
                 ct,
                 b,
                 sweep_lst,
                 mac = None, 
                 Delta_AR = 0,
                 ):
        '''Wing class.

        :param cr: Root chord length [m]
        :type cr: float
        :param ct: Tip chord length [m]
        :type ct: float
        :param b: Span length [m]
        :type b: float
        :param sweep_lst: Combination of sweep angle [rad] and chord \
        fraction [-] in tuple (Lambda, c)
        :type sweep_lst: Tuple[float,float]
        :param mac: Assumed mean aerodynamic chord length [m], defaults to None
        :type mac: float, optional
        :param Delta_AR: Change in effective AR, as a fraction of the \
        geometric AR, defaults to 0
        :type Delta_AR: int, optional
        '''
        self.root = cr
        self.tip  = ct
        self.span = b   
        self._sweep_0 = sweep_lst
        self.mac_est = mac
        self.Delta_AR = Delta_AR

    @property
    def area(self):
        '''Wing surface [m2]'''
        return (self.root + self.tip)*self.span/2
    
    @property
    def geom_AR(self):
        '''Geometric aspect ratio [-]'''
        return self.span**2/self.area
    
    @property
    def AR(self):
        '''Effective aspect ratio [-]'''
        return self.geom_AR*(1+self.Delta_AR)

    @property
    def taper(self):
        '''Taper ratio [-]'''
        return self.tip/self.root
    
    def sweep(self, x):
        '''Sweep angle [rad] at a given chord fraction [-]'''
        sweep0 = self._sweep_0[0]
        x0     = self._sweep_0[1]

        return np.arctan(np.tan(sweep0)+(x0-x)*(2*self.root/self.span)*(1-self.taper))

    def chord(self, y):
        '''Chord length at a spanwise position y [m]'''
        return self.root + (self.tip-self.root)*y/(self.span/2)

    def y_chord(self, c):
        '''Spanwise coordinate [m] of chord of length c [m]'''
        return (self.root-c)/(self.root-self.tip) * self.span/2

    def x_chord(self, c):
        '''X-coordinate (from root LE) of chord of length c [m]'''
        return self.y_chord(c) * np.tan(self.sweep(0))
    
    @property
    def MAC(self):
        '''Mean aerodynamic chord length [m]'''
        if self.mac_est is not None:
            return self.mac_est
        else:
            # Assuming MAC = MGC
            return (2/3)*self.root*(1+self.taper+self.taper**2)/(1+self.taper)

    @property
    def yLEMAC(self):
        '''Spanwise position (y-coord) of MAC [m]'''
        return self.y_chord(self.MAC)
    
    @property
    def xLEMAC(self):
        '''Longitudinal position (x-coord) of MAC [m], relative to root LE'''
        return self.x_chord(self.MAC)
    
    def CLalpha(self, mach, eff=0.95):
        '''DATCOM method to estimate wing lift slope.

        :param mach: Mach number [-]
        :type mach: float
        :param eff: Airfoil efficiency coefficient, defaults to 0.95
        :type eff: float, optional
        '''
        beta = np.sqrt(1-mach**2)
        return 2*np.pi*self.AR/(2+np.sqrt(4+ (self.AR*beta/eff)**2*(1+np.tan(self.sweep(0.5))**2/beta**2)))
    
    def area_int(self, y0, y1):
        '''Area integrator two span positions. It computes the wing area between
        span coord y=y0 and y=y1.

        :param y0: First span coordinate [m]
        :type y0: float
        :param y1: Second span coordinate [m]
        :type y1: float
        :return: Area [m2]
        :rtype: float
        '''
        assert y0>=0 and y1>=0 and y1>y0, 'Specify correct bounds!'
        return (self.chord(y0)+self.chord(y1))/2 * (y1-y0)


class HorizontalTail(Wing):
    def __init__(self,
                cr,
                ct,
                b,
                sweep_lst,
                mac = None,
                Delta_AR = 0,
                config = 'T-tail',
                motion_type = 'full_moving',
                CLmin=None,
                ): 
        '''Horizontal tail surface class.

        :param cr: Root chord length [m]
        :type cr: float
        :param ct: Tip chord length [m]
        :type ct: float
        :param b: Span length [m]
        :type b: float
        :param sweep_lst: Combination of sweep angle [rad] and chord \
        fraction [-] in tuple (Lambda, c)
        :type sweep_lst: Tuple[float,float]
        :param mac: Assumed mean aerodynamic chord length [m], defaults to None
        :type mac: float, optional
        :param Delta_AR: Change in effective AR, as a fraction of the \
        geometric AR, defaults to 0
        :type Delta_AR: int, optional
        :param config: Tail configuration, any of 'T-tail', 'canard', \
        'fuselage-mounted' or 'fin-mounted'
        :type config: str
        :param motion_type: Tail configuration motion capabilities. Any \
        of 'full_moving', 'adjustable' and 'fixed'
        :type motion_type: str
        :param CLmin: Minimum CL (most negative CL), defaults to None
        :type CLmin: float, optional
        '''
        
        super().__init__(cr, ct, b, sweep_lst, mac, Delta_AR)
        
        if config.lower() in ['t-tail', 'canard', 'fuselage-mounted', 'fin-mounted']:
            self.config = config
        else:
            self.config = None
            raise ValueError('Tail configuration not recognised')
        if motion_type in ['full_moving', 'adjustable', 'fixed']:
            self.motion_type = motion_type
        else:
            self.motion_type = None
            raise ValueError('Tail motion not recongised')
        
        self._CLmin = CLmin
    
    @property
    def Vh_V(self):
        self.config = self.config.lower()
        if self.config == 't-tail' or self.config == 'canard':
            return 1
        elif self.config == 'fuselage-mounted':
            return 0.85
        elif self.config == 'fin-mounted':
            return 0.95
        else:
            raise ValueError('Tail configuration not recognised')


    @property
    def CLmin(self):
        if self._CLmin is None:
            if self.motion_type == 'full_moving':
                return -1
            elif self.motion_type == 'adjustable':
                return -0.8
            elif self.motion_type == 'fixed':
                return -0.35*self.AR**(1/3)
            else:
                raise ValueError('Tail motion not recongised')
        else:
            return self._CLmin

    def CLalpha(self, mach, eff=0.95):
        '''DATCOM method to estimate wing lift slope.

        :param mach: Mach number [-]
        :type mach: float
        :param eff: Airfoil efficiency coefficient, defaults to 0.95
        :type eff: float, optional
        '''
        return super().CLalpha(mach*self.Vh_V, eff)


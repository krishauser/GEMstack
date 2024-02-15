class IntelligentDriverModel:
    """An implementation of the IDM model for car-following behavior."""
    def __init__(self):
        self.v0 =  3.   # desired velocity (m/s)
        self.s0 = 5.    # minimum spacing (m)
        self.T = 5.     # desired time headway (s)
        self.a = 1.5    # acceleration (m/s^2)
        self.b = 0.5    # comfortable braking deceleration (m/s^2)
        self.delta = 4  # velocity ratio exponent (keep at 4)
    
    def set_desired_velocity(self,v0):
        self.v0 = v0
    
    def set_minimum_spacing(self,s0):
        self.s0 = s0
    
    def set_desired_time_headway(self,T):
        self.T = T
    
    def set_accelerations(self,a,b):
        self.a = a
        self.b = b

    def __eval__(self,velocity,d_lead,v_lead=0):
        """Returns the desired acceleration from the IDM model.
        
        Args:
            velocity: current vehicle velocity in m/s
            d_lead: the distance to the lead vehicle in m
            v_lead: the velocity of the lead vehicle in m/s
        
        Returns:
            The desired acceleration in m/s^2
        """
        va = velocity
        dva = velocity - v_lead
        sstar = self.s0 + va * self.T + (va * dva) / (2 * (self.a * self.b)**0.5)
        accel = self.a * (1 - (va / self.v0)**self.delta - (sstar / d_lead)**2)
        return accel


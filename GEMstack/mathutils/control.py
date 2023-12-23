class PID(object):
    """Generic SISO PID controller.
    
    Parameters:
        kp (float): proportional gain
        ki (float): integral gain
        kd (float): derivative gain
        windup_limit (optional, float): limit on the integral term. Defaults
            to None, which means no limit.
        difference_jump_threshold (optional, float): threshold used to 
            determine discontinuities between two consecutive errors. If
            the difference is greater than this threshold, the derivative
            term is set to 0. Defaults to 0.5.
    """
    def __init__(self, kp : float, ki : float, kd : float, windup_limit : float = None, difference_jump_threshold : float = 0.5):
        self.iterm  = 0.0
        self.last_t = None
        self.last_e = 0.0
        self.kp     = kp
        self.ki     = ki
        self.kd     = kd
        self.wg     = windup_limit 
        self.dg     = difference_jump_threshold
        self.derror = 0.0                          #differenced error, for debugging

    def reset(self):
        """Resets the controller to its initial state."""
        self.iterm  = 0.0
        self.last_e = 0.0
        self.last_t = None

    def advance(self,
                e : float,
                de : float = None,
                t : float = None,
                dt : float = None,
                feedforward_term : float = 0) -> float:
        """
        Parameters:
            e (float): error
            de (optional, float): error derivative. If not provided, will be
                computed via finite differences.
            t (optional, float): time. Either dt or t must be provided for D or
                I term to have any effect.
            dt (optional, float): time step. Either dt or t must be provided
                for the D or I term to have any effect.
            feedforward_term (optional, float): feedforward term to add to the
                output. Defaults to 0.

        """
        if de is None:
            if dt is None:
                if self.last_t is None:
                    self.last_t = t
                dt = (t - self.last_t)
                
            #finite differences
            if dt == 0.0:
                de = 0.0
            else:
                de = (e - self.last_e) / dt

            if abs(e - self.last_e) > self.dg:
                de = 0.0
        if dt is None:
            dt = 0

        self.iterm += e * dt

        # take care of integral winding-up
        if self.wg is not None:
            if self.iterm > self.wg:
                self.iterm = self.wg
            elif self.iterm < -self.wg:
                self.iterm = -self.wg

        self.last_e = e
        self.last_t = t
        self.derror = de

        #print(feedforward_term,self.kp,e,self.ki,self.iterm,self.kd,de)
        return feedforward_term + self.kp * e + self.ki * self.iterm + self.kd * de

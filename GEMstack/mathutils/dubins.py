

from .dynamics import Dynamics, ControlSpace
from .transforms import vector_add,vector_madd,rotate2d,normalize_angle
import numpy as np

def cmp(x,y):
    if x < y: return -1
    elif x > y: return 1
    return 0
    
class DubinsCar(Dynamics):
    """Defines a first-order Dubins car state space with x = (tx,ty,theta) 
    and u = (fwd_velocity,turnRate).

    If phi is the front wheel angle, then the turn rate is
    tan(phi)/L, where L is the wheelbase.
    """
    def __init__(self,turnRateMin=-1,turnRateMax=1):
        self.turnRateRange = [turnRateMin,turnRateMax]
        self.velocityRange = [-1.0,1.0]
    
    def stateDimension(self):
        return 3
    
    def controlDimension(self):
        return 2

    def clampControls(self,u):
        """Clamps the controls to the control range."""
        u[0] = np.clip(u[0],self.velocityRange[0],self.velocityRange[1])
        u[1] = np.clip(u[1],self.turnRateRange[0],self.turnRateRange[1])
        return u

    def derivative(self,x,u):
        """Returns x' = f(x,u). 
        
        No clamping is performed on the control inputs.
        """
        assert len(x) == 3
        assert len(u) == 2
        pos = [x[0],x[1]]
        fwd = [np.cos(x[2]),np.sin(x[2])]
        right = [-fwd[1],fwd[0]]
        phi = u[1]
        d = u[0]
        return np.array([fwd[0]*d,fwd[1]*d,phi*d])
        

class DubinsCarIntegrator(ControlSpace):
    """A ControlSpace that integrates a DubinsCar by duration T and integration
    timestep dt."""
    def __init__(self,dubins,T=1,dt=1e-2):
        self.dubins = dubins
        self.T = T
        self.dt = dt

    def nextState(self,x,u):
        assert len(x) == 3
        assert len(u) == 2
        pos = [x[0],x[1]]
        fwd = [np.cos(x[2]),np.sin(x[2])]
        right = [-fwd[1],fwd[0]]
        phi = u[1]
        d = u[0]*self.T
        if abs(phi)<1e-8:
            newpos = vector_madd(pos,fwd,d)
            return np.array(newpos + [x[2]])
        else:
            #rotate about a center of rotation, with radius 1/phi
            cor = vector_madd(pos,right,1.0/phi)
            sign = cmp(d*phi,0)
            d = abs(d)
            phi = abs(phi)
            thetaMax=d*phi
            newpos = rotate2d(pos,sign*thetaMax,cor)
            return np.array(newpos + [normalize_angle(x[2]+sign*thetaMax)])


class SecondOrderDubinsCar(Dynamics):
    """Defines a second-order Dubins car state space with
    x = (tx,ty,theta,v,dtheta) and u = (fwd_accel,wheel_angle_rate).

    """
    def __init__(self,
                 wheelAngleMin=-1,wheelAngleMax=1,
                 velocityMin=-1.0,velocityMax=1.0,
                 wheelAngleRateMin=-1,wheelAngleRateMax=1,
                 accelMin=-1.0,accelMax=1.0,
                 wheelBase = 1.0):
        self.wheelAngleRange = [wheelAngleMin,wheelAngleMax]
        self.velocityRange = [velocityMin,velocityMax]
        self.wheelAngleRateRange = [wheelAngleRateMin,wheelAngleRateMax]
        self.accelRange = [accelMin,accelMax]
        self.wheelBase = wheelBase
        self.dubins = DubinsCar()
    
    def stateDimension(self):
        return 5
    
    def controlDimension(self):
        return 2

    def clampControls(self,u):
        """Clamps the controls to the control range."""
        u[0] = np.clip(u[0],self.accelRange[0],self.accelRange[1])
        u[1] = np.clip(u[1],self.wheelAngleRateRange[0],self.wheelAngleRateRange[1])
        return u

    def derivative(self,x,u):
        """Returns x' = f(x,u)"""
        assert len(x) == 5
        assert len(u) == 2
        v,phi = x[3:5]
        turn_rate = np.tan(phi)/self.wheelBase
        return np.hstack((self.dubins.derivative(x[:3],[v,turn_rate]),[u[0],u[1]]))
        
"""Functions to model the vehicle's steering / drivetrain dynamics.

TODO: calibrate drivetrain dynamics.  Power curves are not yet implemented.

TODO: add functions defining steering friction limits at different speeds and on different surfaces.
"""

from ...utils import settings
from typing import Tuple
import math

def sign(x):
    return 1 if x > 0 else -1 if x < 0 else 0

def acceleration_to_pedal_positions(acceleration : float, velocity : float, pitch : float, gear : int) -> Tuple[float,float,int]:
    """Converts acceleration in m/s^2 to pedal positions in % of pedal travel.

    Returns tuple (accelerator_pedal_position, brake_pedal_position, desired_gear)
    """
    model = settings.get('vehicle.dynamics.acceleration_model','v1_hang')
    if model == 'v1_hang':
        if gear != 1:
            print("WARNING can't handle gears other than 1 yet")
        max_accel     = settings.get('vehicle.dynamics.max_accelerator_acceleration') # m/s^2
        max_brake     = settings.get('vehicle.dynamics.max_brake_deceleration') # m/s^2
        dry_decel     = settings.get('vehicle.dynamics.internal_dry_deceleration') # m/s^2
        accel_active_range = settings.get('vehicle.dynamics.accelerator_active_range') # pedal position fraction
        brake_active_range = settings.get('vehicle.dynamics.brake_active_range') # pedal position fraction
        #linear curves
        if acceleration > -dry_decel:
            if acceleration < -dry_decel*0.5 or (acceleration <= 0 and velocity < 0.1):  # a little deadband to avoid oscillation
                throttle_percent = 0.0  #drift to stop
            else:
                throttle_percent = accel_active_range[0] + (acceleration+dry_decel)/max_accel * (accel_active_range[1]-accel_active_range[0])
            brake_percent = 0
        else:
            brake_percent = brake_active_range[0] + -(acceleration+dry_decel)/max_brake * (brake_active_range[1]-brake_active_range[0])
            throttle_percent = 0
        return (max(throttle_percent,0.0),max(brake_percent,0.0),1)
    
    elif model == 'v1_kris':
        brake_max = settings.get('vehicle.dynamics.max_brake_deceleration')
        reverse_accel_max = settings.get('vehicle.dynamics.max_accelerator_acceleration_reverse')
        accel_max = settings.get('vehicle.dynamics.max_accelerator_acceleration')
        assert isinstance(brake_max,(int,float))
        assert isinstance(reverse_accel_max,(int,float))
        assert isinstance(accel_max,list)
        assert isinstance(acceleration,(int,float))

        #compute required acceleration
        vsign = sign(velocity)
        gravity = settings.get('vehicle.dynamics.gravity')
        internal_dry_deceleration = settings.get('vehicle.dynamics.internal_dry_deceleration')
        internal_viscous_deceleration = settings.get('vehicle.dynamics.internal_viscous_deceleration')
        aerodynamic_drag_coefficient = settings.get('vehicle.dynamics.aerodynamic_drag_coefficient')
        drag = -(aerodynamic_drag_coefficient * velocity**2) * vsign - internal_dry_deceleration * vsign - internal_viscous_deceleration * velocity
        sin_pitch = math.sin(pitch)
        acceleration -= drag + gravity * sin_pitch
        #this is the net acceleration that should be achieved by accelerator / brake pedal

        #TODO: power curves to select optimal gear
        if velocity * acceleration < 0:
            accel_pos = 0
            brake_pos = -acceleration / brake_max
            if brake_pos > 1.0:
                brake_pos = 1.0
            return (accel_pos,brake_pos,gear)
        else:
            #may want to switch gear?
            if velocity == 0:
                if acceleration < 0:
                    gear = -1
                else:
                    #forward gear
                    gear = 1
            if gear < 0:
                #reverse gear
                accel_pos = -acceleration / reverse_accel_max
                brake_pos = 0
                if accel_pos > 1.0:
                    accel_pos = 1.0
                return (accel_pos,brake_pos,-1)
            elif gear > 0:
                accel_pos = acceleration / accel_max[gear]
                brake_pos = 0
                if accel_pos > 1.0:
                    accel_pos = 1.0
                return (accel_pos,brake_pos,gear)
            else:
                #stay in neutral gear
                return (0,1,0)


def pedal_positions_to_acceleration(accelerator_pedal_position : float, brake_pedal_position : float, velocity : float, pitch : float, gear : int) -> float:
    """Converts pedal positions in % of pedal travel to acceleration in m/s^2.

    Simulates drag, gravity, and internal viscous deceleration.

    Does not yet simulate velocity-dependent power.

    Returns acceleration
    """
    brake_max = settings.get('vehicle.dynamics.max_brake_deceleration')
    reverse_accel_max = settings.get('vehicle.dynamics.max_accelerator_acceleration_reverse')
    accel_max = settings.get('vehicle.dynamics.max_accelerator_acceleration')
    assert isinstance(brake_max,(int,float))
    assert isinstance(reverse_accel_max,(int,float))
    assert isinstance(accel_max,list)
    assert isinstance(accelerator_pedal_position,(int,float))
    assert isinstance(brake_pedal_position,(int,float))
    vsign = sign(velocity)
    brake_accel = -vsign * brake_max * brake_pedal_position
    if gear < 0:
        accel = - reverse_accel_max * accelerator_pedal_position
    else:
        accel = accel_max[gear] * accelerator_pedal_position
    gravity = settings.get('vehicle.dynamics.gravity')
    internal_dry_deceleration = settings.get('vehicle.dynamics.internal_dry_deceleration')
    internal_viscous_deceleration = settings.get('vehicle.dynamics.internal_viscous_deceleration')
    aerodynamic_drag_coefficient = settings.get('vehicle.dynamics.aerodynamic_drag_coefficient')
    drag = -(aerodynamic_drag_coefficient * velocity**2) * vsign - internal_dry_deceleration * vsign - internal_viscous_deceleration * velocity
    sin_pitch = math.sin(pitch)
    if velocity == 0:
        #brake accel and drag will be 0 based on velocity sign, so instead,
        #brake and dry friction may will counteract prevailing acceleration
        drag = -sign(accel - gravity*sin_pitch)*internal_dry_deceleration
        brake_accel = -sign(accel - gravity*sin_pitch) * brake_max * brake_pedal_position

        #does gravity overcome static friction from braking + dry friction?
        if abs(accel - gravity * sin_pitch) < abs(brake_accel + drag):
            return 0
        #does gravity push against the gear setting?
        if accel - gravity * sin_pitch < 0 and gear > 0:
            return 0
        if accel - gravity * sin_pitch > 0 and gear < 0:
            return 0
    return accel + brake_accel + drag - gravity * sin_pitch


def acceleration_limits(velocity : float, pitch : float, gear : int) -> Tuple[float,float]:
    """Returns the min and max achievable acceleration at the given velocity, pitch, and gear."""
    vals = []
    vals.append(pedal_positions_to_acceleration(0.0,0.0,velocity,pitch,gear))
    vals.append(pedal_positions_to_acceleration(1.0,0.0,velocity,pitch,gear))
    vals.append(pedal_positions_to_acceleration(0.0,1.0,velocity,pitch,gear))
    return min(vals),max(vals)

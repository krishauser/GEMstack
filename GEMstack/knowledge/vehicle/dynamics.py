from ...utils import settings
from typing import Tuple
import math

def acceleration_to_pedal_positions(acceleration : float, velocity : float, pitch : float, gear : int) -> Tuple[float,float,int]:
    """Converts acceleration in m/s^2 to pedal positions in % of pedal travel.

    Returns tuple (accelerator_pedal_position, brake_pedal_position, desired_gear)
    """
    brake_max = settings.get('vehicle.dynamics.max_brake_deceleration')
    reverse_accel_max = settings.get('vehicle.dynamics.max_accelerator_acceleration_reverse')
    accel_max = settings.get('vehicle.dynamics.max_accelerator_acceleration')

    #compute required acceleration
    vsign = (velocity/abs(velocity))
    gravity = settings.get('vehicle.dynamics.gravity')
    internal_viscous_deceleration = settings.get('vehicle.dynamics.internal_viscous_deceleration')
    aerodynamic_drag_coefficient = settings.get('vehicle.dynamics.aerodynamic_drag_coefficient')
    drag = -(aerodynamic_drag_coefficient * velocity**2) * vsign - internal_viscous_deceleration * velocity
    sin_pitch = math.sin(pitch)
    acceleration += drag + gravity * sin_pitch

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
    brake_accel = brake_max * brake_pedal_position
    if gear < 0:
        accel = reverse_accel_max * accelerator_pedal_position
    else:
        accel = accel_max[gear] * accelerator_pedal_position
    vsign = (velocity/abs(velocity))
    gravity = settings.get('vehicle.dynamics.gravity')
    internal_viscous_deceleration = settings.get('vehicle.dynamics.internal_viscous_deceleration')
    aerodynamic_drag_coefficient = settings.get('vehicle.dynamics.aerodynamic_drag_coefficient')
    drag = -(aerodynamic_drag_coefficient * velocity**2) * vsign - internal_viscous_deceleration * velocity
    sin_pitch = math.sin(pitch)
    if velocity == 0:
        #does gravity overcome static friction from braking?
        if abs(accel - gravity * sin_pitch) < brake_accel:
            return 0
    return accel - brake_accel - drag - gravity * sin_pitch

"""Functions to model the vehicle's steering / drivetrain dynamics.

TODO: calibrate drivetrain dynamics.  Power curves are not yet implemented.

TODO: add functions defining steering friction limits at different speeds and on different surfaces.
"""

from ...utils import settings
from typing import Tuple
import math

def sign(x):
    return 1 if x > 0 else -1 if x < 0 else 0
def convert_RPM(vel):
    wheel_rad = settings.get('vehicle.dynamics.wheel_rad',0.55)
    return (vel *60 )/ (2 * math.pi*wheel_rad)

def calculate_drag(velocity: float, rho: float, front_area: float, coeff: float, sign_velocity: int) -> float:
    """Calculate aerodynamic and internal drag.
    
    parameters:
    - velocity: Vehicle's current speed in m/s.
    - rho: Air density in kg/m^3.
    - front_area: Frontal area of the vehicle in m^2.
    - coeff: Drag coefficient of the vehicle.
    - sign_velocity: Sign of the velocity to determine the direction of drag.

    Returns:
    - Drag force in N.
    
    """
    internal_dry_deceleration = settings.get('vehicle.dynamics.internal_dry_deceleration')
    internal_viscous_deceleration = settings.get('vehicle.dynamics.internal_viscous_deceleration')
    return 0.5 * rho * front_area * coeff * velocity**2 * sign_velocity + internal_dry_deceleration * sign_velocity + internal_viscous_deceleration * velocity

def calculate_tire_forces(velocity: float, acceleration: float, pitch: float, vehicle_height: float, vehicle_mass: float) -> Tuple[float, float]:
    """
    Calculate the normal forces on the front and rear tires based on vehicle dynamics.

    Parameters:
    - velocity: Vehicle's current speed in m/s.
    - acceleration: Current acceleration in m/s^2.
    - pitch: The pitch angle of the vehicle in radians.
    - vehicle_height: Height of the vehicle's center of mass from the ground in meters.
    - vehicle_mass: Mass of the vehicle in kg.

    Returns:
    - Tuple containing the normal force on the front tire (front_normal_tire_f)
      and the normal force on the rear tire (rear_normal_tire_r).
    """
    # Constants and settings
    gravity = settings.get('vehicle.dynamics.gravity')
    rho = settings.get('vehicle.dynamics.air_density')
    front_area = settings.get('vehicle.dynamics.front_area')
    aerodynamic_drag_coefficient = settings.get('vehicle.dynamics.aerodynamic_drag_coefficient')
    lf = settings.get('vehicle.dynamics.lf')
    lr = settings.get('vehicle.dynamics.lr')

    # Sign of the velocity to determine the direction of drag
    sign_velocity = sign(velocity)
    
    # Calculate drag force
    drag = calculate_drag(velocity, rho, front_area, aerodynamic_drag_coefficient, sign_velocity)

    # Trigonometric functions for pitch
    sin_pitch = math.sin(pitch)
    cos_pitch = math.cos(pitch)

    # Normal forces calculations
    front_normal_tire_f = (-drag - vehicle_mass * acceleration * vehicle_height - vehicle_mass * gravity * sin_pitch + vehicle_height * gravity * lr * cos_pitch) / (lf + lr)
    rear_normal_tire_r  = (drag + vehicle_mass * acceleration * vehicle_height + vehicle_mass * gravity * sin_pitch + vehicle_height * gravity * lf * cos_pitch) / (lf + lr)

    return front_normal_tire_f, rear_normal_tire_r

def acceleration_to_pedal_positions(acceleration : float, velocity : float, pitch : float, gear : int) -> Tuple[float,float,int]:
    """Converts acceleration in m/s^2 to pedal positions in % of pedal travel.

    Returns tuple (accelerator_pedal_position, brake_pedal_position, desired_gear)
    """
    model = settings.get('vehicle.dynamics.acceleration_model','hang_v1')
    
    if model == 'hang_v1':
        if gear != 1:
            print("WARNING can't handle gears other than 1 yet")
        max_accel     = settings.get('vehicle.dynamics.max_accelerator_acceleration')[1] # m/s^2

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
    
    elif model == 'group8_v1':
        brake_maximum = settings.get('vehicle.dynamics.max_brake_deceleration')
        reverse_accel_max = settings.get('vehicle.dynamics.max_accelerator_acceleration_reverse')
        accel_max = settings.get('vehicle.dynamics.max_accelerator_acceleration')
        wheel_rad = settings.get('vehicle.dynamics.wheel_rad')
        lf = settings.get('vehicle.dynamics.lf')
        lr = settings.get('vehicle.dynamics.lr')
        assert isinstance(brake_maximum,(int,float))
        assert isinstance(reverse_accel_max,(int,float))
        assert isinstance(accel_max,list)
        assert isinstance(acceleration,(int,float))

        #compute required acceleration
        vsign = sign(velocity)
        gravity = settings.get('vehicle.dynamics.gravity')
        internal_dry_deceleration = settings.get('vehicle.dynamics.internal_dry_deceleration')
        internal_viscous_deceleration = settings.get('vehicle.dynamics.internal_viscous_deceleration')
        aerodynamic_drag_coefficient = settings.get('vehicle.dynamics.aerodynamic_drag_coefficient')
        accel_active_range = settings.get('vehicle.dynamics.accelerator_active_range') # pedal position fraction
        brake_active_range = settings.get('vehicle.dynamics.brake_active_range') # pedal position fraction
        acceleration_deadband = settings.get('vehicle.dynamics.acceleration_deadband',0.0)
        vehicle_mass = settings.get('vehicle.dynamics.mass')
        vehicle_height = settings.get('vehicle.dynamics.vehicle_height')
        dry_decel     = settings.get('vehicle.dynamics.internal_dry_deceleration') # m/s^2
        # rolling resistance coefficient
        # rolling_resistance_f = settings.get('vehicle.dynamics.rolling_resistance_f') # N
        # rolling_resistance_r = settings.get('vehicle.dynamics.rolling_resistance_r') # N
        
        # do the offset of the acceleration drag force to compensate real acceleration
        front_area = settings.get('vehicle.dynamics.front_area')

        rho = settings.get('vehicle.dynamics.air_density')
        air_drag = calculate_drag(velocity,rho,front_area,aerodynamic_drag_coefficient,vsign)

        sin_pitch = math.sin(pitch)
        cos_pitch = math.cos(pitch)
        # reference : https://link.springer.com/book/10.1007/978-1-4614-1433-9
        # by Rajesh Rajamani
        # page 119 

        front_normal_tire_f = calculate_tire_forces(velocity,acceleration,pitch,vehicle_height,vehicle_mass)[0]
        rear_normal_tire_r  = calculate_tire_forces(velocity,acceleration,pitch,vehicle_height,vehicle_mass)[1]
        
        total_rolling_resistance = (front_normal_tire_f  + rear_normal_tire_r )/ vehicle_mass
        acceleration += air_drag  + gravity * sin_pitch + total_rolling_resistance


        if acceleration > -dry_decel:
            # safe to accelerate
            # if acceleration is between 0 and dry_decel, we can assume that we are in a deadband
            if acceleration < 0:
                # deadband to stop
                # change current gear to neutral 
                gear = 0
                return (0,0,gear)
            else:
                # do forward
                accel_pos = acceleration / accel_max[gear]
                accel_pos = min(1.0,max(accel_pos,0.0))
                brake_pos = 0
                return (accel_active_range[0] + accel_pos*(accel_active_range[1]-accel_active_range[0]),brake_pos,gear)

        else:
            # in this case, acceleration is negative
            # we need to brake
            if velocity > 0 :
                # need to brake and make velocity to 0
                accel_pos = 0
                brake_pos = -acceleration / brake_maximum
                brake_pos = min(1.0,max(brake_pos,0.0))
                return (accel_pos,brake_active_range[0] + brake_pos*(brake_active_range[1]-brake_active_range[0]),gear)
            elif velocity <= 0:
                # do reverse  
                gear = -1
                accel_pos = -acceleration / reverse_accel_max
                brake_pos = 0
                accel_pos = min(1.0,max(accel_pos,0.0))
                return (accel_active_range[0] + accel_pos*(accel_active_range[1]-accel_active_range[0]),brake_pos,gear)
            else:
                # stay in neutral gear, brake
                return (0,1,0)


        
    elif model == 'kris_v1':
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
        accel_active_range = settings.get('vehicle.dynamics.accelerator_active_range') # pedal position fraction
        brake_active_range = settings.get('vehicle.dynamics.brake_active_range') # pedal position fraction
        acceleration_deadband = settings.get('vehicle.dynamics.acceleration_deadband',0.0)

        drag = -(aerodynamic_drag_coefficient * velocity**2) * vsign - internal_dry_deceleration * vsign - internal_viscous_deceleration * velocity
        sin_pitch = math.sin(pitch)
        acceleration -= drag + gravity * sin_pitch
        #this is the net acceleration that should be achieved by accelerator / brake pedal

        #TODO: power curves to select optimal gear
        if abs(acceleration) < acceleration_deadband:
            #deadband?
            return (0,0,gear)
        if velocity * acceleration < 0:
            accel_pos = 0
            brake_pos = -acceleration / brake_max
            if brake_pos > 1.0:
                brake_pos = 1.0
            return (accel_pos,brake_active_range[0] + brake_pos*(brake_active_range[1]-brake_active_range[0]),gear)
        
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
                return (accel_active_range[0] + accel_pos*(accel_active_range[1]-accel_active_range[0]),brake_pos,-1)
            elif gear > 0:
                accel_pos = acceleration / accel_max[gear]
                brake_pos = 0
                if accel_pos > 1.0:
                    accel_pos = 1.0
                return (accel_active_range[0] + accel_pos*(accel_active_range[1]-accel_active_range[0]),brake_pos,gear)
            else:
                #stay in neutral gear, brake
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
    accel_active_range = settings.get('vehicle.dynamics.accelerator_active_range') # pedal position fraction
    brake_active_range = settings.get('vehicle.dynamics.brake_active_range') # pedal position fraction
    vehicle_mass = settings.get('vehicle.dynamics.mass')
    internal_viscous_deceleration = settings.get('vehicle.dynamics.internal_viscous_deceleration')

    lr = settings.get('vehicle.dynamics.lr')
    lf = settings.get('vehicle.dynamics.lf')
    vehicle_height = settings.get('vehicle.dynamics.vehicle_height')
    #normalize to 0-1 depending on pedal range
    accelerator_pedal_position = (accelerator_pedal_position - accel_active_range[0]) / (accel_active_range[1]-accel_active_range[0])
    accelerator_pedal_position = min(1.0,max(accelerator_pedal_position,0.0))
    brake_pedal_position = (brake_pedal_position - brake_active_range[0]) / (brake_active_range[1]-brake_active_range[0])
    brake_pedal_position = min(1.0,max(brake_pedal_position,0.0))
    # rolling resistance coefficient
    # rolling_resistance_f = settings.get('vehicle.dynamics.rolling_resistance_f') # N
    # rolling_resistance_r = settings.get('vehicle.dynamics.rolling_resistance_r') # N
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
    aerodynamic_drag_coefficient = settings.get('vehicle.dynamics.aerodynamic_drag_coefficient')
    # align the dynamics with acceleration to pelda position
    # do the offset of the acceleration drag force to compensate real acceleration
    front_area = settings.get('vehicle.dynamics.front_area')

    rho = settings.get('vehicle.dynamics.air_density')
    drag = - calculate_drag(velocity,rho,front_area,aerodynamic_drag_coefficient,vsign)

    sin_pitch = math.sin(pitch)
    cos_pitch = math.cos(pitch)

    front_normal_tire_f = calculate_tire_forces(velocity,accel,pitch,vehicle_height,vehicle_mass)[0]
    rear_normal_tire_r  = calculate_tire_forces(velocity,accel,pitch,vehicle_height,vehicle_mass)[1]
    
    total_rolling_resistance = (front_normal_tire_f  + rear_normal_tire_r )/ vehicle_mass

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
    # align with acceleration to pedal position dynamics
    return accel + brake_accel + drag - gravity * sin_pitch - total_rolling_resistance


def acceleration_limits(velocity : float, pitch : float, gear : int) -> Tuple[float,float]:
    """Returns the min and max achievable acceleration at the given velocity, pitch, and gear."""
    vals = []
    vals.append(pedal_positions_to_acceleration(0.0,0.0,velocity,pitch,gear))
    vals.append(pedal_positions_to_acceleration(1.0,0.0,velocity,pitch,gear))
    vals.append(pedal_positions_to_acceleration(0.0,1.0,velocity,pitch,gear))
    return min(vals),max(vals)
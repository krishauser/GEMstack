"""Functions to model the vehicle's steering geometry.

TODO: double-check steer2front and front2steer coefficients.
"""

from ...utils import settings
import math

def clamp_wheel_angle(wheel_angle : float) -> float:
    """Clamps the wheel angle to the range defined in the settings."""
    return max(settings.get('vehicle.limits.wheel_angle_min'), min(settings.get('vehicle.limits.wheel_angle_max'), wheel_angle))

def clamp_steering_angle(wheel_angle : float) -> float:
    """Clamps the steering angle to the range defined in the settings."""
    return max(settings.get('vehicle.limits.steering_angle_min'), min(settings.get('vehicle.limits.steering_angle_max'), wheel_angle))

def front2steer_degrees(f_angle : float) -> float:
    """Conversion of front wheel to steering wheel angle.
    Both input and output are in degrees.

    No clamping is done: the caller should clamp the input to
    the steering wheel range defined in the settings.
    """
    if (f_angle > 0):
        steer_angle = -0.1084*f_angle**2 + 21.775*f_angle
    else:
        steer_angle = 0.1084*f_angle**2 + 21.775*f_angle
    return steer_angle

def front2steer(f_angle : float):
    """Conversion of front wheel to steering wheel angle.
    Both input and output are in radians."""
    return math.radians(front2steer_degrees(math.degrees(f_angle)))

# y = a x**2 + b x  =>  0 = a x**2 + b x - y 
# => x = (-b +- sqrt(b**2 - 4 a (-y))) / (2 a)
# since we need the positive solution, we can ignore the negative one

def steer2front_degrees(s_angle : float) -> float:
    """Conversion of steering wheel to front wheel angle.
    Both input and output are in degrees.

    No clamping is done: the caller should clamp the input to
    the steering wheel range defined in the settings.
    """
    if (s_angle > 0):
        a = -0.1084
        b = 21.775
        front_angle = (-b + math.sqrt(b**2 - 4*a*(-s_angle))) / (2*a)
    else:
        a = 0.1084
        b = 21.775
        front_angle = (-b + math.sqrt(b**2 - 4*a*(-s_angle))) / (2*a)
    return front_angle

def steer2front(s_angle : float):
    """Conversion of front wheel to steering wheel angle.
    Both input and output are in radians."""
    return math.radians(steer2front_degrees(math.degrees(s_angle)))

def heading_rate(front_angle : float, speed : float, wheelbase : float) -> float:
    """Returns the heading rate in radians/sec given the front wheel angle, speed, and wheelbase."""
    return math.tan(front_angle) * speed / wheelbase
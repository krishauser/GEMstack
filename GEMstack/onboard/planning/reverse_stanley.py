import numpy as np
from math import sin, cos, atan2

from ...mathutils.control import PID
from ...utils import settings
from ...mathutils import transforms
from ...knowledge.vehicle.geometry import front2steer
from ...knowledge.vehicle.dynamics import acceleration_to_pedal_positions
from ...state.vehicle import VehicleState, ObjectFrameEnum
from ...state.trajectory import Path, Trajectory
from ..interface.gem import GEMVehicleCommand
from ..component import Component

#####################################
# 1. Angle normalization
#####################################
def normalise_angle(angle):
    """Wraps an angle to the [-pi, pi] range."""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

#####################################
# 2. Reverse-only Stanley controller with longitudinal PID
#####################################
class Stanley(object):
	"""
	A Stanley controller modified to ONLY perform reverse path tracking.
	It uses the rear axle as the control reference and calculates
	commands for backward motion.
	"""

	def __init__(
		self,
		control_gain=None,
		softening_gain=None,
		desired_speed=None # This will be interpreted as desired speed magnitude
	):
		"""
		:param control_gain: Stanley lateral control gain k.
		:param softening_gain: Softening gain k_soft.
		:param desired_speed: Desired speed magnitude (float) or 'path'/'trajectory'.
							   The controller will always target a negative speed.
		"""

		self.k = control_gain if control_gain is not None else settings.get('control.stanley.control_gain')
		self.k_soft = softening_gain if softening_gain is not None else settings.get('control.stanley.softening_gain')

		self.max_steer = settings.get('vehicle.geometry.max_wheel_angle')
		self.wheelbase = settings.get('vehicle.geometry.wheelbase')

		self.speed_limit = settings.get('vehicle.limits.max_speed')
		self.reverse_speed_limit = settings.get('vehicle.limits.max_reverse_speed') # add for reve
		self.max_accel 	 = settings.get('vehicle.limits.max_acceleration') # Max positive acceleration magnitude
		self.max_decel 	 = settings.get('vehicle.limits.max_deceleration') # Max deceleration magnitude (used for max n	egative accel)

		# PID for longitudinal speed tracking
		p = settings.get('control.longitudinal_control.pid_p')
		d = settings.get('control.longitudinal_control.pid_d')
		i = settings.get('control.longitudinal_control.pid_i')
		self.pid_speed = PID(p, d, i, windup_limit=20)

		# Speed source: numeric or derived from path/trajectory
		if desired_speed is not None:
			self.desired_speed_source = -abs(desired_speed)
		else:
			self.desired_speed_source = settings.get('control.stanley.desired_speed', 'path')

		if isinstance(self.desired_speed_source, (int, float)):
			self.desired_speed_magnitude = self.desired_speed_source
		else:
			self.desired_speed_magnitude = None


		# For path/trajectory
		self.path_arg = None
		self.path = None
		self.trajectory = None
		self.current_path_parameter = 0.0
		self.current_traj_parameter = 0.0
		self.t_last = None

	def set_path(self, path: Path):
		"""Sets the path or trajectory to track."""
		if path == self.path_arg:
			return
		self.path_arg = path

        # If the path has more than 2D, reduce to (x,y)
		if len(path.points[0]) > 2:
			path = path.get_dims([0,1])

		# If no timing info, cannot use 'path'/'trajectory' speed
		if not isinstance(path, Trajectory):
			self.path = path.arc_length_parameterize()
			self.trajectory = None
			self.current_traj_parameter = 0.0
			# Error if speed source requires trajectory but none provided
			if self.desired_speed_source in ['path', 'trajectory']:
				raise ValueError("Stanley: Provided path has no timing. Either provide a Trajectory or set a constant desired_speed.")
		else:
			self.path = path.arc_length_parameterize() # Parameterize by arc length for lateral tracking
			self.trajectory = path # Store the original trajectory object
			self.current_traj_parameter = self.trajectory.domain()[0]

		# Reset path parameter to the start of the parameterized path
		self.current_path_parameter = self.path.domain()[0]


	# Helper to find rear axle position (Front axle not needed in reverse-only)
	def _find_rear_axle_position(self, x, y, yaw):
		"""Compute rear-axle world position from the vehicle's reference point and yaw."""
		rx = x - self.wheelbase * cos(yaw)
		ry = y - self.wheelbase * sin(yaw)
		return rx, ry


	# compute method hardcoded for reverse
	def compute(self, state: VehicleState, component: Component = None):
		"""Compute the control outputs for reverse tracking:
		   (longitudinal acceleration, front wheel angle).

		   :param state: Current vehicle state.
		   :param component: Optional component for debugging.
		   # No 'reverse' parameter here, always reverse
		"""
		t = state.pose.t
		if self.t_last is None:
			self.t_last = t
			dt = 0 # Initialize dt to 0 on first iteration
		else:
			dt = t - self.t_last

		# Current vehicle states
		curr_x = state.pose.x
		curr_y = state.pose.y
		curr_yaw = state.pose.yaw if state.pose.yaw is not None else 0.0
		# Use signed longitudinal velocity. Assuming state.v is signed.
		velocity = state.v


		# Must have a trajectory to track in ReverseStanley if speed source is 'path'/'trajectory'
		# or if we need path domain/evaluation. Let's assume Path is always needed.
		if self.path is None:
			if component:
				component.debug_event("No path provided to ReverseStanley controller. Doing nothing.")
			self.t_last = t # Update t_last even if doing nothing
			self.pid_speed.reset() # Reset PID if path is lost
			return (0.0, 0.0)

		# Ensure same frame
		if self.path.frame != state.pose.frame:
			if component:
				component.debug_event(f"Transforming path from {self.path.frame.name} to {state.pose.frame.name}")
			# Use transforms utility correctly
			self.path = transforms.transform_path(self.path, self.path.frame, state.pose.frame, current_pose=state.pose)

		# Must transform trajectory if using trajectory speed source
		if self.trajectory and self.trajectory.frame != state.pose.frame:
			if component:
				component.debug_event(f"Transforming trajectory from {self.trajectory.frame.name} to {state.pose.frame.name}")
			# Use transforms utility correctly for trajectory
			self.trajectory = transforms.transform_path(self.trajectory, self.trajectory.frame, state.pose.frame, current_pose=state.pose)


		# Reverse always uses the rear axle position as the reference
		ref_x, ref_y = self._find_rear_axle_position(curr_x, curr_y, curr_yaw)


		# 1) Closest point on the arc-length parameterized path
		# Find closest point relative to the rear axle
		# Search range around the current path parameter.
		search_start = self.current_path_parameter - 5.0 # Search range behind current parameter
		search_end 	 = self.current_path_parameter + 5.0 # Search range ahead of current parameter

		# Pass ref_x, ref_y to closest_point_local
		closest_dist, closest_parameter = self.path.closest_point_local((ref_x, ref_y), [search_start, search_end])

		# Update path parameter based on closest point.
		# Note: This simple update might not be ideal for transitions between forward/reverse segments
		# if using a trajectory with mixed gears, but it works for full-path reverse.
		self.current_path_parameter = closest_parameter


		# 2) Path heading
		target_x, target_y = self.path.eval(self.current_path_parameter) # Use updated parameter
		tangent = self.path.eval_tangent(self.current_path_parameter) # Use updated parameter
		path_yaw = atan2(tangent[1], tangent[0])

		# 3) Lateral error calculation
		# Calculate cross-track error from the reference point (rear axle) to the path.
		dx = ref_x - target_x
		dy = ref_y - target_y
		# Calculate signed cross-track error relative to path tangent
		# This gives signed distance to the "left" of the path tangent
		cross_track_error = dx * (-tangent[1]) + dy * tangent[0]


		# 4) Heading error
		# The heading error is the difference between the path tangent's orientation
		# and the vehicle's current orientation.
		yaw_error = normalise_angle(path_yaw - curr_yaw)

		# 5) Stanley lateral control terms (hardcoded for reverse)
		# For reverse, heading error term is typically inverted
		heading_term = -yaw_error
		# For reverse, cross-track error term's effect is inverted.
		# Negate the cross_track_error input to atan2.
		# Use absolute velocity in the denominator. Avoid division by small velocity.
		cross_term_input = self.k * (-cross_track_error) / (self.k_soft + abs(velocity))
		cross_term = atan2(cross_term_input, 1.0) # atan2(y, x) gives angle of vector (x, y)

		# The final steering angle is the sum of adjusted terms
		desired_steering_angle = heading_term + cross_term

		# Longitudinal Control - Hardcoded for reverse (negative speed)
		desired_speed_magnitude = self.desired_speed_magnitude # Start with base desired speed magnitude
		feedforward_accel = 0.0 # Initialize feedforward

		# If speed source is trajectory, get magnitude from trajectory
		if self.desired_speed_source in ['path', 'trajectory']:
			# Get speed magnitude from trajectory at current parameter
			# Assuming trajectory.eval_derivative(s) returns velocity vector at path parameter 's'
			# or we map s to trajectory's native parameter (like time) if needed
			current_trajectory_param_for_eval = self.current_path_parameter # Start with s
			if hasattr(self.trajectory, 'parameter_to_time'): # Check if time mapping is possible
				current_trajectory_param_for_eval = self.trajectory.parameter_to_time(self.current_path_parameter)
				# Fallback to using path parameter 's' if time mapping fails is removed

			# Evaluate derivative (assumed to return velocity vector along path)
			if self.trajectory is not None: # Ensure trajectory object exists
				deriv = self.trajectory.eval_derivative(current_trajectory_param_for_eval)
				path_speed_magnitude = np.linalg.norm(deriv) # Get speed magnitude
			else:
				path_speed_magnitude = 0.0 # Default if trajectory object somehow missing

			# Set desired speed magnitude, clipped by limit
			desired_speed_magnitude = min(path_speed_magnitude, self.reverse_speed_limit)


		# Apply direction sign (always negative for ReverseStanley)
		desired_speed = desired_speed_magnitude * -1.0


		# Check for reaching the START of the path (for stopping in reverse)
		is_at_start_backward = self.current_path_parameter <= self.path.domain()[0] + 1e-3 # Close to start parameter

		if is_at_start_backward:
			 # Reached the start of the path in reverse -> stop
			# if component:
			# 	component.debug_event(f"ReverseStanley: Reached start of path, stopping.")
			desired_speed = 0.0
			# Apply braking force when moving negatively (positive acceleration)
			feedforward_accel = -2.0 * -1.0 # = +2.0

		else:
			# Calculate feedforward acceleration based on speed change (still assuming magnitude from trajectory)
			difference_dt = 0.1
			# Estimate future path parameter based on current speed magnitude and reverse direction
			current_speed_abs = abs(velocity) # Use current vehicle speed magnitude
			# Avoid division by near zero when estimating future parameter distance
			if current_speed_abs < 0.1: estimated_path_step = 0.1 * difference_dt * -1.0 # Fixed step backwards
			else: estimated_path_step = current_speed_abs * difference_dt * -1.0 # Step backwards based on speed

			future_parameter = self.current_path_parameter + estimated_path_step # Move along path towards start

			# Clip future parameter to path domain (ensure it doesn't go before start)
			future_parameter = np.clip(future_parameter, self.path.domain()[0], self.path.domain()[-1])

			# Need to map future_parameter (s) back to trajectory's native parameter if necessary
			future_trajectory_param_for_eval = future_parameter # Start with s
			if hasattr(self.trajectory, 'parameter_to_time'): # Check if time mapping is possible
				future_trajectory_param_for_eval = self.trajectory.parameter_to_time(future_parameter)
				# Fallback to using path parameter 's' if time mapping fails is removed

			# Evaluate derivative at future point (assumed to return velocity vector along path)
			if self.trajectory is not None: # Ensure trajectory object exists
				future_deriv = self.trajectory.eval_derivative(future_trajectory_param_for_eval)
				next_path_speed_magnitude = min(np.linalg.norm(future_deriv), self.speed_limit)
			else:
				next_path_speed_magnitude = 0.0 # Default if trajectory object somehow missing

			# Apply direction sign (always negative)
			next_desired_speed = next_path_speed_magnitude * -1.0

			# Estimate acceleration = delta_v / delta_t
			# Use the fixed difference_dt for acceleration calculation
			feedforward_accel = (next_desired_speed - desired_speed) / difference_dt
			feedforward_accel = np.clip(feedforward_accel, -self.max_decel, self.max_accel)


		# Cross-track-based slowdown applied to speed magnitude (uses abs error)
		desired_speed_magnitude_slowed = abs(desired_speed) * np.exp(-abs(cross_track_error) * 0.6)

		# Reapply direction sign (always negative)
		desired_speed = desired_speed_magnitude_slowed * -1.0 if desired_speed_magnitude_slowed != 0 else 0.0


		# Clip to speed limit (magnitude)
		if abs(desired_speed) > self.speed_limit:
			desired_speed = self.speed_limit * -1.0 if desired_speed != 0 else 0.0


		# PID for longitudinal control
		# Use signed velocity for error calculation
		speed_error = desired_speed - velocity
		output_accel = self.pid_speed.advance(e=speed_error, t=t, feedforward_term=feedforward_accel)

		# Clip acceleration
		# output_accel can be positive (forward accel / reverse brake) or negative (forward brake / reverse accel)
		if output_accel > self.max_accel: # Max positive accel
			output_accel = self.max_accel
		elif output_accel < -self.max_decel: # Max negative accel (corresponding to max decel magnitude)
			output_accel = -self.max_decel


		# Avoid acceleration from near zero if desired speed is zero
		# This prevents "creep" when stopped.
		if desired_speed == 0 and abs(velocity) < 0.05: # Use small epsilon for near zero velocity
			 # Check if output_accel would increase speed magnitude away from zero
			 # If current velocity > 0, positive accel increases speed. If velocity < 0, negative accel increases speed (makes it more negative).
			 # If velocity is 0, any non-zero accel would move away from zero.
			 if (velocity * output_accel > 0) or (abs(velocity) < 1e-3 and output_accel != 0): # Use smaller epsilon for the zero check
				 output_accel = 0.0




		# Debug
		if component is not None:
			component.debug('curr pt',(curr_x,curr_y))
			component.debug("   ", self.current_path_parameter)
			component.debug("crosstrack error", cross_track_error)
			component.debug("crosstrack dist", closest_dist)
			component.debug("yaw_error", yaw_error)
			component.debug('steering_angle', desired_steering_angle)
			component.debug("desired_speed (m/s)", desired_speed)
			component.debug("feedforward_accel (m/s^2)", feedforward_accel)
			component.debug(" output_accel (m/s^2)", output_accel)
	
		self.t_last = t
		# Return signed acceleration and desired front wheel steering angle
		return (output_accel, desired_steering_angle)

#####################################
# 3. Tracker component
#####################################
class StanleyTrajectoryTracker(Component):
    """
    A trajectory-tracking Component that uses the above Stanley controller
    for lateral control plus PID-based longitudinal control.
    It now includes measures to mitigate oscillations.
    """

    def __init__(self, vehicle_interface=None, **kwargs):
        """
        :param vehicle_interface: The low-level interface to send commands to the vehicle.
        :param kwargs: Optional parameters to pass into the Stanley(...) constructor.
        """
        self.stanley = Stanley(**kwargs)
        self.vehicle_interface = vehicle_interface

    def rate(self):
        """Control frequency in Hz."""
        return 50.0

    def state_inputs(self):
        """
        Required state inputs:
        - Vehicle state
        - Trajectory
        """
        return ["vehicle", "trajectory"]

    def state_outputs(self):
        """No direct output state here."""
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        """
        Per control cycle:
          1) Set the path/trajectory
          2) Compute (acceleration, front wheel angle)
          3) Convert front wheel angle to steering wheel angle (if necessary)
          4) Send command to the vehicle
        """
        self.stanley.set_path(trajectory)
        accel, f_delta = self.stanley.compute(vehicle, self)

        # If your low-level interface expects steering wheel angle:
        steering_angle = front2steer(f_delta)
        steering_angle = np.clip(
            steering_angle,
            settings.get('vehicle.geometry.min_steering_angle', -0.5),
            settings.get('vehicle.geometry.max_steering_angle',  0.5)
        )

        cmd = self.vehicle_interface.simple_command(accel, steering_angle, vehicle)
        self.vehicle_interface.send_command(cmd)

    def healthy(self):
        """Optional: check if the controller has a valid path."""
        return self.stanley.path is not None

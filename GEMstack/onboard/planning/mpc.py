from ...utils import settings
from ...state.vehicle import VehicleState,ObjectFrameEnum
from ...state.trajectory import Trajectory
from ...knowledge.vehicle.geometry import front2steer
from ..component import Component
import numpy as np
import casadi

class MPCController(object):
    """Model Predictive Controller for trajectory tracking."""
    def __init__(self):
        self.T = settings.get('control.mpc.horizon', 10)  # Prediction horizon
        self.dt = settings.get('control.mpc.dt', 0.1)  # Time step
        self.L = settings.get('vehicle.geometry.wheelbase')
        self.v_bounds = [-settings.get('vehicle.limits.max_reverse_speed'), settings.get('vehicle.limits.max_speed')]
        self.delta_bounds = [settings.get('vehicle.geometry.min_wheel_angle'),settings.get('vehicle.geometry.max_wheel_angle')]
        self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')]
        self.a_bounds = [-settings.get('vehicle.limits.max_deceleration'), settings.get('vehicle.limits.max_acceleration')]
        self.trajectory = None 
        self.prev_x = None  # Previous state trajectory
        self.prev_u = None  # Previous control inputs

    def set_path(self, trajectory: Trajectory):
        """Set the trajectory for the MPC controller."""
        # Assume the argument can only be trajectory
        assert isinstance(trajectory, Trajectory), "Invalid trajectory type."
        self.trajectory = trajectory

    def compute(self, state: VehicleState, component: Component = None):
        """Compute the control commands using MPC."""
        assert self.trajectory is not None
        assert state.pose.frame != ObjectFrameEnum.CURRENT

        x0 = np.array([state.pose.x, state.pose.y, state.pose.yaw, state.v])

       # Interpolate trajectory points to match MPC time horizon
        times = self.trajectory.times
        points = self.trajectory.points
        traj_points = []
        j = 0
        for i in range(self.T + 1):
            t_query = times[0] + i * self.dt
            if t_query <= times[0]:
                traj_points.append(points[0])
            elif t_query >= times[-1]:
                traj_points.append(points[-1])
            else:
                while j < len(times) - 2 and times[j+1] < t_query:
                    j += 1
                alpha = (t_query - times[j]) / (times[j+1] - times[j])
                pt = (1 - alpha) * np.array(points[j]) + alpha * np.array(points[j+1])
                traj_points.append(pt)

        # Optimization setup
        opti = casadi.Opti()
        x = opti.variable(self.T+1, 4)  # [x, y, theta, v]
        u = opti.variable(self.T, 2)    # [a, delta]

        def model(x, u):
            """Dynamic model of the vehicle using kinematic bicycle model"""
            px, py, theta, v = x[0], x[1], x[2], x[3]
            a, delta = u[0], u[1]
            beta = casadi.atan(casadi.tan(delta) / 2.0)
            dx = v * casadi.cos(theta + beta)
            dy = v * casadi.sin(theta + beta)
            dtheta = v * casadi.tan(delta) / self.L
            dv = a
            return casadi.vertcat(dx, dy, dtheta, dv)

        obj = 0
        for t in range(self.T):
            # Vehicle dynamics
            x_next = x[t,:] + self.dt * model(x[t,:], u[t,:]).T
            opti.subject_to(x[t+1,:] == x_next)
            target = traj_points[t+1]
            # Cost function
            obj += casadi.sumsqr(x[t+1,0:2] - casadi.reshape(target[0:2], 1, 2))
            obj += 0.1 * casadi.sumsqr(u[t,:])

        # Initial condition
        opti.subject_to(x[0, :] == casadi.reshape(x0[:4], 1, 4))

        # Constraints
        for t in range(self.T):
            opti.subject_to(opti.bounded(self.a_bounds[0], u[t,0], self.a_bounds[1]))
            opti.subject_to(opti.bounded(self.delta_bounds[0], u[t,1], self.delta_bounds[1]))
            opti.subject_to(opti.bounded(self.v_bounds[0], x[t+1,3], self.v_bounds[1]))

        # Initial guess
        if self.prev_x is not None and self.prev_u is not None:
            if len(self.prev_x) == self.T+1 and len(self.prev_u) == self.T:
                opti.set_initial(x, np.vstack((self.prev_x[1:], self.prev_x[-1])))
                opti.set_initial(u, np.vstack((self.prev_u[1:], self.prev_u[-1])))

        # Solver settings
        opti.minimize(obj)
        opti.solver("ipopt", {"expand": True}, {"max_iter": 100})

        try:
            # Solve the optimization problem
            sol = opti.solve()
            acc = float(sol.value(u[0,0]))
            delta = float(sol.value(u[0,1]))
            self.prev_x = sol.value(x)
            self.prev_u = sol.value(u)
            return acc, delta
        except RuntimeError:
            # Handle optimization failure
            print("MPC optimization failed.")
            return 0.0, 0.0


class MPCTrajectoryTracker(Component):
    def __init__(self, vehicle_interface=None, **args):
        self.mpc = MPCController(**args)
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 10.0

    def state_inputs(self):
        return ['vehicle', 'trajectory']

    def state_outputs(self):
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        self.mpc.set_path(trajectory)
        accel, delta = self.mpc.compute(vehicle, self)
        
        # Clip acceleration and steering angle to vehicle limits
        accel = np.clip(accel, self.mpc.a_bounds[0], self.mpc.a_bounds[1])
        delta = np.clip(delta, self.mpc.delta_bounds[0], self.mpc.delta_bounds[1])

        # Convert delta to steering angle
        steering_angle = np.clip(front2steer(delta), 
            self.mpc.steering_angle_range[0], self.mpc.steering_angle_range[1])
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel, steering_angle, vehicle))

    def healthy(self):
        return self.mpc.trajectory is not None

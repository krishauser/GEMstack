import numpy as np
from math import sin, cos
import osqp
from scipy import sparse

from ...knowledge.vehicle.geometry import front2steer
from ...state.vehicle import VehicleState
from ...state.trajectory import Trajectory
from ..interface.gem import GEMVehicleCommand
from ..component import Component
from ...utils import settings


class MPCController:

    def _get_reference_points_from_trajectory(self, trajectory):
        """
        Generates [x, y, yaw, v] reference points for MPC using trajectory.eval().
        """
        ref_points = []
        t0, tf = trajectory.domain()

        for i in range(self.horizon):
            t_query = t0 + i * self.dt
            t_query = min(t_query, tf)  # clamp to domain

            pos = trajectory.eval(t_query)
            deriv = trajectory.eval_derivative(t_query)

            x, y = pos[0], pos[1]
            v = np.linalg.norm(deriv)
            yaw = math.atan2(deriv[1], deriv[0]) if v > 1e-3 else 0.0

            ref_points.append([x, y, yaw, v])

        return ref_points

    def __init__(self, horizon=10, dt=0.1):
        self.horizon = horizon
        self.dt = dt

        # Vehicle params
        self.Lf = settings.get('vehicle.geometry.wheelbase')
        self.max_steer = settings.get('vehicle.geometry.max_wheel_angle')
        self.max_accel = settings.get('vehicle.limits.max_acceleration')
        self.max_decel = settings.get('vehicle.limits.max_deceleration')
        self.speed_limit = settings.get('vehicle.limits.max_speed')

        # Weights for cost function
        self.w_cte = 1.0
        self.w_epsi = 1.0
        self.w_v = 1.0
        self.w_steer = 10.0
        self.w_accel = 10.0

    def solve_mpc(self, state: VehicleState, trajectory: Trajectory):
        x0 = np.array([state.pose.x, state.pose.y, state.pose.yaw, state.v])

        # Defensive check for trajectory
        if not hasattr(trajectory, 'get_reference_points'):
            print("MPC ERROR: Trajectory missing get_reference_points() method.")
            return 0.0, 0.0  # Safe fallback

        # Get reference points for horizon
        
        ref_points = self._get_reference_points_from_trajectory(trajectory)


        if len(ref_points) < self.horizon:
            print("MPC WARNING: Not enough reference points. Expected:", self.horizon, "Got:", len(ref_points))
            return 0.0, 0.0  # Stop safely

        N = self.horizon
        nx = 4  # States: x, y, psi, v
        nu = 2  # Inputs: steer, accel

        # Build cost matrices
        Q = sparse.diags([self.w_cte, self.w_cte, self.w_epsi, self.w_v])
        R = sparse.diags([self.w_steer, self.w_accel])

        P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), sparse.kron(sparse.eye(N), R)]).tocsc()
        q = np.zeros(nx * N + nu * N)

        # Constraints
        Ax = sparse.lil_matrix((nx * (N-1), nx * N + nu * N))
        l = np.zeros(nx * (N-1))
        u = np.zeros(nx * (N-1))

        for i in range(N-1):
            A, B = self._linearized_model(ref_points[i][2], ref_points[i][3])
            Ax[nx*i:nx*(i+1), nx*i:nx*(i+1)] = -A
            Ax[nx*i:nx*(i+1), nx*(i+1):nx*(i+2)] = sparse.eye(nx)
            Ax[nx*i:nx*(i+1), nx * N + nu*i:nx * N + nu*(i+1)] = -B

        Ax = Ax.tocsc()

        # Input bounds
        l_input = np.tile([-self.max_steer, -self.max_decel], N)
        u_input = np.tile([self.max_steer, self.max_accel], N)

        # Combine constraints
        A_tot = sparse.vstack([Ax, sparse.eye(nu * N)]).tocsc()
        l_tot = np.hstack([l, l_input])
        u_tot = np.hstack([u, u_input])

        # Solver
        prob = osqp.OSQP()
        prob.setup(P, q, A_tot, l_tot, u_tot, verbose=False)
        res = prob.solve()

        if res.info.status != 'solved':
            print("MPC ERROR: OSQP solver failed:", res.info.status)
            return 0.0, 0.0  # Safe fallback

        u_opt = res.x[nx * N : nx * N + nu]

        accel = np.clip(u_opt[1], -self.max_decel, self.max_accel)
        steer = np.clip(u_opt[0], -self.max_steer, self.max_steer)

        print("MPC OK: accel =", accel, "steer =", steer)
        return accel, steer
        print("Trajectory domain:", trajectory.domain())
        print("Trajectory start pose:", trajectory.eval(trajectory.domain()[0]))
        print("Trajectory end pose:", trajectory.eval(trajectory.domain()[1]))
        print("Ref points generated for MPC:")
        for pt in ref_points:
            print(pt)


    def _linearized_model(self, psi, v):
        A = np.eye(4)
        A[0, 2] = -v * sin(psi) * self.dt
        A[0, 3] = cos(psi) * self.dt
        A[1, 2] = v * cos(psi) * self.dt
        A[1, 3] = sin(psi) * self.dt
        A[2, 3] = self.dt / self.Lf
        A[3, 3] = 1.0

        B = np.zeros((4, 2))
        B[2, 0] = v / self.Lf * self.dt
        B[3, 1] = self.dt

        return A, B


class MPCTrajectoryTracker(Component):
    def __init__(self, vehicle_interface=None):
        self.mpc = MPCController()
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 50.0

    def state_inputs(self):
        return ["vehicle", "trajectory"]

    def state_outputs(self):
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        accel, f_delta = self.mpc.solve_mpc(vehicle, trajectory)

        steering_angle = front2steer(f_delta)
        steering_angle = np.clip(
            steering_angle,
            settings.get('vehicle.geometry.min_steering_angle', -0.5),
            settings.get('vehicle.geometry.max_steering_angle', 0.5)
        )

        cmd = self.vehicle_interface.simple_command(accel, steering_angle, vehicle)
        self.vehicle_interface.send_command(cmd)

    def healthy(self):
        return True

# File: GEMstack/onboard/planning/yield_spline_planner.py

from typing import List, Tuple
import numpy as np

from ..component import Component
from ...state import AllState, Path, Trajectory


class QuinticHermiteSplinePlanner:
    """
    Core quintic-Hermite engine: given coarse 2D or 3D waypoints
    (x,y[,heading]) builds a C2-continuous spline and samples it at fixed Δt.
    """
    def __init__(self, v_des: float = 1.0, dt: float = 0.02):
        self.v_des = v_des
        self.dt   = dt

    def _compute_headings(self, pts: np.ndarray) -> np.ndarray:
        """
        If pts.shape[1] == 3, assume pts[:,2] already contains heading ψ.
        Otherwise fall back to finite-difference approximation.
        """
        n, d = pts.shape
        if d == 3:
            # user-provided headings
            return pts[:, 2].copy()

        # approximate by central differences
        headings = np.zeros(n)
        for i in range(n):
            if i == 0:
                delta = pts[1] - pts[0]
            elif i == n - 1:
                delta = pts[-1] - pts[-2]
            else:
                delta = pts[i+1] - pts[i-1]
            headings[i] = np.arctan2(delta[1], delta[0])
        return headings

    def build(self,
              waypoints: List[List[float]]
             ) -> Tuple[np.ndarray, np.ndarray]:
        """
        waypoints: list of [x,y] or [x,y,ψ] entries
        returns (pts_out, t_out), each as an np.ndarray
        """
        W = np.array(waypoints, float)         # shape = (n,2) or (n,3)
        headings = self._compute_headings(W)
        tangents = np.stack([np.cos(headings),
                             np.sin(headings)], axis=1) * self.v_des

        pts_out = []
        t_out   = []
        t_accum = 0.0

        M = np.array([[1,  1,   1],
                      [3,  4,   5],
                      [6, 12,  20]], float)

        # build one quintic segment between each adjacent pair
        for i in range(len(W) - 1):
            p0, p1 = W[i,0:2],   W[i+1,0:2]
            m0, m1 = tangents[i], tangents[i+1]

            L = np.linalg.norm(p1 - p0)
            T = (L / self.v_des) if self.v_des > 0 else 0.0

            # Hermite coefficients a0..a5
            a0 = p0
            a1 = m0 * T
            a2 = np.zeros(2)

            RHS = np.vstack([
                p1      - (a0 + a1 + a2),
                m1 * T  - (        a1 + 2*a2),
                np.zeros(2) - (      2*a2)
            ])  # shape = (3,2)

            # solve for a3,a4,a5
            a3, a4, a5 = np.linalg.solve(M, RHS)

            # sample
            if T > 0:
                t_samples = np.arange(0.0, T, self.dt)
            else:
                t_samples = np.array([0.0])

            for tt in t_samples:
                s = tt / T if T > 0 else 0.0
                p = (a0
                     + a1 * s
                     + a2 * s**2
                     + a3 * s**3
                     + a4 * s**4
                     + a5 * s**5)
                pts_out.append(p.tolist())
                t_out.append(t_accum + tt)

            t_accum += T

        # append very last waypoint
        pts_out.append(W[-1,0:2].tolist())
        t_out.append(t_accum)

        return np.array(pts_out), np.array(t_out)


class SplinePlanner(Component):
    """Follows route by smoothing coarse waypoints into a quintic spline."""
    def __init__(self):
        super().__init__()
        self.route_progress = None
        self.t_last         = None

        # how far ahead to plan (m), and sampling speed & dt
        self.lookahead_dist = 10.0
        self.v_des          = 2.0
        self.dt             = 0.02

        # the spline engine
        self._spline = QuinticHermiteSplinePlanner(self.v_des, self.dt)

    def state_inputs(self):
        return ['all']

    def state_outputs(self):
        return ['trajectory']

    def rate(self):
        return 10.0  # Hz

    def update(self, state: AllState) -> Trajectory:
        t = state.t
        if self.t_last is None:
            self.t_last = t

        # keep route_progress up to date
        veh = state.vehicle
        curr = np.array([veh.pose.x, veh.pose.y])

        if self.route_progress is None:
            self.route_progress = 0.0
        _, new_param = state.route.closest_point_local(
            curr.tolist(),
            (self.route_progress - 5.0,
             self.route_progress + 5.0)
        )
        self.route_progress = new_param

        # extract a look-ahead segment from the route
        seg: Path = state.route.trim(
            self.route_progress,
            self.route_progress + self.lookahead_dist
        )

        # pull out the raw waypoints (may be [x,y] or [x,y,ψ])
        pts_raw: List[List[float]] = [
            list(pt) for pt in seg.points
        ]

        # build the quintic spline
        spline_pts, spline_times = self._spline.build(pts_raw)

        # wrap in GEMstack Trajectory
        traj = Trajectory(
            frame  = seg.frame,
            points = spline_pts.tolist(),
            times  = spline_times.tolist()
        )

        self.t_last = t
        return traj

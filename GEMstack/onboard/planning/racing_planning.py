from ...state.trajectory import Trajectory
from ...state.vehicle import VehicleState
from ...state import AgentState, AgentEnum
from ..component import Component

from ...state.trajectory import Trajectory, compute_headings, Path
from ...state.physical_object import ObjectFrameEnum

import numpy as np
# import matplotlib.pyplot as plt
import matplotlib                
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline
import signal
import threading

from typing import Dict
import atexit, os, numpy as np, matplotlib.pyplot as plt
from datetime import datetime


# --------------------------
# This is the main code for the racing trajectory planner.
# Contributors: Hua-Ta, Shilan
# --------------------------

_TRAJ_BUF = []

def make_reference_slalom(cones, amp=2.0, tail=10.0, pts_per_seg=50):
    """
    Quick centre-line generator that weaves around a cone list.

    orientation keywords may be lower/upper case.

    LEFT  → pass on +amp   |  RIGHT → amp   |  STANDING → back to 0
    """
    xk, yk = [0.0], [0.0]                     # start at (0,0)
    for c in cones:
        o = c['orientation'].lower()
        y = +amp if o == 'left' else -amp if o == 'right' else 0.0
        xk.append(c['x']); yk.append(y)
    xk.append(cones[-1]['x'] + tail); yk.append(0.0)   # little exit tail

    # monotone in x ⇒ cubic spline works well
    spline = CubicSpline(xk, yk, bc_type="natural")
    x_ref = np.linspace(xk[0], xk[-1], pts_per_seg*len(cones))
    y_ref = spline(x_ref)
    return x_ref, y_ref

def buffer_slalom_trajectory(
        x, y, c,
        waypoints=None,
        cones=None,
        ref=None,              
        tag=None):
    """Cache a trajectory segment for end-of-run plots."""
    _TRAJ_BUF.append(dict(
        x=np.asarray(x),
        y=np.asarray(y),
        c=np.asarray(c),
        waypoints=waypoints or [],
        cones=cones or [],
        x_ref=np.asarray(ref['x']) if ref else None,
        y_ref=np.asarray(ref['y']) if ref else None,
        tag=tag or f"seg{len(_TRAJ_BUF)}",
    ))


def _render_slalom_buffer(
    save_dir="/home/hbst/RACING/GEMstack/GEMstack/onboard/planning/test_slalom",
):
    if not _TRAJ_BUF:
        print("[slalom-plot] nothing buffered – no figure created")
        return

    os.makedirs(save_dir, exist_ok=True)
    fname = os.path.join(save_dir, f"slalom_{datetime.now():%Y%m%d_%H%M%S}.png")

    fig, axs  = plt.subplots(2, 2, figsize=(14, 10))
    ax_path, ax_rad = axs[0, 0], axs[0, 1]
    ax_cmp,  ax_err = axs[1, 0], axs[1, 1]
    R_MAX_PLOT   = 50.0 
    R_SCALE      = 5.0
    step_offset = 0
    plotted_cones = False 
    plotted_gen   = False   

    for d in _TRAJ_BUF:
        x, y = d["x"], d["y"]
        s  = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
        dx = np.gradient(x, s)
        dy = np.gradient(y, s)
        ddx = np.gradient(dx, s)
        ddy = np.gradient(dy, s)
        kappa = np.abs(dx*ddy - dy*ddx) / np.power(dx*dx + dy*dy, 1.5)
        r = 1.0 / np.where(kappa < 1e-9, np.inf, kappa)

        # s = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
        # dx  = np.gradient(x, s)
        # dy  = np.gradient(y, s)
        # ddx = np.gradient(dx, s)
        # ddy = np.gradient(dy, s)
        # kappa = np.abs(dx*ddy - dy*ddx) / np.power(dx*dx + dy*dy, 1.5)
        # r = 1.0 / np.clip(kappa, 1e-3, None)

        # path panel
        ax_path.plot(x, y, label=d["tag"])
        if d["cones"] and not plotted_cones:
            for cone in d["cones"]:
                ax_path.plot(cone['x'], cone['y'],
                marker='^', ms=6, mfc='gold', mec='black')
            plotted_cones = True
        # for wp in d["waypoints"]:
            # ax_path.plot(*wp, "rx")

        # radius panel
        # ax_rad.plot(r, label=d["tag"])
        r_plot = r.copy()
        r_plot[r_plot > R_MAX_PLOT] = np.nan
        r_plot = r_plot / R_SCALE                  
        steps   = np.arange(len(r_plot)) + step_offset
        ax_rad.plot(steps, r_plot, label=d["tag"])
        step_offset += len(r_plot)

        if d["x_ref"] is not None:
            # resample reference to same length as generated path
            # t_gen = np.linspace(0, 1, len(x))
            # t_ref = np.linspace(0, 1, len(d["x_ref"]))
            # x_ref = np.interp(t_gen, t_ref, d["x_ref"])
            # y_ref = np.interp(t_gen, t_ref, d["y_ref"])

            ax_cmp.plot(d["x_ref"], d["y_ref"],
                        lw=2, ls="--", color="black",
                        label=None if plotted_gen else "expected")

            # generated (always blue)
            ax_cmp.plot(x, y,
                        color="tab:blue",
                        label=None if plotted_gen else "generated")
            plotted_gen = True


            if d["cones"]:
                for cone in d["cones"]:
                    ax_cmp.plot(cone['x'], cone['y'],
                                marker='^', ms=6, mfc='gold', mec='black')

            # ax_cmp.plot(x_ref, y_ref, lw=2, ls='--', label=f'{d["tag"]} ref')
            # ax_cmp.plot(x,     y,     label=f'{d["tag"]} gen')

            # err = np.hypot(x_ref - x, y_ref - y)
            # err = np.hypot(d["x_ref"] - x, d["y_ref"] - y)
            # ax_err.plot(err, label=d["tag"])

            t_gen = np.linspace(0.0, 1.0, len(x))
            t_ref = np.linspace(0.0, 1.0, len(d["x_ref"]))
            x_ref_rs = np.interp(t_gen, t_ref, d["x_ref"])
            y_ref_rs = np.interp(t_gen, t_ref, d["y_ref"])

            err = np.hypot(x_ref_rs - x, y_ref_rs - y)
            ax_err.plot(err, label=d["tag"])


            if d["cones"]:
                cum_s   = np.cumsum(np.hypot(np.diff(x), np.diff(y)))
                cum_s   = np.insert(cum_s, 0, 0.0)
                for cone in d["cones"]:
                    idx = np.argmin(np.hypot(x - cone['x'], y - cone['y']))
                    ax_err.plot(idx, err[idx], marker='^',
                                ms=5, mfc='gold', mec='black')

    # styling  –  keep it tidy
    ax_path.set(title="generated paths", xlabel="x", ylabel="y");       ax_path.grid(); ax_path.legend(); ax_path.axhline(0, ls="--", c="gray")
    ax_rad.set(title="turn-radius profile", xlabel="step", ylabel="radius (m)"); ax_rad.grid(); ax_rad.legend(); ax_rad.axhline(11.0, ls="--", c="red", label="min safe R")
    ax_cmp.set(title="expected vs generated path", xlabel="x", ylabel="y");       ax_cmp.grid();  ax_cmp.legend()
    ax_err.set(title="absolute lateral error",    xlabel="step", ylabel="error (m)"); ax_err.grid(); ax_err.legend()

    fig.tight_layout()
    fig.savefig(fname)
    print(f"[slalom-plot] saved → {fname}")
    plt.close(fig)

atexit.register(_render_slalom_buffer)

def _sig_flush_then_exit(signum, frame):
    _render_slalom_buffer()
    # re-raise the signal’s default behaviour so the program still terminates
    signal.signal(signum, signal.SIG_DFL)
    os.kill(os.getpid(), signum)

signal.signal(signal.SIGINT,  _sig_flush_then_exit)   # Ctrl-C
signal.signal(signal.SIGTERM, _sig_flush_then_exit)   # docker / kill

def scenario_check(vehicle_state, cone_state):
    """
    Check the cone state and determine the scenario.
    Args:
        vehicle_state: The state of the vehicle.
        cone_state: The state of the cones.
    Returns:
        str: The scenario type.
        - 'left_pass': cone pointed left, go left
        - 'right_pass': cone pointed right, go right
        - 'u_turn': cone standing up, make a U-turn
        tuple: (cone_x, cone__y)
    """
    # Get the closest cone info
    cones_ahead = sorted(cone_state, key=lambda c: np.linalg.norm(
        np.array([c['x'], c['y']]) - np.array(vehicle_state['position'])))
    cone_direction = cones_ahead[0]['orientation']
    cone_position = (cones_ahead[0]['x'], cones_ahead[0]['y'])
    
    # Check the cone orientation
    if cone_direction != 'LEFT' and cone_direction != 'RIGHT' and cone_direction != 'STANDING' and cone_direction != '90turn':
        raise ValueError("Unknown cone orientation")
    
    return cone_direction, cone_position
    
def waypoint_generate(vehicle_state, cones, cone_idx, next_cone_idx, prev_cone_idx):
    """
    Generate waypoints based on the scenario.
    Args:
        vehicle_state: The state of the vehicle.
        cone_state: The state of the cones.
    Returns:
        Tuple[str, Tuple[float, float], Tuple[float, float]]:
            scenario: detected scenario type
            flex_wp: flexible waypoint (used to maneuver)
            fixed_wp: fixed waypoint (goal position)
    """
    print("cone idx: ", cone_idx, next_cone_idx, prev_cone_idx)
    scenario, cone_position = scenario_check(vehicle_state, [cones[cone_idx]])
    car_position = np.array(vehicle_state['position'])
    car_heading = vehicle_state['heading']  # in radians
    current_heading = car_heading
    target_heading = car_heading

    # ===== Parameters =====
    u_turn_radius = 8      # Radius for U-turn
    offset = 2.0                # Offset for left/right pass
    lookahead_distance = 10.0   # Distance ahead for fixed point
    # ======================

    # Direction vector based on heading
    heading_vector = np.array([np.cos(car_heading), np.sin(car_heading)])
    
    # Vector perpendicular to heading (to determine left/right)
    perpendicular_vector = np.array([-np.sin(car_heading), np.cos(car_heading)])

    if cone_idx > 0:
        if prev_cone_idx == None:
            prev_cone_idx = cone_idx - 1
        prev_cone_position = np.array((cones[prev_cone_idx]['x'], cones[prev_cone_idx]['y'])) 
        cone_position = np.array((cones[cone_idx]['x'], cones[cone_idx]['y']))
        target_heading = np.arctan2(cone_position[1] - prev_cone_position[1],
                                    cone_position[0] - prev_cone_position[0])

    # If next cone (2nd closest) is available, use it to determine lookahead_distance
    if next_cone_idx != None:
        next_cone_position = np.array((cones[next_cone_idx]['x'], cones[next_cone_idx]['y']))
        lookahead_distance = np.linalg.norm(next_cone_position - cone_position) / 2

    if scenario == 'STANDING':
        # U-turn: Generate points in a semi-circular arc around the cone
        cone = np.array(cone_position)
        
        # Number of waypoints to generate for the arc
        num_arc_points = 9
        
        # Generate waypoints in a smooth semi-circular pattern on the RIGHT side
        flex_wps_list = []
        
        # Create a semi-circular arc from 0 to π
        # But negate perpendicular_vector to go to the right side
        for i in range(num_arc_points):
            # Calculate angle in the semi-circle (0 to π)
            angle = i * np.pi * 1 / (num_arc_points - 1)
            
            # MODIFIED: Negative perpendicular_vector to go to right side of cone
            # This creates a counter-clockwise turn around the cone
            wp = cone - u_turn_radius * (
                perpendicular_vector * np.cos(angle) - 
                heading_vector * np.sin(angle)
            )
            
            flex_wps_list.append(wp)
            
        # Fixed waypoint: behind the cone after U-turn (also on right side)
        fixed_wp = cone + u_turn_radius * perpendicular_vector

        target_heading = target_heading + np.pi
        
        # For visualization only
        # for i, wp in enumerate(flex_wps_list):
        #     plt.plot(wp[0], wp[1], 'ro')  # Plot all waypoints in red
        #     plt.text(wp[0], wp[1], f'wp{i}', fontsize=9)
            
        # plt.plot(fixed_wp[0], fixed_wp[1], 'bo')  # Plot fixed waypoint in blue
        # plt.plot(cone[0], cone[1], 'gx')  # Plot cone in green
        # plt.plot(car_position[0], car_position[1], 'ks')  # Plot car position
        # plt.axis('equal')  # Equal aspect ratio for better visualization
        # plt.grid(True)
        # plt.show()

    elif scenario == 'LEFT':
        cone = np.array(cone_position)

        # Flexible waypoint: go to the left of the cone
        flex_wp1 = cone + offset * perpendicular_vector
        flex_wps_list = [flex_wp1]

        # Fixed waypoint: forward after passing the cone
        # fixed_wp = flex_wp + heading_vector * lookahead_distance - offset * perpendicular_vector * 2
        fixed_wp = flex_wp1 + heading_vector * lookahead_distance - offset * perpendicular_vector

    elif scenario == 'RIGHT':
        cone = np.array(cone_position)

        # Flexible waypoint: go to the right of the cone
        flex_wp1 = cone - offset * perpendicular_vector
        flex_wps_list = [flex_wp1]

        # Fixed waypoint: forward after passing the cone
        # fixed_wp = flex_wp + heading_vector * lookahead_distance + offset * perpendicular_vector * 2
        fixed_wp = flex_wp1 + heading_vector * lookahead_distance + offset * perpendicular_vector

    # TODO: move to a different setting instead of as a scenario
    elif scenario == '90turn':
        turn_vector = np.array([-np.sin(car_heading + np.pi / 6), np.cos(car_heading + np.pi / 6)])
        cone = np.array(cone_position)
        distance_to_cone = np.linalg.norm(car_position - cone)

        flex_wps_list = []
        if distance_to_cone > 7:
            steps = int(distance_to_cone // 6)
            print("steps:", steps)

            final_flex_wp = cone + 1.0 * perpendicular_vector + heading_vector * 1.0
            fixed_wp = cone - turn_vector * 2.5 + heading_vector * 0.0

            cone_direction = (final_flex_wp - car_position) / np.linalg.norm(final_flex_wp - car_position)
            for i in range(steps - 2):
                intermediate_wp = car_position + cone_direction * ((i + 1) * 6)
                flex_wps_list.append(intermediate_wp)

            flex_wps_list.append(final_flex_wp)
        else:
            flex_wp = cone + 1.0 * perpendicular_vector + heading_vector * 1.0
            fixed_wp = cone - turn_vector * 2.5 + heading_vector * 0.0
            flex_wps_list = [flex_wp]

    else:
        flex_wps_list = None
        fixed_wp = None

    target_heading = (target_heading + np.pi) % (2 * np.pi) - np.pi
    current_heading = (current_heading + np.pi) % (2 * np.pi) - np.pi

    if abs(target_heading - current_heading) > np.pi: 
        if target_heading < current_heading:
            target_heading += 2 * np.pi
        else:
            target_heading -= 2 * np.pi

    return scenario, flex_wps_list, fixed_wp, target_heading

def trajectory_generation(init_state, final_state, N=30, T=0.1, Lr=1.5,
                          w_c=10.0, w_eps=0.0, w_vvar=4.0,
                          w_terminal=10.0,
                          v_min=3.0, v_max=11.0,
                          waypoints=None, waypoint_penalty_weight=100.0, 
                          enable_debug_viz=True):
    """
    Generate a dynamically feasible trajectory between init_state and final_state
    using curvature-based vehicle dynamics and nonlinear optimization.
    
    Now supports multiple waypoints.

    Parameters:
    - init_state (dict): Initial vehicle state with keys 'x', 'y', 'psi', 'c', 'v'.
    - final_state (dict): Target vehicle state with keys 'x', 'y', 'psi', 'c'.
    - N (int): Number of discrete time steps in the trajectory.
    - T (float): Duration of each time step (in seconds).
    - Lr (float): Distance from the vehicle's center to the rear axle.
    - w_c (float): Weight for penalizing curvature (smoothness of turns).
    - w_eps (float): Weight for penalizing curvature rate (reduces sharp steering changes).
    - w_vvar (float): Weight for penalizing speed variance (encourages speed smoothness).
    - w_terminal (float): Weight for penalizing final state deviation (soft constraint).
    - v_min (float): Minimum allowed speed (in m/s).
    - v_max (float): Maximum allowed speed (in m/s).
    - waypoints (list or None): Optional list of (x, y) coordinates that the trajectory should pass near.
    - waypoint_penalty_weight (float): Penalty weight for distance from waypoints (soft constraint).

    Returns:
    - x, y, psi, c, v, eps (np.ndarray): Arrays of optimized state and control values.
    - final_error (dict): Final state errors in x, y, psi, and c.
    """
    final_heading = (final_state['psi'] + np.pi) % (2 * np.pi) - np.pi
    init_heading = (init_state['psi'] + np.pi) % (2 * np.pi) - np.pi

    if abs(final_heading - init_heading) > np.pi: 
        if final_heading < init_heading:
            final_heading += 2 * np.pi
        else:
            final_heading -= 2 * np.pi

    init_state['psi'] = init_heading
    final_state['psi'] = final_heading
    print("init and final headings: ", init_heading, final_heading)

    def cost(p):
        x_, y_, psi_, c_, v_, eps_ = np.split(p, [N - 1, 2 * (N - 1), 3 * (N - 1), 4 * (N - 1), 5 * (N - 1)])
        c_seq = np.concatenate(([init_state['c']], c_))
        v_seq = np.concatenate(([init_state['v']], v_))
        x_final, y_final, psi_final, c_final = x_[-1], y_[-1], psi_[-1], c_[-1]

        cost_c = w_c * np.sum(c_seq ** 2)
        cost_eps = w_eps * np.sum(eps_ ** 2)
        v_mean = np.mean(v_seq)
        cost_vvar = w_vvar * np.mean((v_seq - v_mean) ** 2)

        cost_terminal = w_terminal * (
            (x_final - final_state['x']) ** 2 +
            (y_final - final_state['y']) ** 2 +
            (psi_final - final_state['psi']) ** 2 * 100 + 
            (c_final - final_state['c']) ** 2 * 100
        )

        cost_waypoints = 0.0
        if waypoints is not None and len(waypoints) > 0:
            # Calculate equally spaced indices for each waypoint
            num_waypoints = len(waypoints)
            indices = [int((i + 1) * (N - 1) / (num_waypoints + 1)) for i in range(num_waypoints)]
            
            # Sum penalties for each waypoint at its corresponding index
            for wp_idx, waypoint in enumerate(waypoints):
                traj_idx = indices[wp_idx]
                cost_waypoints += waypoint_penalty_weight * (
                    (x_[traj_idx] - waypoint[0])**2 + (y_[traj_idx] - waypoint[1])**2
                )

        return cost_c + cost_eps + cost_vvar + cost_terminal + cost_waypoints
    def dynamics_constraints(p):
        x_, y_, psi_, c_, v_, eps_ = np.split(p, [N - 1, 2 * (N - 1), 3 * (N - 1), 4 * (N - 1), 5 * (N - 1)])
        constraints = []
        x_prev, y_prev, psi_prev, c_prev, v_prev = init_state['x'], init_state['y'], init_state['psi'], init_state['c'], init_state['v']
        for k in range(N - 1):
            dx = v_prev * np.cos(psi_prev + c_prev * Lr) * T
            dy = v_prev * np.sin(psi_prev + c_prev * Lr) * T
            dpsi = v_prev * c_prev * T
            dc = eps_[k] * T
            constraints.extend([
                x_[k] - (x_prev + dx),
                y_[k] - (y_prev + dy),
                psi_[k] - (psi_prev + dpsi),
                c_[k] - (c_prev + dc)
            ])
            x_prev, y_prev, psi_prev, c_prev, v_prev = x_[k], y_[k], psi_[k], c_[k], v_[k]
        return constraints

    # Initial guesses
    x_vals = np.linspace(init_state['x'], final_state['x'], N)
    y_vals = np.linspace(init_state['y'], final_state['y'], N)
    psi_vals = np.linspace(init_state['psi'], final_state['psi'], N)
    c_vals = np.linspace(init_state['c'], final_state['c'], N)
    v_vals = np.ones(N) * init_state['v']
    eps_vals = np.zeros(N - 1)

    p0 = np.concatenate([x_vals[1:], y_vals[1:], psi_vals[1:], c_vals[1:], v_vals[1:], eps_vals])
    bounds = [(None, None)] * (4 * (N - 1)) + [(v_min, v_max)] * (N - 1) + [(None, None)] * (N - 1)

    result = minimize(cost, p0, bounds=bounds,
                      constraints={'type': 'eq', 'fun': dynamics_constraints},
                      options={'maxiter': 1000})
    if not result.success:
        raise RuntimeError("Optimization failed")

    x_, y_, psi_, c_, v_, eps_ = np.split(result.x, [N - 1, 2 * (N - 1), 3 * (N - 1), 4 * (N - 1), 5 * (N - 1)])
    x_full = np.concatenate(([init_state['x']], x_))
    y_full = np.concatenate(([init_state['y']], y_))
    psi_full = np.concatenate(([init_state['psi']], psi_))
    c_full = np.concatenate(([init_state['c']], c_))
    v_full = np.concatenate(([init_state['v']], v_))

    final_error = {
        'x_error': abs(x_full[-1] - final_state['x']),
        'y_error': abs(y_full[-1] - final_state['y']),
        'psi_error': abs(psi_full[-1] - final_state['psi']),
        'c_error': abs(c_full[-1] - final_state['c']),
    }

    cones = [
        {'x': 10, 'y': 0.0, 'orientation': 'left'},
        {'x': 30, 'y': 1.0, 'orientation': 'right'},
        {'x': 50, 'y': 0.0, 'orientation': 'left'},
        {'x': 70, 'y': 1.0, 'orientation': 'standing'},
    ]
    x_ref, y_ref = make_reference_slalom(cones, amp=2.0, tail=10.0, pts_per_seg=80)
    buffer_slalom_trajectory(x_full, y_full, c_full,
                         waypoints=waypoints,
                         ref={'x': x_ref, 'y': y_ref},
                         cones=cones)  


    return x_full, y_full, psi_full, c_full, v_full, eps_, final_error

######## more dynamics version
def trajectory_generation_dynamics(init_state, final_state, N=30, Lr=1.5,
                          a_min=-3.0, a_max=3.0,
                          eps_min=-0.2, eps_max=0.2,
                          v_min=2.0, v_max=11.0,
                          T_min = 0.5, T_max = 1000.0,
                          waypoints=None, waypoint_penalty_weight=10, waypoint_headings=None):
    
    """
    Generate a dynamically feasible trajectory between init_state and final_state
    using curvature-based vehicle dynamics and nonlinear optimization.
    
    Now supports multiple waypoints.

    Parameters:
    - init_state (dict): Initial vehicle state with keys 'x', 'y', 'psi', 'c', 'v'.
    - final_state (dict): Target vehicle state with keys 'x', 'y', 'psi', 'c'.
    - N (int): Number of discrete time steps in the trajectory.
    - T (float): Duration of each time step (in seconds).
    - Lr (float): Distance from the vehicle's center to the rear axle.
    - w_c (float): Weight for penalizing curvature (smoothness of turns).
    - w_eps (float): Weight for penalizing curvature rate (reduces sharp steering changes).
    - w_vvar (float): Weight for penalizing speed variance (encourages speed smoothness).
    - w_terminal (float): Weight for penalizing final state deviation (soft constraint).
    - v_min (float): Minimum allowed speed (in m/s).
    - v_max (float): Maximum allowed speed (in m/s).
    - waypoints (list or None): Optional list of (x, y) coordinates that the trajectory should pass near.
    - waypoint_penalty_weight (float): Penalty weight for distance from waypoints (soft constraint).

    Returns:
    - x, y, psi, c, v, eps (np.ndarray): Arrays of optimized state and control values.
    - final_error (dict): Final state errors in x, y, psi, and c.
    """
    def cost(p):
        T_total = p[-1]
        return T_total
    
    def dynamics_constraints(p):
        # x_, y_, psi_, c_, v_, eps_ = np.split(p, [N - 1, 2 * (N - 1), 3 * (N - 1), 4 * (N - 1), 5 * (N - 1)])
        # print("x_: " +str(x_))
        split_idx = 5 *(N - 1)
        x_, y_, psi_, c_, v_= np.split(p[:split_idx], 5)
        
        a = p[split_idx:split_idx + (N - 1)]
        eps = p[split_idx + (N - 1):-1]
        T_total = p[-1]
        T_k = T_total / (N - 1)
        
        constraints = []
        x_prev, y_prev, psi_prev, c_prev, v_prev = init_state['x'], init_state['y'], init_state['psi'], init_state['c'], init_state['v']
        
        for k in range(N - 1):
            dx = v_prev * np.cos(psi_prev + c_prev * Lr) * T_k
            dy = v_prev * np.sin(psi_prev + c_prev * Lr) * T_k
            dpsi = v_prev * c_prev * T_k
            dv = a[k] * T_k
            dc = eps[k] * T_k
            constraints.extend([
                x_[k] - (x_prev + dx),
                y_[k] - (y_prev + dy),
                psi_[k] - (psi_prev + dpsi),
                v_[k] - (v_prev + dv),
                c_[k] - (c_prev + dc)
            ])
            x_prev, y_prev, psi_prev, c_prev, v_prev = x_[k], y_[k], psi_[k], c_[k], v_[k]
        return constraints
    
    def waypoint_penalty(p):
        if waypoints is None or len(waypoints) == 0:
            return 0.0
    
        split_idx = 5 *(N - 1)
        x_, y_, psi_, c_, v_= np.split(p[:split_idx], 5)
        # x_ = p[:N -1]
        # y_ = p[N - 1:2 * (N-1)]
        # psi_ = [ 3 * (N - 1)]
        # Calculate equally spaced indices for each waypoint
        num_waypoints = len(waypoints)
        indices = [int((i + 1) * (N - 1) / (num_waypoints + 1)) for i in range(num_waypoints)]
        penalty = 0.0
        alignment_threshold = 0.3
        # Sum penalties for each waypoint at its corresponding index
        for wp_idx, waypoint in enumerate(waypoints):
            traj_idx = indices[wp_idx]
            penalty += waypoint_penalty_weight * (
                (x_[traj_idx] - waypoint[0])**2 + (y_[traj_idx] - waypoint[1])**2
            )
        return penalty

    # Initial guesses
    x_vals = np.linspace(init_state['x'], final_state['x'], N)
    y_vals = np.linspace(init_state['y'], final_state['y'], N)
    psi_vals = np.linspace(init_state['psi'], final_state['psi'], N)
    c_vals = np.linspace(init_state['c'], final_state['c'], N)
    v_vals = np.ones(N) * init_state['v']

    a_vals = np.zeros(N - 1)
    eps_vals = np.zeros(N - 1)
    T_guess = 60.0

    p0 = np.concatenate([x_vals[1:], y_vals[1:], psi_vals[1:], c_vals[1:], v_vals[1:], 
                         a_vals, eps_vals, [T_guess]])
    num_vars = len(p0)
    bounds = ([(None, None)] * (4 * (N - 1)) + 
              [(v_min, v_max)] * (N - 1) + 
              [(a_min, a_max)] * (N - 1) +
              [(eps_min, eps_max)] * (N - 1) +
              [(T_min, T_max)]
    )
    def total_cost(p):
        return cost(p) + waypoint_penalty(p)
    
    result = minimize(total_cost, 
                      p0, 
                      bounds=bounds,
                      constraints={'type': 'eq', 'fun': dynamics_constraints},
                      options={'maxiter': 1000})
    
    # print("result.x:" + str(result.x))
    if not result.success:
        raise RuntimeError("Optimization failed")

    split_idx = 5 *(N - 1)
    x_, y_, psi_, c_, v_= np.split(result.x[:split_idx], 5)
    
    a = result.x[split_idx:split_idx + (N - 1)]
    eps = result.x[split_idx + (N - 1):-1]
    T_total = result.x[-1]
    
    x_full = np.concatenate(([init_state['x']], x_))
    y_full = np.concatenate(([init_state['y']], y_))
    psi_full = np.concatenate(([init_state['psi']], psi_))
    c_full = np.concatenate(([init_state['c']], c_))
    v_full = np.concatenate(([init_state['v']], v_))

    final_error = {
        'x_error': abs(x_full[-1] - final_state['x']),
        'y_error': abs(y_full[-1] - final_state['y']),
        'psi_error': abs(psi_full[-1] - final_state['psi']),
        'c_error': abs(c_full[-1] - final_state['c']),
    }

    return x_full, y_full, psi_full, c_full, v_full, a, eps, T_total, final_error
#########

def feasibility_check(trajectory, cone_map, car_width=2.0, safety_margin=0.3, v=10.0, Lr=1.5, T=0.1):
    """
    Check if the car trajectory collides with any cones.

    Parameters:
    - trajectory: list of (y, psi, c) states
    - cone_map: list of (x, y) cone positions
    - car_width: width of the vehicle in meters
    - safety_margin: buffer around the vehicle
    - v: vehicle constant speed (used for x position estimation)
    - Lr: distance to rear axle
    - T: time step

    Returns:
    - feasible: True if no collisions
    - collisions: list of indices of cones that were collided with (for plotting purpose)
    - x_vals, y_vals: trajectory positions for plotting (for plotting purpose)
    """
    y_vals, psi_vals, c_vals = zip(*trajectory)
    x_vals = [0.0]
    for i in range(1, len(trajectory)):
        dx = v * np.cos(psi_vals[i-1] + c_vals[i-1] * Lr) * T
        x_vals.append(x_vals[-1] + dx)

    collision_radius = (car_width / 2.0) + safety_margin
    collisions = []

    for j, (cone_x, cone_y) in enumerate(cone_map):
        for x, y in zip(x_vals, y_vals):
            if np.hypot(x - cone_x, y - cone_y) < collision_radius:
                collisions.append(j)
                break

    feasible = len(collisions) == 0
    return feasible, collisions, x_vals, y_vals


def waypoint_search_optimization(vehicle_state, cones, search_attempts=3):
    valid_flex_wps = []
    valid_fixed_wps = []
    current_state = vehicle_state.copy()
    cones_copy = cones.copy()

    for i in range(min(search_attempts, len(cones_copy))):
        scenario, flex_wps, fixed_wp = waypoint_generate(current_state, cones_copy)

        if flex_wps is None or fixed_wp is None:
            break

        for flex_wp in flex_wps:
            init_state = {
                'x': current_state['position'][0],
                'y': current_state['position'][1],
                'psi': current_state['heading'],
                'c': 0.0,
                'v': current_state['velocity']
            }
            final_state = {
                'x': fixed_wp[0],
                'y': fixed_wp[1],
                'psi': current_state['heading'],
                'c': 0.0
            }

            try:
                x, y, psi, c, v, eps, final_error = trajectory_generation(init_state, final_state, waypoint=flex_wp)
                trajectory = list(zip(y, psi, c))
                feasible, collisions, x_vals, y_vals = feasibility_check(trajectory, [(cone['x'], cone['y']) for cone in cones])
                # print(f"Checking waypoint: {flex_wp}, Fixed: {fixed_wp}, Feasible: {feasible}, Collisions: {collisions}")
                if feasible:
                    valid_flex_wps.append(flex_wp)
                    valid_fixed_wps.append(fixed_wp)
                    current_state['position'] = list(fixed_wp)
                    break
            except:
                continue
        if len(cones_copy) > 0:
            cones_copy.pop(0)
    return valid_flex_wps, valid_fixed_wps


def to_gemstack_trajectory(x_all, y_all, v_all, T=0.1):
    t_vals = np.arange(len(x_all)) * T
    combined_xy = [[x, y] for x, y in zip(x_all, y_all)]
    curr_path = Path(ObjectFrameEnum.START,combined_xy)
    path = compute_headings(curr_path, smoothed=True)
    path_normalized = path.arc_length_parameterize()
    points = [p for p in path_normalized.points]
    return Trajectory(points=points, times=t_vals, frame=ObjectFrameEnum.START)


def plan_full_slalom_trajectory(vehicle_state, cones):
    x_all, y_all, v_all = [], [], []
    current_pos = np.array(vehicle_state['position'])
    current_heading = vehicle_state['heading']

    for cone_idx, cone in enumerate(cones):
        scenario, flex_wps, fixed_wp, target_heading = waypoint_generate(vehicle_state, cones, cone_idx)
        print(f"Scenario: {scenario}, Cone: {cone}, Flex WP: {flex_wps}, Fixed WP: {fixed_wp}")
        if not flex_wps or fixed_wp is None:
            continue
        # flex_wp = flex_wps[0]
        current_heading = (current_heading + np.pi) % (2 * np.pi) - np.pi

        init_state = {
            'x': current_pos[0], 'y': current_pos[1], 'psi': current_heading,
            'c': 0.0, 'v': vehicle_state['velocity']
        }
        final_state = {
            'x': fixed_wp[0], 'y': fixed_wp[1], 'psi': target_heading, 'c': 0.0
        }

        x, y, psi, c, v, eps, _ = trajectory_generation(init_state, final_state, waypoints=flex_wps)
        x_all.extend(x)
        y_all.extend(y)
        v_all.extend(v)

        current_pos = np.array([x[-1], y[-1]])
        current_heading = psi[-1]
        current_steering = c[-1]

        vehicle_state = {
            'position': current_pos,
            'heading': current_heading,
            'velocity': v[-1]
        }
        current_pos = np.array([x[-1], y[-1]])

    # # Plot overall trajectory
    # plt.figure()
    # plt.plot(x_all, y_all, label='Overall Trajectory')

    # # Plot cones
    # for i, cone in enumerate(cones):
    #     plt.scatter(cone['x'], cone['y'], color='orange', s=10, label='Cone' if i == 0 else "")
    #     plt.text(cone['x'], cone['y'] + 0.5, f'C{i+1}', ha='center', fontsize=9, color='darkorange')

    # plt.title('4-Cone Full Course Trajectory')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    # plt.legend()
    # plt.axis('equal')
    # plt.grid(True)
    # plt.show()

    combined_xy = [[x, y] for x, y in zip(x_all, y_all)]
    # print(combined_xy)
    path = Path(ObjectFrameEnum.START,combined_xy)
    path = compute_headings(path)
    path = path.arc_length_parameterize()
    # print(path)
    return path.racing_velocity_profile()
    # return to_gemstack_trajectory(x_all, y_all, v_all)


def no_cone_planning(vehicle_dict):
    temp_points = []
    vehicle_x, vehicle_y = vehicle_dict['position'][0], vehicle_dict['position'][1]
    vehicle_heading = vehicle_dict['heading']
    vehicle_velocity = vehicle_dict['velocity']
    step_size = 0.00001
    for i in range(10):
        temp_points.append([vehicle_x + i * step_size * np.cos(vehicle_heading),
                            vehicle_y + i * step_size * np.sin(vehicle_heading)])

    path = Path(ObjectFrameEnum.START,temp_points)
    path = compute_headings(path)
    path = path.arc_length_parameterize()
    # print(path)
    return path.racing_velocity_profile()

def got_new_cone(current_cones, prev_cones):
    if current_cones is None:
        return False
    if prev_cones is None:
        return True
    prev_ids = {cone['id'] for cone in prev_cones}

    for cone in current_cones:
        if cone['id'] not in prev_ids:
            return True  # Found a new cone not in previous list

    return False
################################################
# Main Racing Trajectory Planner Class
################################################
class SlalomTrajectoryPlanner(Component):
    def __init__(self, **kwargs):
        # You can accept args here if needed
        self.prev_vehicle_position = None
        self.trajectory = None
        self.prev_cones = None
        self.prev_cone_idx = None # Used to set heading in waypoint_generate
        self.travelled_distance = 0.0
        self.cones = []
        # ----------------------------
        # Predifined-Cones Simulation
        # self.run_fake_plan = True
        # self.onboard = False

        # # Onboard
        self.run_fake_plan = False
        self.onboard = True       
        # ----------------------------
        # Planner runs on different thread
        self.plan_thread = None
        self.plan_lock = threading.Lock()
        self.plan_pending = False

        self.DEBUG_MODE = True

    def state_inputs(self):
        return ['agents', 'vehicle']  # Receive VehicleState & AgentState input

    def state_outputs(self):
        return ['trajectory']         # Return trajectory output
    
    def rate(self):                   # Setup the update rate
        return 1.0 

    def update(self, agents: Dict[str, AgentState], vehicle: VehicleState):
        # Running on real vehicle
        if self.onboard:
            # Get all current detected cones
            cones = []
            n = 0
            for id, agent in agents.items():
                if agent.type == AgentEnum.CONE:
                    # ===== RUNNING ONBOARD =====
                    # cones.append({
                    #     'id': id,
                    #     'x': agent.pose.x,
                    #     'y': agent.pose.y,
                    #     'orientation': agent.activity
                    # })
                    # ===== TESTING ONBOARD in BASIC SIM =====
                    if n > 3:
                        break
                    if n % 4 == 0:
                        curr_activity = 'LEFT'
                    elif n % 4 == 1:
                        curr_activity = 'RIGHT'
                    elif n % 4 == 2:
                        curr_activity = 'LEFT'
                    else:
                        curr_activity = 'STANDING'
                    c = {
                        'id': id,
                        'x': agent.pose.x,
                        'y': agent.pose.y,
                        'orientation': curr_activity
                    }
                    n = n + 1
                    if c['id'] not in {cone['id'] for cone in self.cones}:
                        self.cones.append(c)
            
            curr_pos = np.array([vehicle.pose.x, vehicle.pose.y])
            if self.prev_vehicle_position is None:
                distance_increment = 0.0
            else:
                distance_increment = np.linalg.norm(curr_pos - self.prev_vehicle_position)

            self.prev_vehicle_position = curr_pos

            vehicle_dict = {
                'position': [vehicle.pose.x, vehicle.pose.y],
                'heading': vehicle.pose.yaw,
                'velocity': vehicle.v
            }
            if self.DEBUG_MODE:
                print("===================== STATES =====================")
                print(f"Vehicle State: {vehicle_dict}")
                print(f"Detected Cones: {self.cones}")
                print("===================== ====== =====================")

            # self.trajectory = self.online_trajectory_planning(vehicle_dict, self.cones, distance_increment)
            if not self.plan_pending:
                self.plan_pending = True
                vehicle_copy = vehicle_dict.copy()
                cones_copy = list(self.cones)
                distance_copy = distance_increment

                def plan():
                    new_traj = self.online_trajectory_planning(vehicle_copy, cones_copy, distance_copy)
                    with self.plan_lock:
                        self.trajectory = new_traj
                        self.plan_pending = False

                self.plan_thread = threading.Thread(target=plan)
                self.plan_thread.start()

            # If no cones detected, drive forward
            if len(self.cones) == 0:
                self.trajectory = no_cone_planning(vehicle_dict)
            # # Otherwise, plan trajectory
            # elif got_new_cone(cones, self.prev_cones):
            #     # Replan only if new cones are detected
            #     self.trajectory = plan_full_slalom_trajectory(vehicle_dict, cones)
            #     self.prev_cones = cones
            # else:
            #     # No need to update the plan if the same cones are detected
            #     self.prev_cones = cones
        
        # Testing with predefined fake generated cone positions
        elif self.run_fake_plan:
            # Example Test - Slalom
            cones = [
                {'x': 10, 'y': 0.0, 'orientation': 'LEFT'},
                {'x': 30, 'y': 1.0, 'orientation': 'RIGHT'},
                {'x': 50, 'y': 0.0, 'orientation': 'LEFT'},
                {'x': 70, 'y': 1.0, 'orientation': 'STANDING'},
                {'x': 50, 'y': 0.0, 'orientation': 'RIGHT'},
                {'x': 30, 'y': 1.0, 'orientation': 'LEFT'},
                {'x': 10, 'y': 0.0, 'orientation': 'RIGHT'}
            ]
            vehicle_dict = {
                'position': [vehicle.pose.x, vehicle.pose.y],
                'heading': vehicle.pose.yaw,
                'velocity': vehicle.v
            }

            # Example Test - Racing Circle
            # cones = [
            #     {'x': -60.083, 'y': -33.118, 'orientation': '90turn'},
            #     {'x': -6.392, 'y': -5.147, 'orientation': '90turn'},
            #     {'x': -5.625, 'y': -13.637, 'orientation': '90turn'},
            #     {'x': -56.258, 'y': -41.080, 'orientation': '90turn'}
            # ]
            # vehicle_dict = {
            #     'position': [-60.0, -39.0],
            #     'heading': np.pi * 2 / 3,
            #     'velocity': 0.0
            # }

            # Example Test - Racing Circle
            # cones = [
            #     {'x': 0.0, 'y': 70.0, 'orientation': '90turn'},
            #     {'x': 30.0, 'y': 70.0, 'orientation': '90turn'},
            #     {'x': 30.0, 'y': 0.0, 'orientation': '90turn'},
            #     {'x': 0.0, 'y': 0.0, 'orientation': '90turn'}
            # ]
            # vehicle_dict = {
            #     'position': [0.0, 20.0],
            #     'heading': 0.0,
            #     'velocity': 0.0
            # }

            self.trajectory = plan_full_slalom_trajectory(vehicle_dict, cones)
            self.run_fake_plan = False
        
        # Update output
        with self.plan_lock:
            return self.trajectory

    def online_trajectory_planning(self, vehicle_state, cones, distance_increment, replan_threshold=100.0):
        if not hasattr(self, 'prev_cones'):
            self.prev_cones = None

        if not hasattr(self, 'no_cone_ahead'):
            self.no_cone_ahead = False

        if not hasattr(self, 'visited_cone_ids'):
            self.visited_cone_ids = set()

        stitch_idx = -1

        def got_new_cone(current, prev):
            if prev is None:
                return True
            prev_ids = {c['id'] for c in prev}
            return any(c['id'] not in prev_ids for c in current)

        self.travelled_distance += distance_increment
        new_cone_detected = got_new_cone(self.cones, self.prev_cones)

        # Plan at the beginning or when new cones detected or after threshold distance
        if self.trajectory is None or new_cone_detected or not self.no_cone_ahead or True:
            self.prev_cones = self.cones

            if self.trajectory is None:
                current_position = vehicle_state['position']
                init_state = {
                    'x': current_position[0],
                    'y': current_position[1],
                    'psi': vehicle_state['heading'],  
                    'c': 0.0,
                    'v': vehicle_state['velocity']
                }
            else:
                stitch_idx, init_point, heading = self.get_future_point_on_trajectory(self.trajectory, vehicle_state['position'], lookahead_distance=500.0)
                init_state = {
                    'x': init_point[0], 
                    'y': init_point[1], 
                    'psi': heading,
                    'c': 0.0, 
                    'v': vehicle_state['velocity']
                }
                vehicle_state['position'] = np.array([init_point[0], init_point[1]])
                vehicle_state['heading'] = heading

            print("all cones: ", self.cones)
            current_cone_idx, next_cone_idx, updated_cones = self.get_current_cone_idx(self.cones, init_state)
            self.cones = updated_cones
            print("init state: ", init_state)
            print("current cone: ", current_cone_idx, next_cone_idx)

            # No cone ahead
            if current_cone_idx == None:
                self.no_cone_ahead = True
                return self.trajectory
            else:
                self.no_cone_ahead = False

            # No need to plan if there is no new cone detected and no cone ahead
            if not new_cone_detected and self.no_cone_ahead:
                return self.trajectory

            self.visited_cone_ids.add(self.cones[current_cone_idx]['id'])                
            scenario, flex_wps, fixed_wp, target_heading = waypoint_generate(vehicle_state, self.cones, current_cone_idx, next_cone_idx, self.prev_cone_idx)
            self.prev_cone_idx = current_cone_idx

            if flex_wps and fixed_wp is not None:
                final_state = {
                    'x': fixed_wp[0], 'y': fixed_wp[1], 'psi': target_heading, 'c': 0.0
                }

                # Stitch from current vehicle position to new plan start
                if self.trajectory is not None:
                    print("init and final state: ", init_state, final_state)
                    # 1. Plan new trajectory from init_state onward
                    x_new, y_new, psi_new, _, v_new, _, _ = trajectory_generation(init_state, final_state, waypoints=flex_wps)

                    # 2. Cut old trajectory up to init_state (e.g., index `stitch_idx`)
                    old_points = self.trajectory.points[:stitch_idx]
                    old_x = [p[0] for p in old_points]
                    old_y = [p[1] for p in old_points]
                    old_v = [vehicle_state['velocity']] * len(old_x)  # or extract from old trajectory if available

                    # 3. Combine old + new
                    x_full = np.concatenate([old_x, x_new])
                    y_full = np.concatenate([old_y, y_new])
                    v_full = np.concatenate([old_v, v_new])

                    # if current_cone_idx == 6:
                    #     # Plot overall trajectory
                    #     plt.figure()
                    #     plt.plot(x_full, y_full, label='Overall Trajectory')

                    #     # Plot cones
                    #     for i, cone in enumerate(self.cones):
                    #         plt.scatter(cone['x'], cone['y'], color='orange', s=10, label='Cone' if i == 0 else "")
                    #         plt.text(cone['x'], cone['y'] + 0.5, f'C{i+1}', ha='center', fontsize=9, color='darkorange')

                    #     # Plot fixed waypoint
                    #     if fixed_wp is not None:
                    #         plt.plot(fixed_wp[0], fixed_wp[1], 'ro', label='Fixed Waypoint')
                    #         plt.text(fixed_wp[0], fixed_wp[1] + 0.5, 'Fixed', fontsize=9, color='red')

                    #     plt.title('4-Cone Full Course Trajectory')
                    #     plt.xlabel('X')
                    #     plt.ylabel('Y')
                    #     plt.legend()
                    #     plt.axis('equal')
                    #     plt.grid(True)
                    #     plt.show()

                    # 4. Create trajectory
                    self.trajectory = to_gemstack_trajectory(x_full, y_full, v_full)
                else:
                    x, y, _, _, v, _, _ = trajectory_generation(init_state, final_state, waypoints=flex_wps)
                    self.trajectory = to_gemstack_trajectory(x, y, v)

                self.travelled_distance = 0.0

        return self.trajectory

    @staticmethod
    def get_future_point_on_trajectory(trajectory, vehicle_position, lookahead_distance=80.0):
        """
        Finds a point `lookahead_distance` ahead of the current vehicle position along the trajectory.
        """
        traj_points = trajectory.points
        current_pos = np.array(vehicle_position)

        # Step 1: Find the closest point on trajectory
        dists = [np.linalg.norm(current_pos - np.array([p[0], p[1]])) for p in traj_points]
        closest_idx = np.argmin(dists)

        # Step 2: Accumulate distance from closest_idx forward
        accumulated = 0.0
        heading = 0
        for i in range(closest_idx + 1, len(traj_points)):
            p1 = np.array([traj_points[i - 1][0], traj_points[i - 1][1]])
            p2 = np.array([traj_points[i][0], traj_points[i][1]])
            heading = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
            segment = np.linalg.norm(p2 - p1)
            accumulated += segment
            if accumulated >= lookahead_distance:
                return i, traj_points[i], heading  # Return future point

        p1 = np.array([traj_points[-2][0], traj_points[-2][1]])
        p2 = np.array([traj_points[-1][0], traj_points[-1][1]])
        heading = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
        # If not enough length, return the last point
        return -1, traj_points[-1], heading

    def get_current_cone_idx(self, cones, init_state, forward_dist=50.0, angle_thresh=np.pi):
        """
        Get the index of the nearest cone in front of the init_state.
        If a 'STANDING' cone is found, previous cones are flipped and returned with the index of the standing cone.
        
        Args:
            cones: List of cones.
            init_state: Dict with keys 'x', 'y', 'psi'.
            forward_dist: Max distance to search for cones ahead.
            angle_thresh: Angle threshold to filter cones roughly in front.
        
        Returns:
            idx: Index of the cone in front (after STANDING logic if needed).
            updated_cones: Possibly updated cones with flipped orientations.
        """
        pos = np.array([init_state['x'], init_state['y']])
        heading = init_state['psi']
        heading_vec = np.array([np.cos(heading), np.sin(heading)])

        best_idx = None
        next_idx = None # 2nd cone in front of init_state
        min_dist = float('inf')

        # Search in the list of cones, which one is nearest ahead of init_state
        for i, cone in enumerate(cones):
            if cone['id'] in self.visited_cone_ids:
                continue  # Skip already visited cones
            cone_pos = np.array([cone['x'], cone['y']])
            vec_to_cone = cone_pos - pos
            dist = np.linalg.norm(vec_to_cone)
            if dist > forward_dist:
                continue
            angle = np.arccos(np.clip(np.dot(heading_vec, vec_to_cone / (dist + 1e-8)), -1.0, 1.0))
            if angle < angle_thresh and dist < min_dist:
                next_idx = best_idx
                best_idx = i
                min_dist = dist
        print("getting cone idx.............")

        # If STANDING cone is ahead, flip previous cone directions
        if best_idx is not None and cones[best_idx]['orientation'] == 'STANDING':
            updated_cones = cones[:best_idx + 1] + [
                self.flip_cone_orientation(c) for c in cones[:best_idx][::-1]
            ] + cones[best_idx + 1:]
            print("updated cones: ", updated_cones)
            return best_idx, next_idx, updated_cones
        else:
            return best_idx, next_idx, cones

    @staticmethod
    def flip_cone_orientation(cone):
        """
        Flip cone orientation LEFT↔RIGHT
        """
        flipped = cone.copy()
        flipped['id'] = cone['id'] + 'flipped'
        if cone['orientation'] == 'LEFT':
            flipped['orientation'] = 'RIGHT'
        elif cone['orientation'] == 'RIGHT':
            flipped['orientation'] = 'LEFT'
        return flipped

########################################################################################################################
########################################################################################################################

# ------------ Test Code START --------------

# ======= Overall test cases to generate full trajectories with known cone positions =======
def test_4_cone_slalom():
    vehicle_state = {
        'position': [0.0, 0.0],
        'heading': 0.0,
        'velocity': 5.0
    }

    cones = [
        {'x': 10, 'y': 0.0, 'orientation': 'left'},
        {'x': 30, 'y': 1.0, 'orientation': 'right'},
        {'x': 50, 'y': 0.0, 'orientation': 'left'},
        {'x': 70, 'y': 1.0, 'orientation': 'standing'}
    ]

    flex_wps, fixed_wps = waypoint_search_optimization(vehicle_state, cones, search_attempts=1)

    current_pos = np.array(vehicle_state['position'])

    x_all, y_all = [], []

    for idx, (flex_wp, fixed_wp) in enumerate(zip(flex_wps, fixed_wps)):
        init_state = {
            'x': current_pos[0], 'y': current_pos[1], 'psi': vehicle_state['heading'],
            'c': 0.0, 'v': vehicle_state['velocity']
        }
        final_state = {
            'x': fixed_wp[0], 'y': fixed_wp[1], 'psi': vehicle_state['heading'], 'c': 0.0
        }
        x, y, psi, c, v, eps, final_error = trajectory_generation(init_state, final_state, waypoint=flex_wp)

        print(f"\nSegment {idx + 1}:")
        print(f"  Waypoint: {flex_wp}")
        print(f"  Final Errors:")
        for k, e in final_error.items():
            print(f"    {k}: {e:.4f}")

        x_all += list(x)
        y_all += list(y)

        plot_trajectory(x, y, v, c, eps, waypoint=flex_wp)

        current_pos = np.array(fixed_wp)

    # Plot overall trajectory
    plt.figure()
    plt.plot(x_all, y_all, label='Overall Trajectory')

    # Plot cones
    for i, cone in enumerate(cones):
        plt.scatter(cone['x'], cone['y'], color='orange', s=80, label='Cone' if i == 0 else "")
        plt.text(cone['x'], cone['y'] + 0.5, f'C{i+1}', ha='center', fontsize=9, color='darkorange')

    plt.title('4-Cone Full Course Trajectory')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

######################## 
if __name__ == "__main__":
    def plot_trajectory(x, y, v, c, eps, waypoint=None):
        plt.figure(figsize=(12, 10))

        # Trajectory plot
        plt.subplot(4, 1, 1)
        plt.plot(x, y, label="Trajectory")
        plt.scatter([x[0], x[-1]], [y[0], y[-1]], color='red', label="Start/End")
        
        if waypoint is not None:
            plt.scatter(*waypoint, color='purple', s=60, marker='X', label="Waypoint")
            plt.annotate("Waypoint", (waypoint[0], waypoint[1]), textcoords="offset points", xytext=(5,5), color='purple')

        plt.axis('equal')
        plt.ylabel("y (m)")
        plt.title("Trajectory")
        plt.grid(True)
        plt.legend()

        # Speed plot
        plt.subplot(4, 1, 2)
        plt.plot(v, label="Speed (v)", color="blue")
        plt.ylabel("Speed (m/s)")
        plt.grid(True)
        plt.legend()

        # Curvature plot
        plt.subplot(4, 1, 3)
        plt.plot(c, label="Curvature (c)", color="orange")
        plt.ylabel("Curvature (1/m)")
        plt.grid(True)
        plt.legend()

        # Curvature rate plot
        plt.subplot(4, 1, 4)
        plt.plot(eps, label="Curvature Rate (ε)", color="green")
        plt.xlabel("Step")
        plt.ylabel("ε (1/m²)")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()

    # --- Test Case 1: pass a cone in slalom ---
    def trajectory_generation_test1():
        # Init and final
        init_state = {'x': 0.0, 'y': 0.0, 'psi': 0.0, 'c': 0.0, 'v': 5.0}
        final_state = {'x': 15.0, 'y': 0.0, 'psi': np.pi / 20000000, 'c': np.pi / 20000000}
        waypoint = (8.0, 6.0)

        # Solve
        x, y, psi, c, v, eps, final_error = trajectory_generation(
            init_state, final_state, waypoint=waypoint
        )
        plot_trajectory(x, y, v, c, eps, waypoint)

        # Error
        print("\nFinal State Errors:")
        for k, e in final_error.items():
            print(f"{k}: {e:.6f}")

    # --- Test case 2: 90 degree turn ---
    def trajectory_generation_test2():
        # Init and final
        init_state = {'x': 0.0, 'y': 0.0, 'psi': 0.0, 'c': 0.0, 'v': 5.0}
        final_state = {'x': 15.0, 'y': 15.0, 'psi': np.pi / 2, 'c': np.pi / 2}
        waypoint = (13.0, 3.0)

        # Solve
        x, y, psi, c, v, eps, final_error = trajectory_generation(
            init_state, final_state, waypoint=waypoint
        )
        plot_trajectory(x, y, v, c, eps, waypoint)

        # Error
        print("\nFinal State Errors:")
        for k, e in final_error.items():
            print(f"{k}: {e:.6f}")

    #############
    #########################
    # --- Test Case ---
    def test_feasibility_check():
        N = 50
        y_traj = np.linspace(0, 10, N)
        psi_traj = np.linspace(0, 0.1, N)
        c_traj = np.linspace(0, 0.2, N)
        trajectory = list(zip(y_traj, psi_traj, c_traj))

        # Cone map near the path
        cone_map = [(5.0, 1.0), (10.0, 1.5), (15.0, 2.0), (25.0, 4.0), (25.0, 10.0), (16.0, 9.0), (40.0, 5.0)]

        # Run check
        feasible, collisions, x_vals, y_vals = feasibility_check(trajectory, cone_map)

        # Plot
        plt.figure(figsize=(10, 6))
        plt.plot(x_vals, y_vals, label="Trajectory", linewidth=2)
        for i, (cx, cy) in enumerate(cone_map):
            color = 'red' if i in collisions else 'green'
            plt.scatter(cx, cy, color=color, s=100, label=f'Cone {i}' if i == 0 else "")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title("Trajectory and Cone Map")
        plt.legend()
        plt.axis("equal")
        plt.grid(True)
        plt.show()

        print("Feasible:", feasible)
        print("Collisions with cones:", collisions)


# TODO 1: Compute target heading in waypoint_generation for different settings (slalom or course run)
# TODO 2: Implement replanning
def test_fixed_course():
    # steering angle c is relative to car heading!!
    vehicle_state = {
        'position': [-60.0, -39.0],
        'heading': np.pi * 2 / 3,
        'velocity': 0.0
    }

    cones = [
        {'x': -60.083, 'y': -33.118, 'orientation': '90turn'},
        {'x': -6.392, 'y': -5.147, 'orientation': '90turn'},
        {'x': -5.625, 'y': -13.637, 'orientation': '90turn'},
        {'x': -56.258, 'y': -41.080, 'orientation': '90turn'}
    ]

    current_pos = np.array(vehicle_state['position'])
    current_heading = vehicle_state['heading']
    current_steering = 0.0

    x_all, y_all = [], []

    for cone_idx, cone in enumerate(cones):
        print("current cone: ", cone)
        scenario, flex_wps, fixed_wp = waypoint_generate(vehicle_state, [cone])
        if not flex_wps or fixed_wp is None:
            continue

        # Setting target_heading in final_state
        if cone_idx > 0:
            prev_cone_position = np.array((cones[cone_idx - 1]['x'], cones[cone_idx - 1]['y']))
            current_cone_position = np.array((cones[cone_idx]['x'], cones[cone_idx]['y']))
            target_heading = np.arctan2(current_cone_position[1] - prev_cone_position[1],
                                      current_cone_position[0] - prev_cone_position[0]) - np.pi / 2 + np.pi / 18

        else: # no previous cone
            target_heading = current_heading - np.pi / 2 + np.pi / 18

        # mod to -pi to pi range
        target_heading = (target_heading + np.pi) % (2 * np.pi) - np.pi
        current_heading = (current_heading + np.pi) % (2 * np.pi) - np.pi

        if abs(target_heading - current_heading) > np.pi: 
            if target_heading < current_heading:
                target_heading += 2 * np.pi
            else:
                target_heading -= 2 * np.pi

        init_state = {
            'x': current_pos[0], 'y': current_pos[1], 'psi': current_heading,
            'c': current_steering, 'v': vehicle_state['velocity']
        }
        
        final_state = {
            'x': fixed_wp[0], 'y': fixed_wp[1], 'psi': target_heading, 'c': 0.0
        }

        estimated_N = int(((current_pos[0]-fixed_wp[0]) ** 2 + (current_pos[1]-fixed_wp[1]) ** 2) ** 0.5 / 0.7)

        x, y, psi, c, v, eps, _ = trajectory_generation(init_state, final_state, waypoints=flex_wps, N=estimated_N)
        x_all.extend(x)
        y_all.extend(y)

        plot_trajectory(x, y, v, c, eps)

        current_pos = np.array([x[-1], y[-1]])
        current_heading = psi[-1]
        current_steering = c[-1]

        vehicle_state = {
            'position': current_pos,
            'heading': current_heading,
            'velocity': v[-1]
        }

    # Plot overall trajectory
    plt.figure()
    plt.plot(x_all, y_all, label='Overall Trajectory')

    # Plot cones
    for i, cone in enumerate(cones):
        plt.scatter(cone['x'], cone['y'], color='orange', s=10, label='Cone' if i == 0 else "")
        plt.text(cone['x'], cone['y'] + 0.5, f'C{i+1}', ha='center', fontsize=9, color='darkorange')

    plt.title('4-Cone Full Course Trajectory')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    # --------------------------
    # Plotting Function
    # --------------------------
    def plot_results(vehicle_state, cones, wpt_flexes=None, wpt_fixed=None, scenario_label=""):
        plt.figure(figsize=(10, 5))
        ax = plt.gca()
        ax.set_aspect('equal')

        all_x = [cone['x'] for cone in cones] + [vehicle_state['position'][0]]
        all_y = [cone['y'] for cone in cones] + [vehicle_state['position'][1]]

        if wpt_flexes is not None:
            all_x += [pt[0] for pt in wpt_flexes]
            all_y += [pt[1] for pt in wpt_flexes]

        if wpt_fixed is not None:
            all_x.append(wpt_fixed[0])
            all_y.append(wpt_fixed[1])

        # Compute axis limits with padding
        padding = 2
        x_min, x_max = min(all_x) - padding, max(all_x) + padding
        y_min, y_max = min(all_y) - padding, max(all_y) + padding
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)

        # Plot cones
        for cone in cones:
            plt.scatter(cone['x'], cone['y'], c='orange', label='Cone' if cone == cones[0] else "")
            plt.text(cone['x'], cone['y'] + 0.5, cone['orientation'][0].upper(), fontsize=10, ha='center')

        # Plot vehicle
        vx, vy = vehicle_state['position']
        plt.plot(vx, vy, 'bo', label='Vehicle Start')
        plt.arrow(vx, vy, np.cos(vehicle_state['heading']) * 2, np.sin(vehicle_state['heading']) * 2,
                head_width=0.25, color='blue')

        # Plot flexible waypoint
        if wpt_flexes is not None:
            for wpt_flex in wpt_flexes:
                plt.plot(wpt_flex[0], wpt_flex[1], 'go', label='Flexible Waypoint')
                plt.text(wpt_flex[0], wpt_flex[1] + 0.5, 'Flex', fontsize=9, color='green')

        # Plot fixed waypoint
        if wpt_fixed is not None:
            plt.plot(wpt_fixed[0], wpt_fixed[1], 'ro', label='Fixed Waypoint')
            plt.text(wpt_fixed[0], wpt_fixed[1] + 0.5, 'Fixed', fontsize=9, color='red')

        plt.title(scenario_label)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)
        plt.legend()
        plt.show()
        return
    
    # --------------------------
    # Vehicle State Test Update 
    # --------------------------
    def drive(vehicle_state):
        # Update vehicle state
        vehicle_state['position'][0] += vehicle_state['velocity'] * np.cos(vehicle_state['heading'])
        vehicle_state['position'][1] += vehicle_state['velocity'] * np.sin(vehicle_state['heading'])
        return vehicle_state
    # --------------------------
    # Step 1: Generate Fake Cones
    # --------------------------
    def generate_test_cones(case='slalom'):
        if case == 'slalom':
            cones = []
            for i in range(3):
                x = (i+1) * 10
                y = 0 if i % 2 == 0 else 1
                orientation = 'left' if i % 2 == 0 else 'right'
                cones.append({'x': x, 'y': y, 'orientation': orientation})
            return cones

        elif case == 'u_turn':
            return [{'x': 10, 'y': 0, 'orientation': 'standing'}]

    # --------------------------
    # Step 2: Define Fake Vehicle
    # --------------------------    
    def get_test_vehicle_state(vehicle_state=None):
        if vehicle_state is not None:
            return vehicle_state
        return {
            'position': [0, 0],
            'heading': 0.0 * 180/np.pi,  # Facing right
            'velocity': 10.0
        }

    # --------------------------
    # Step 3: Run Scenario + Waypoints
    # --------------------------
    def test_waypoint_generation(case='slalom', test_loop=1):
        vehicle_state = get_test_vehicle_state()
        cones = generate_test_cones(case)
        
        for i in range(test_loop):
            scenario = scenario_check(vehicle_state, cones)
            scenario_label = f"Scenario: {scenario}"
            scenario, wpt_flexes, wpt_fixed  = waypoint_generate(vehicle_state, cones)
            plot_results(vehicle_state, cones, wpt_flexes, wpt_fixed, scenario_label)
            vehicle_state = drive(vehicle_state)
            if case == 'slalom':
                cones.pop(0)

    # --------------------------
    # Main for waypoint
    # --------------------------
    # test_waypoint_generation(case='slalom', test_loop=2)
    # test_waypoint_generation(case='u_turn')


    # --------------------------
    ### Combine Test
    # --------------------------
    def test_planning(case='slalom', test_loop=2):
        vehicle_state = get_test_vehicle_state()
        cones = generate_test_cones(case)
        
        for i in range(test_loop):
            # Way Points
            scenario = scenario_check(vehicle_state, cones)
            scenario_label = f"Scenario: {scenario}"
            scenario, wpt_flexes, wpt_fixed  = waypoint_generate(vehicle_state, cones)
            plot_results(vehicle_state, cones, wpt_flexes, wpt_fixed, scenario_label)
            # Trajectory
            # plot_trajectory(y_traj, psi_traj, c_traj, label="Generated trajectory")
            # plot_dynamics(psi_traj, c_traj, eps_traj)

            # Iterate
        vehicle_state = drive(vehicle_state)
        if case == 'slalom':
            cones.pop(0)
    # test_planning(case='slalom', test_loop=2)
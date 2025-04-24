import math
from utils import normalize_angle

_DUBINS_EPS = 1e-10
_LEFT_BIAS = 1e-3

def _mod2pi(x: float) -> float:
    """Wrap angle to [0, 2π)."""
    return x - 2*math.pi*math.floor(x/(2*math.pi))

def _dubins_LSL(a, b, d):
    """Left-Straight-Left path."""
    tmp = d + math.sin(a) - math.sin(b)
    p_sq = 2 + d * d - 2 * math.cos(a - b) + 2 * d * (math.sin(a) - math.sin(b))
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    t = _mod2pi(-a + math.atan2(math.cos(b) - math.cos(a), tmp))
    q = _mod2pi(b - math.atan2(math.cos(b) - math.cos(a), tmp))
    return t, p, q


def _dubins_RSR(a, b, d):
    """Right-Straight-Right path."""
    tmp = d - math.sin(a) + math.sin(b)
    p_sq = 2 + d * d - 2 * math.cos(a - b) + 2 * d * (math.sin(b) - math.sin(a))
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    t = _mod2pi(a - math.atan2(math.cos(a) - math.cos(b), tmp))
    q = _mod2pi(-b + math.atan2(math.cos(a) - math.cos(b), tmp))
    return t, p, q


def _dubins_LSR(a, b, d):
    """Left-Straight-Right path."""
    p_sq = -2 + d * d + 2 * math.cos(a - b) + 2 * d * (math.sin(a) + math.sin(b))
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(-math.cos(a) - math.cos(b), d + math.sin(a) + math.sin(b))
    t = _mod2pi(-a + tmp)
    q = _mod2pi(-b + tmp)
    return t, p, q


def _dubins_RSL(a, b, d):
    """Right-Straight-Left path."""
    p_sq = -2 + d * d + 2 * math.cos(a - b) - 2 * d * (math.sin(a) + math.sin(b))
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(math.cos(a) + math.cos(b), d - math.sin(a) - math.sin(b))
    t = _mod2pi(a - tmp)
    q = _mod2pi(b - tmp)
    return t, p, q


def _dubins_RLR(a, b, d):
    """Right-Left-Right path."""
    tmp = (6 - d * d + 2 * math.cos(a - b) + 2 * d * (math.sin(a) - math.sin(b))) / 8
    if abs(tmp) > 1:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp))
    t = _mod2pi(a - math.atan2(math.cos(a) - math.cos(b), d - math.sin(a) + math.sin(b)) + p / 2)
    q = _mod2pi(a - b - t + p)
    return t, p, q


def _dubins_LRL(a, b, d):
    """Left-Right-Left path."""
    tmp = (6 - d * d + 2 * math.cos(a - b) + 2 * d * (-math.sin(a) + math.sin(b))) / 8
    if abs(tmp) > 1:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp))
    t = _mod2pi(-a - math.atan2(math.cos(a) - math.cos(b), d + math.sin(a) - math.sin(b)) + p / 2)
    q = _mod2pi(b - a - t + p)
    return t, p, q

# All possible path types as (name, solver_function, (steer_directions))
_PATH_TYPES = [
    ("LSL", _dubins_LSL, ( 1,0, 1)),
    ("RSR", _dubins_RSR, (-1,0,-1)),
    ("LSR", _dubins_LSR, ( 1,0,-1)),
    ("RSL", _dubins_RSL, (-1,0, 1)),
    ("RLR", _dubins_RLR, (-1,0,-1)),
    ("LRL", _dubins_LRL, ( 1,0, 1)),
]

def dubins_shortest_path(start, goal, rho, left_turn_bias=_LEFT_BIAS):
    """
    Compute the shortest Dubins path between two poses.
    
    Args:
        start: (x, y, theta) start pose
        goal: (x, y, theta) goal pose
        rho: turning radius
        left_turn_bias: small bias to prefer left turns (for tie-breaking)
    
    Returns:
        Tuple of (path_type, (t, p, q), (steer_directions)) or None if no path exists
    """
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    D = math.hypot(dx, dy)
    if D < _DUBINS_EPS and abs(normalize_angle(goal[2]-start[2])) < _DUBINS_EPS:
        return ("TRIVIAL", (0.0, 0.0, 0.0), (0, 0, 0))
    
    d = D/rho
    theta = math.atan2(dy, dx)
    alpha = _mod2pi(start[2]-theta)
    beta  = _mod2pi(goal[2]-theta)
    
    best_cost = float("inf")
    best = None

    curvature_factor = min(1.0, D/(10*rho))
    
    for name, solver, dirs in _PATH_TYPES:
        sol = solver(alpha, beta, d)
        if sol is None: 
            continue
        
        t, p, q = sol
        
        # Calculate the raw path length
        raw_length = (t+p+q)*rho
        
        stretch_penalty = 0
        if name in ["LSL", "RSR", "LSR", "RSL"] and p > 2*rho and D < 5*rho:
            # Apply penalty to long straight segments for short distances
            stretch_penalty = p * 0.1 * (1 - curvature_factor)
        
        turn_bias = left_turn_bias if dirs[0]==1 else 0
        
        # Total cost with penalties and biases
        cost = raw_length + stretch_penalty - turn_bias
        
        if cost < best_cost:
            best_cost, best = cost, (name, (t, p, q), dirs)
            
    return best

def generate_dubins_path(start, goal, rho, step_size=0.5, left_turn_bias=_LEFT_BIAS):
    """
    Generate a list of (x, y, theta) states along the exact Dubins path.
    Arc-segments are sampled to ensure smooth heading changes.
    
    Note: Dubins paths only allow forward motion (no reverse).
    
    Args:
        start: (x, y, theta) start pose
        goal: (x, y, theta) goal pose
        rho: turning radius
        step_size: maximum distance between successive points
        left_turn_bias: small bias to prefer left turns
        
    Returns:
        List of (x, y, theta) poses along the path, or None if no path exists
    """
    res = dubins_shortest_path(start, goal, rho, left_turn_bias)
    if res is None:
        return None
    
    name, (t, p, q), dirs = res
    seg_lengths = (t*rho, p*rho, q*rho)
    path = [start]
    x, y, h = start

    def _segment(sign, seg_len):
        """Generate points along a segment (arc or straight line)."""
        nonlocal x, y, h
        travelled = 0.0
        arc_step = rho * math.radians(5.0)  # max 1° per step
        
        while travelled < seg_len - _DUBINS_EPS:
            # For straight segments use step_size, for arcs also cap by arc_step
            if sign == 0:  # Straight segment
                d = min(step_size, seg_len - travelled)
            else:  # Arc segment (left or right turn)
                d = min(step_size, seg_len - travelled, arc_step)
                
            travelled += d

            if sign == 0:  # Straight segment
                x_new = x + d*math.cos(h)
                y_new = y + d*math.sin(h)
                h_new = h
            else:  # Arc segment
                dh = (d/rho)*sign
                cx = x - rho*math.sin(h)*sign  # Center of turning circle
                cy = y + rho*math.cos(h)*sign
                h_new = normalize_angle(h + dh)
                x_new = cx + rho*math.sin(h_new)*sign
                y_new = cy - rho*math.cos(h_new)*sign

            yield x_new, y_new, h_new

    # Generate points for each segment
    for sign, seg_len in zip(dirs, seg_lengths):
        for (x, y, h) in _segment(sign, seg_len):
            path.append((x, y, h))
            
    # Force exact goal pose
    if (abs(path[-1][0]-goal[0]) > 1e-3 or
        abs(path[-1][1]-goal[1]) > 1e-3 or
        abs(normalize_angle(path[-1][2]-goal[2])) > math.radians(0.5)):
        path.append(goal)
        
    return path

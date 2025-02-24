import math

def quad_root(a : float, b : float, c : float) -> float:
    x1 = (-b + max(0,(b**2 - 4*a*c))**0.5)/(2*a)
    x2 = (-b - max(0,(b**2 - 4*a*c))**0.5)/(2*a)

    if math.isnan(x1): x1 = 0
    if math.isnan(x2): x2 = 0

    valid_roots = [n for n in [x1, x2] if not type(n) is complex]
    return valid_roots
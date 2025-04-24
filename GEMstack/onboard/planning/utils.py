"""
Utility functions for path planning.
"""
import math
import numpy as np

def normalize_angle(angle: float) -> float:
    """Wrap angle to (‑π, π]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

def bresenham(x0, y0, x1, y1):
    """Bresenham's line algorithm for grid traversal."""
    x0, y0, x1, y1 = map(int, map(round, (x0, y0, x1, y1)))
    dx, sx = abs(x1 - x0), (1 if x0 < x1 else -1)
    dy, sy = -abs(y1 - y0), (1 if y0 < y1 else -1)
    err = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy

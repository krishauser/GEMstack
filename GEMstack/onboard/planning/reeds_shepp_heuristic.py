import numpy as np
from ...utils import settings
from .reeds_shepp import *

precomputed = settings.get("planner.search_planner.precomputed", None)
HS, RESOLUTION, ANGLE_RESOLUTION, BOUNDS = None, None, None, None

def load_precomputed():
    try:
        h_dict = np.load(precomputed, allow_pickle=True).item()
        hs = h_dict['hs']
        resolution = h_dict['resolution']
        angle_resolution = h_dict['angle_resolution']
        bounds = h_dict['bounds']
    except:
        print("WARNING: Could not load precomputed heuristic")
        hs, resolution, angle_resolution, bounds = None, None, None, None
    return hs, resolution, angle_resolution, bounds

if precomputed is not None:
    HS, RESOLUTION, ANGLE_RESOLUTION, BOUNDS = load_precomputed()

def precompute(start, end):
    if HS is None:
        return path_length(get_optimal_path(start, end))
    x, y, phi = change_of_basis(start, end)
    x = round(x/RESOLUTION)
    y = round(y/RESOLUTION)
    phi = round(rad2deg(phi)/ANGLE_RESOLUTION) % HS.shape[2]
    x, y, phi = abs(x), abs(y), abs(phi)
    if x >= HS.shape[0] or y >= HS.shape[1]:
        return path_length(get_optimal_path(start, end))
    return HS[x, y, phi]



import numpy as np
import sys
sys.path.append('../../onboard/planning/')
from reeds_shepp import *
from tqdm import tqdm

# Precomputed optimal paths lengths
def precompute_heuristic():
    """
    Precompute the optimal paths for the 12 functions for all the possible
    combinations of start and end positions
    """
    resolution = 0.25
    angle_resolution = 5
    bounds = 25
    start = (0, 0, 0)
    shape = (int(bounds//resolution),
             int(bounds//resolution), 
             int(360//angle_resolution))
    hs = np.zeros(shape)
    for x in tqdm(range(shape[0])):
        for y in range(shape[1]):
            for phi in range(0, 360, angle_resolution):
                end = (x*resolution, y*resolution, deg2rad(phi))
                if abs(end[0]) < 1e-6 and abs(end[1]) < 1e-6 and abs(end[2]) < 1e-6:
                    hs[x, y, phi//angle_resolution] = 0
                    continue
                h = path_length(get_optimal_path(start,end))
                hs[x, y, phi//angle_resolution] = h
    return hs, resolution, angle_resolution, bounds

if __name__ == "__main__":
    precomputed = "../../knowledge/heuristics/reeds_shepp.npy"
    hs, resolution, angle_resolution, bounds = precompute_heuristic()
    h_dict = {}
    h_dict['hs'] = hs
    h_dict['resolution'] = resolution
    h_dict['angle_resolution'] = angle_resolution
    h_dict['bounds'] = bounds
    np.save(precomputed, h_dict)

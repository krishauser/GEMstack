#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

from GEMstack.state import Path, ObjectFrameEnum
from GEMstack.onboard.planning.hybrid_astar import Astar
import matplotlib.pyplot as plt
import numpy as np

def test_astar_planning():
    heuristic = lambda x,y: np.linalg.norm(np.array(x)-np.array(y))
    test_configs = ((0,0,0), (5,5,0)) #start and goal configurations

    planner = Astar(None, None, heuristic)

    test_traj = planner(*test_configs)
    print(test_traj)


if __name__ == '__main__':
    test_astar_planning()
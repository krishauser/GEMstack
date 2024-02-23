#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

from GEMstack.state import AgentState, Route, SceneState, Path, ObjectFrameEnum
from GEMstack.onboard.planning.pedestrian_avoidance_planning import longitudinal_plan,longitudinal_brake
import matplotlib.pyplot as plt
    
def test_pedestrian_avoidance():
    # TODO: Add test cases if you'd like
    pass

if __name__ == '__main__':
    test_pedestrian_avoidance()

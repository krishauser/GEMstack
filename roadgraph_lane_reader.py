import numpy as np
from typing import List
from GEMstack.state import Roadgraph, Path
from GEMstack.utils import serialization
import os
import matplotlib.pyplot as plt


def get_lane_points_from_roadgraph(roadgraph: Roadgraph) -> List:
    """
    Get all lane points from a roadgraph.
    Ouput: A list of [x, y, z], z = 0 for general cases.
    """
    lane_points = []
    for lane in roadgraph.lanes.values():
        for pts in lane.left.segments:
            for pt in pts:
                lane_points.append(pt)
        for pts in lane.right.segments:
            for pt in pts:
                lane_points.append(pt)
    return lane_points


if __name__ == "__main__":

    roadgraphfn = "GEMstack/knowledge/routes/summoning_roadgraph_sim.json"
    map_frame = 'start'
    # roadgraphfn = "GEMstack/knowledge/routes/summoning_roadgraph_highbay.json"
    # map_frame = 'global'
    base, ext = os.path.splitext(roadgraphfn)
    if ext in ['.json', '.yml', '.yaml']:
        with open(roadgraphfn, 'r') as f:
            roadgraph = serialization.load(f)
        map_type = 'roadgraph'
    elif ext in ['.csv', '.txt']:
        roadgraph = np.loadtxt(roadgraphfn, delimiter=',', dtype=float)
        map_type = 'pointlist'
        roadgraph = Path(frame=map_frame, points=roadgraph.tolist())
    else:
        raise ValueError("Unknown roadgraph file extension", ext)
    
    if map_type == 'roadgraph':
        lane_points = get_lane_points_from_roadgraph(roadgraph)
    elif map_type == 'pointlist':
        lane_points = roadgraph.points

    # np.savetxt('roadgraph_lane_points.txt', lane_points, delimiter=',')

    lane_points = np.array(lane_points)
    x = lane_points[:, 0] #*100
    y = lane_points[:, 1] #*100
    plt.scatter(x,y)
    plt.axis('equal')
    plt.show()
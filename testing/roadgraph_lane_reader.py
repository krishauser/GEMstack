import numpy as np
from typing import List
from GEMstack.state import Roadgraph, Path
from GEMstack.utils import serialization
import os
import matplotlib.pyplot as plt


def get_lane_points_from_roadgraph(roadgraph: Roadgraph) -> List:
    lane_points = []
    for lane in roadgraph.lanes.values():
        for pts in lane.left.segments:
            for pt in pts:
                lane_points.append(pt)
        for pts in lane.right.segments:
            for pt in pts:
                lane_points.append(pt)
    return lane_points


def get_region_points_from_roadgraph(roadgraph: Roadgraph) -> List:
    outlines = []
    for region in roadgraph.regions.values():
        outlines.extend(region.outline)
    return outlines


def plot_roadgraph(roadgraphfn, scale) -> None:
    base, ext = os.path.splitext(roadgraphfn)
    if ext in ['.json', '.yml', '.yaml']:
        with open(roadgraphfn, 'r') as f:
            roadgraph = serialization.load(f)
        map_type = 'roadgraph'
    elif ext in ['.csv', '.txt']:
        roadgraph = np.loadtxt(roadgraphfn, delimiter=',', dtype=float)
        map_type = 'pointlist'
    else:
        raise ValueError("Unknown roadgraph file extension", ext)

    if map_type == 'roadgraph':
        lane_points = get_lane_points_from_roadgraph(roadgraph)
        outlines = np.array(get_region_points_from_roadgraph(roadgraph))
    elif map_type == 'pointlist':
        lane_points = roadgraph.points

    # np.savetxt('roadgraph_lane_points.txt', lane_points, delimiter=',')

    lane_points = np.array(lane_points)
    x = lane_points[:, 0] * scale
    y = lane_points[:, 1] * scale
    plt.scatter(x, y)
    plt.axis('equal')
    if map_type == 'roadgraph':
        if len(outlines) > 0:
            outlines = np.array(outlines)
            p_x = outlines[:, 0]
            p_y = outlines[:, 1]
            plt.scatter(p_x, p_y)
    plt.show()


if __name__ == "__main__":

    roadgraphfn = "GEMstack/knowledge/routes/summoning_roadgraph_sim.json"
    scale = 1
    plot_roadgraph(roadgraphfn, scale)

    roadgraphfn = "GEMstack/knowledge/routes/summoning_roadgraph_highbay.json"
    scale = 1000
    plot_roadgraph(roadgraphfn, scale)

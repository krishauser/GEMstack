import numpy as np
from dataclasses import asdict, is_dataclass
from typing import List,Tuple,Any,Optional,Dict
import enum
from GEMstack.utils import serialization
import argparse


from GEMstack.state import Roadgraph, ObjectFrameEnum, RoadgraphLane, RoadgraphCurve, RoadgraphCurveEnum, Obstacle, ObstacleMaterialEnum


def segment_straight_line(start : Tuple[float,float,float], end : Tuple[float,float,float],
                          resolution : float):
    start = np.array(start, dtype=np.float64)
    end = np.array(end, dtype=np.float64)
    total_length = np.linalg.norm(end - start)

    if total_length == 0:
        return [tuple(start)]

    n_segments = max(1, int(np.ceil(total_length / resolution)))
    points = [(float(x), float(y), float(z))
        for i in range(n_segments + 1)
        for (x, y, z) in [start + (end - start) * (i / n_segments)]
    ]
    return [points]

def segment_arc(start : Tuple[float,float,float], end : Tuple[float,float,float], radius : float,
                direction : str, resolution : float):
    p1 = np.array(start[:2], dtype=np.float64)
    p2 = np.array(end[:2], dtype=np.float64)
    chord = p2 - p1
    chord_length = np.linalg.norm(chord)

    if chord_length > 2 * radius:
        raise ValueError("The distance between two points is larger than the radius.")

    # Calculate centers
    midpoint = (p1 + p2) / 2
    # vertical vectors in two directions
    perp = np.array([-chord[1], chord[0]])
    perp = perp / np.linalg.norm(perp)
    # distance to center
    h = np.sqrt(radius ** 2 - (chord_length / 2) ** 2)
    # two possible centers
    center1 = midpoint + h * perp
    center2 = midpoint - h * perp

    # choose suitable center
    def angle_diff(a1, a2):
        return (a2 - a1 + 2 * np.pi) % (2 * np.pi)

    def get_angle(center, pt):
        return np.arctan2(pt[1] - center[1], pt[0] - center[0])

    theta1_c1 = get_angle(center1, p1)
    theta2_c1 = get_angle(center1, p2)
    theta1_c2 = get_angle(center2, p1)
    theta2_c2 = get_angle(center2, p2)

    if direction == 'ccw':
        if angle_diff(theta1_c1, theta2_c1) < angle_diff(theta1_c2, theta2_c2):
            center = center1
            theta1, theta2 = theta1_c1, theta2_c1
        else:
            center = center2
            theta1, theta2 = theta1_c2, theta2_c2
    elif direction == 'cw':
        if angle_diff(theta2_c1, theta1_c1) < angle_diff(theta2_c2, theta1_c2):
            center = center1
            theta1, theta2 = theta1_c1, theta2_c1
        else:
            center = center2
            theta1, theta2 = theta1_c2, theta2_c2
    else:
        raise ValueError("Direction should be 'ccw' or 'cw'.")

    if direction == 'ccw':
        if theta2 <= theta1:
            theta2 += 2 * np.pi
        delta_theta = theta2 - theta1
    else:
        if theta2 >= theta1:
            theta2 -= 2 * np.pi
        delta_theta = theta1 - theta2

    arc_length = radius * delta_theta
    num_segments = max(1, int(np.ceil(arc_length / resolution)))
    angles = np.linspace(theta1, theta2, num_segments + 1)

    z = [start[2] + (end[2] - start[2]) * (i / num_segments) for i in range(num_segments + 1)]

    points = [(center[0] + radius * np.cos(a), center[1] + radius * np.sin(a), z[i]) for i, a in enumerate(angles)]

    return [points]


def create_straight_lane(left_back : Tuple[float,float,float], left_forward : Tuple[float,float,float],
                         right_back : Tuple[float,float,float], right_forward : Tuple[float,float,float],
                         begin_left: Tuple[float, float, float] = None, begin_right: Tuple[float, float, float] = None,
                         end_right: Tuple[float, float, float] = None, end_left: Tuple[float, float, float] = None,
                         resolution=0.1,
                         crossable=True,
                         route_name=''):
    lane = RoadgraphLane()
    left_boundary_points = segment_straight_line(left_back, left_forward, resolution=resolution)
    right_boundary_points = segment_straight_line(right_back, right_forward, resolution=resolution)
    lane.left = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY, segments=left_boundary_points)
    lane.right = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY, segments=right_boundary_points)

    if begin_left is not None and begin_right is not None:
        begin_boundary_points = segment_straight_line(begin_left, begin_right, resolution=resolution)
        lane.begin = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY, segments=begin_boundary_points)
    if end_right is not None and end_left is not None:
        end_boundary_points = segment_straight_line(end_right, end_left, resolution=resolution)
        lane.end = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY, segments=end_boundary_points)

    lane.crossable = crossable
    lane.route_name = route_name
    return lane


def create_arc_lane(left_back : Tuple[float,float,float], left_forward : Tuple[float,float,float], left_radius : float,
                    right_back : Tuple[float,float,float], right_forward : Tuple[float,float,float], right_radius : float,
                    direction : str = 'ccw',
                    begin_left : Tuple[float,float,float] = None, begin_right : Tuple[float,float,float] = None,
                    end_right : Tuple[float,float,float] = None, end_left : Tuple[float,float,float] = None,
                    resolution=0.1,
                    crossable=True,
                    route_name=''):
    lane = RoadgraphLane()
    left_boundary_points = segment_arc(left_back, left_forward, left_radius, direction, resolution=resolution)
    right_boundary_points = segment_arc(right_back, right_forward, right_radius, direction, resolution=resolution)
    lane.left = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY, segments=left_boundary_points)
    lane.right = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY, segments=right_boundary_points)

    if begin_left is not None and begin_right is not None:
        begin_boundary_points = segment_straight_line(begin_left, begin_right, resolution=resolution)
        lane.begin = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY, segments=begin_boundary_points)
    if end_right is not None and end_left is not None:
        end_boundary_points = segment_straight_line(end_right, end_left, resolution=resolution)
        lane.end = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY, segments=end_boundary_points)

    lane.crossable = crossable
    lane.route_name = route_name
    return lane


if __name__ == '__main__':
    filename = 'GEMstack/knowledge/routes/summoning_roadgraph.json'
    resolution = 0.4

    frame = ObjectFrameEnum.GLOBAL
    roadgraph = Roadgraph(frame=frame)

    # Create lane segments
    roadgraph.lanes['0'] = create_straight_lane(left_back=(0.0, 1.5, 0.0), left_forward=(20, 1.5, 0.0),
                                                right_back=(0.0, -1.5, 0.0), right_forward=(20.0, -1.5, 0.0),
                                                resolution=resolution,
                                                crossable=True,
                                                route_name='')
    roadgraph.lanes['1'] = create_arc_lane(left_back=(20.0, 1.5, 0.0), left_forward=(20.0, 9.5, 0.0), left_radius=4.0,
                                           right_back=(20.0, -1.5, 0.0), right_forward=(20.0, 12.5, 0.0), right_radius=7.0,
                                           direction='ccw',
                                           resolution=resolution,
                                           crossable=True,
                                           route_name='')
    roadgraph.lanes['2'] = create_straight_lane(left_back=(20.0, 9.5, 0.0), left_forward=(0.0, 9.5, 0.0),
                                                right_back=(20.0, 12.5, 0.0), right_forward=(0.0, 12.5, 0.0),
                                                resolution=resolution,
                                                crossable=True,
                                                route_name='')
    roadgraph.lanes['4'] = create_arc_lane(left_back=(0.0, 9.5, 0.0), left_forward=(0.0, 1.5, 0.0), left_radius=4.0,
                                           right_back=(0.0, 12.5, 0.0), right_forward=(0.0, -1.5, 0.0), right_radius=7.0,
                                           direction='ccw',
                                           resolution=resolution,
                                           crossable=True,
                                           route_name='')

    with open(filename, 'w') as f:
        serialization.save(roadgraph, f)

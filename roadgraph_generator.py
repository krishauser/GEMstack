import numpy as np
from dataclasses import asdict, is_dataclass
from typing import List, Tuple, Any, Optional, Dict
import enum

from GEMstack.utils import serialization
import argparse

from GEMstack.state import Roadgraph, ObjectFrameEnum, RoadgraphLane, RoadgraphCurve, RoadgraphCurveEnum, \
    Obstacle, ObstacleMaterialEnum, RoadgraphRegion, RoadgraphRegionEnum


def segment_straight_line(start: Tuple, end: Tuple,
                          resolution: float):
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

def segment_arc(start : Tuple, end : Tuple, radius : float,
                direction : str, resolution : float):
    p1 = np.array(start[:2], dtype=np.float64)
    p2 = np.array(end[:2], dtype=np.float64)
    chord = p2 - p1
    chord_length = np.linalg.norm(chord)

    if chord_length > 2 * radius + 1e-6:
        raise ValueError("The distance between two points is larger than the radius.")

    # Calculate centers
    midpoint = (p1 + p2) / 2
    # vertical vectors in two directions
    perp = np.array([-chord[1], chord[0]])
    perp = perp / np.linalg.norm(perp)
    # distance to center
    h = np.sqrt(abs(radius ** 2 - (chord_length / 2) ** 2))
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
                    crossable=False,
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


def create_lane(left_back : Tuple, left_forward : Tuple,
                right_back : Tuple, right_forward : Tuple,
                left_crossable : bool = False, left_type : str = 'line', left_radius : Optional[float] = None, left_direction = 'ccw',
                right_crossable : bool = False, right_type : str = 'line', right_radius : Optional[float] = None, right_direction = 'ccw',
                begin_left : Tuple = None, begin_right : Tuple = None,
                end_right : Tuple = None, end_left : Tuple = None,
                resolution=0.1, route_name=''):

    # Initiate lane
    lane = RoadgraphLane()

    # Create left curve
    if left_type == 'line':
        left_boundary_points = segment_straight_line(left_back, left_forward, resolution=resolution)
    elif left_type == 'arc':
        left_boundary_points = segment_arc(left_back, left_forward, left_radius, left_direction, resolution=resolution)
    else:
        raise ValueError('Unknown curve type.')

    # Create right curve
    if right_type == 'line':
        right_boundary_points = segment_straight_line(right_back, right_forward, resolution=resolution)
    elif right_type == 'arc':
        right_boundary_points = segment_arc(right_back, right_forward, right_radius, right_direction, resolution=resolution)
    else:
        raise ValueError('Unknown curve type.')

    lane.left = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY, segments=left_boundary_points, crossable=left_crossable)
    lane.right = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY, segments=right_boundary_points, crossable=right_crossable)

    if begin_left is not None and begin_right is not None:
        begin_boundary_points = segment_straight_line(begin_left, begin_right, resolution=resolution)
        lane.begin = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY, segments=begin_boundary_points, crossable=True)
    if end_right is not None and end_left is not None:
        end_boundary_points = segment_straight_line(end_right, end_left, resolution=resolution)
        lane.end = RoadgraphCurve(type=RoadgraphCurveEnum.LANE_BOUNDARY, segments=end_boundary_points, crossable=True)

    lane.route_name = route_name

    return lane


if __name__ == '__main__':
    resolution = 0.4

    filename = 'GEMstack/knowledge/routes/summoning_roadgraph_sim.json'
    frame = ObjectFrameEnum.START
    roadgraph = Roadgraph(frame=frame)

    # Create lane segments
    roadgraph.lanes['lane_0'] = create_lane(left_back=(0.0, 1.49, 0.0), left_forward=(30, 1.49, 0.0),
                                            right_back=(0.0, -1.5, 0.0), right_forward=(30, -1.5, 0.0),
                                            left_crossable=False,
                                            right_crossable=False,
                                            resolution=resolution,
                                            route_name=''
                                            )
    roadgraph.lanes['arc_1_1'] = create_lane(left_back=(30.0, 1.5, 0.0), left_forward=(37.0, 8.5, 0.0),
                                           right_back=(30.0, -1.5, 0.0), right_forward=(40.0, 8.5, 0.0),
                                           left_crossable=False, left_type='arc', left_radius=7.0, left_direction='ccw',
                                           right_crossable=False, right_type='arc', right_radius=10.0, right_direction='ccw',
                                           resolution=resolution,
                                           route_name=''
                                           )
    roadgraph.lanes['arc_1_2'] = create_lane(left_back=(37.0, 8.5, 0.0), left_forward=(33.0, 12.5, 0.0),
                                           right_back=(40.0, 8.5, 0.0), right_forward=(33.0, 15.5, 0.0),
                                           left_crossable=False, left_type='arc', left_radius=4.0, left_direction='ccw',
                                           right_crossable=False, right_type='arc', right_radius=7.0, right_direction='ccw',
                                           resolution=resolution,
                                           route_name=''
                                           )
    roadgraph.lanes['t_1_1'] = create_lane(left_back=(33.0, 12.5, 0.0), left_forward=(29.0, 8.5, 0.0),
                                            right_back=(33.0, 15.5, 0.0), right_forward=(27.5, 15.5, 0.0),
                                            left_crossable=False, left_type='arc', left_radius=4.0, left_direction='ccw',
                                            right_crossable=False,
                                            resolution=resolution,
                                            route_name=''
                                            )
    roadgraph.lanes['t_1_2'] = create_lane(left_back=(26.0, 8.5, 0.0), left_forward=(22.0, 12.5, 0.0),
                                            right_back=(27.5, 15.5, 0.0), right_forward=(22.0, 15.5, 0.0),
                                            left_crossable=False, left_type='arc', left_radius=4.0, left_direction='ccw',
                                            right_crossable=False,
                                            resolution=resolution,
                                            route_name=''
                                            )
    roadgraph.lanes['t_1_3'] = create_lane(left_back=(29.0, 8.5, 0.0), left_forward=(22.0, 1.5, 0.0),
                                            right_back=(26.0, 8.5, 0.0), right_forward=(22.0, 4.5, 0.0),
                                            left_crossable=False, left_type='arc', left_radius=7.0, left_direction='cw',
                                            right_crossable=False, right_type='arc', right_radius=4.0, right_direction='cw',
                                            resolution=resolution,
                                            route_name=''
                                            )
    roadgraph.lanes['line_1'] = create_lane(left_back=(22.0, 1.5, 0.0), left_forward=(8.0, 1.5, 0.0),
                                            right_back=(22.0, 4.5, 0.0), right_forward=(8.0, 4.5, 0.0),
                                            left_crossable=False,
                                            right_crossable=False,
                                            resolution=resolution,
                                            route_name=''
                                            )
    roadgraph.lanes['t_2_1'] = create_lane(left_back=(8.0, 1.5, 0.0), left_forward=(1.0, 8.5, 0.0),
                                            right_back=(8.0, 4.5, 0.0), right_forward=(4.0, 8.5, 0.0),
                                            left_crossable=False, left_type='arc', left_radius=7.0, left_direction='cw',
                                            right_crossable=False, right_type='arc', right_radius=4.0, right_direction='cw',
                                            resolution=resolution,
                                            route_name=''
                                            )
    roadgraph.lanes['t_2_2'] = create_lane(left_back=(8.0, 12.5, 0.0), left_forward=(4.0, 8.5, 0.0),
                                            right_back=(8.0, 15.5, 0.0), right_forward=(2.5, 15.5, 0.0),
                                            left_crossable=False, left_type='arc', left_radius=4.0, left_direction='ccw',
                                            right_crossable=False,
                                            resolution=resolution,
                                            route_name=''
                                            )
    roadgraph.lanes['t_2_3'] = create_lane(left_back=(1.0, 8.5, 0.0), left_forward=(-3.0, 12.5, 0.0),
                                            right_back=(2.5, 15.5, 0.0), right_forward=(-3.0, 15.5, 0.0),
                                            left_crossable=False, left_type='arc', left_radius=4.0, left_direction='ccw',
                                            right_crossable=False,
                                            resolution=resolution,
                                            route_name=''
                                            )
    roadgraph.lanes['line_2'] = create_lane(left_back=(22.0, 12.5, 0.0), left_forward=(8.0, 12.5, 0.0),
                                            right_back=(22.0, 15.5, 0.0), right_forward=(8.0, 15.5, 0.0),
                                            left_crossable=False,
                                            right_crossable=False,
                                            resolution=resolution,
                                            route_name=''
                                            )
    roadgraph.lanes['arc_2_1'] = create_lane(left_back=(-3.0, 12.5, 0.0), left_forward=(-7.0, 8.5, 0.0),
                                           right_back=(-3.0, 15.5, 0.0), right_forward=(-10.0, 8.5, 0.0),
                                           left_crossable=False, left_type='arc', left_radius=4.0, left_direction='ccw',
                                           right_crossable=False, right_type='arc', right_radius=7.0, right_direction='ccw',
                                           resolution=resolution,
                                           route_name=''
                                           )
    roadgraph.lanes['arc_2_2'] = create_lane(left_back=(-7.0, 8.5, 0.0), left_forward=(0.0, 1.5, 0.0),
                                           right_back=(-10.0, 8.5, 0.0), right_forward=(0.0, -1.5, 0.0),
                                           left_crossable=False, left_type='arc', left_radius=7.0, left_direction='ccw',
                                           right_crossable=False, right_type='arc', right_radius=10.0, right_direction='ccw',
                                           resolution=resolution,
                                           route_name=''
                                           )

    # Parking lots
    roadgraph.regions['highbay_parallel_parking_lot_1'] = RoadgraphRegion(type=RoadgraphRegionEnum.PARKING_LOT,
                                                                          outline=[(30.0, 12.5), (30.0, 15), (0.0, 15), (0.0, 12.5)])
  

    with open(filename, 'w') as f:
        serialization.save(roadgraph, f)
        print('File saved:', filename)


    filename = 'GEMstack/knowledge/routes/summoning_roadgraph_highbay.json'
    frame = ObjectFrameEnum.GLOBAL
    roadgraph = Roadgraph(frame=frame)

    lon_ratio = (-88.235527 + 88.236129) / 51.34  # lon / m
    lat_ratio = (40.092819 - 40.092741) / 8.66    # lat / m

    # Create lane segments
    roadgraph.lanes['highbay_outer_lane'] = create_straight_lane(left_back=(-88.236129, 40.092741 + lat_ratio * 1.5, 0.0), left_forward=(-88.235527, 40.092741 + lat_ratio * 1.5, 0.0),
                                                right_back=(-88.236129, 40.092741 - lat_ratio * 1.5, 0.0), right_forward=(-88.235527, 40.092741 - lat_ratio * 1.5, 0.0),
                                                resolution=resolution,
                                                crossable=False,
                                                route_name='')
    roadgraph.lanes['highbay_east_u_turn'] = create_arc_lane(left_back=(-88.235527, 40.092741 + lat_ratio * 1.5, 0.0), left_forward=(-88.235527, 40.092819 - lat_ratio * 1.5, 0.0),
                                           left_radius=(40.092819 - 40.092741 - lat_ratio * 3) / 2,
                                           right_back=(-88.235527, 40.092741 - lat_ratio * 1.5, 0.0), right_forward=(-88.235527, 40.092819 + lat_ratio * 1.5, 0.0),
                                           right_radius=(40.092819 - 40.092741 + lat_ratio * 3) / 2,
                                           direction='ccw',
                                           resolution=resolution,
                                           crossable=False,
                                           route_name='')
    roadgraph.lanes['highbay_inner_lane'] = create_straight_lane(left_back=(-88.235527, 40.092819 - lat_ratio * 1.5, 0.0), left_forward=(-88.236129, 40.092819 - lat_ratio * 1.5, 0.0),
                                                right_back=(-88.235527, 40.092819 + lat_ratio * 1.5, 0.0), right_forward=(-88.236129, 40.092819 + lat_ratio * 1.5, 0.0),
                                                resolution=resolution,
                                                crossable=False,
                                                route_name='')
    roadgraph.lanes['highbay_west_u_turn'] = create_arc_lane(left_back=(-88.236129, 40.092819 - lat_ratio * 1.5, 0.0), left_forward=(-88.236129, 40.092741 + lat_ratio * 1.5, 0.0),
                                           left_radius=(40.092819 - 40.092741 - lat_ratio * 3) / 2,
                                           right_back=(-88.236129, 40.092819 + lat_ratio * 1.5, 0.0), right_forward=(-88.236129, 40.092741 - lat_ratio * 1.5, 0.0),
                                           right_radius=(40.092819 - 40.092741 + lat_ratio * 3) / 2,
                                           direction='ccw',
                                           resolution=resolution,
                                           crossable=False,
                                           route_name='')

    roadgraph.regions['highbay_parallel_parking_slot_1'] = RoadgraphRegion(type=RoadgraphRegionEnum.PARKING_LOT,
                                                                           outline=[(-88.235527, 40.092819 + lat_ratio * 1.5),
                                                                                    (-88.235527, 40.092819 + lat_ratio * 4),
                                                                                    (-88.236129, 40.092819 + lat_ratio * 4),
                                                                                    (-88.236129, 40.092819 + lat_ratio * 1.5)])


    with open(filename, 'w') as f:
        serialization.save(roadgraph, f)
        print('File saved:', filename)


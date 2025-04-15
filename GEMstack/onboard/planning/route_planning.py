from typing import List
from ..component import Component
from ...utils import serialization
from ...state import Route,ObjectFrameEnum
import os
import numpy as np
import math

class StaticRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, routefn : str, frame : str = 'start'):
        self.routefn = routefn
        base, ext = os.path.splitext(routefn)
        if ext in ['.json','.yml','.yaml']:
            with open(routefn,'r') as f:
                self.route = serialization.load(f)
        elif ext == '.csv':
            waypoints = np.loadtxt(routefn,delimiter=',',dtype=float)
            if waypoints.shape[1] == 3:
                waypoints = waypoints[:,:2]
            if frame == 'start':
                self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())
            elif frame == 'global':
                self.route = Route(frame=ObjectFrameEnum.GLOBAL,points=waypoints.tolist())
            elif frame == 'cartesian':
                self.route = Route(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,points=waypoints.tolist())
            else:
                raise ValueError("Unknown route frame {} must be start, global, or cartesian".format(frame))
        else:
            raise ValueError("Unknown route file extension",ext)

    def state_inputs(self):
        return []

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 1.0

    def update(self):
        return self.route


def is_inside_geofence(x, y, xmin, xmax, ymin, ymax):
    return xmin < x < xmax and ymin < y < ymax


def max_visible_arc(circle_center, radius, geofence):
    xc, yc = circle_center
    (xmin, ymin), (xmax, ymax) = geofence

    angles = np.linspace(0, 2 * np.pi, 500, endpoint=False)
    arc_segments = []
    curr_segment = []

    first_inside = last_inside = False

    for i, theta in enumerate(angles):
        x = xc + radius * np.cos(theta)
        y = yc + radius * np.sin(theta)

        inside = is_inside_geofence(x, y, xmin, xmax, ymin, ymax)

        if i == 0:
            first_inside = inside
        if i == len(angles) - 1:
            last_inside = inside

        if inside:
            curr_segment.append((x, y))
        else:
            if curr_segment:
                arc_segments.append(curr_segment)
                curr_segment = []

    if curr_segment:
        arc_segments.append(curr_segment)

    # If arc wraps around from 2π back to 0, combine first and last segments
    if first_inside and last_inside and len(arc_segments) > 1:
        arc_segments[0] = arc_segments[-1] + arc_segments[0]
        arc_segments.pop()

    if not arc_segments:
        return []

    max_arc = max(arc_segments, key=len)
    for i in range(max_arc):
        max_arc[i].append(heading_on_circle(xc,yc,max_arc[0], max_arc[1]))
    return max_arc

def heading_on_circle(cx, cy, px, py):
    dx = px - cx
    dy = py - cy
    tx = -dy
    ty = dx
    return math.atan2(ty, tx)  # Heading in radians


def check_point_exists(server_url="http://localhost:8000"):
    try:
        response = requests.get(f"{server_url}/point")
        response.raise_for_status()
        points = response.json().get("points", [])
        
        for point in points:
            return True, [[point[0]["x"], point[0]["y"]],[point[1]["x"], point[1]["y"]]]
        return False, []

    except requests.exceptions.RequestException as e:
        print("Error contacting server:", e)
        return False, []

class InspectRoutePlanner(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self, geofence_area, frame : str = 'start'):
        # if frame == 'start':
        #     self.route = Route(frame=ObjectFrameEnum.START,points=waypoints.tolist())
        # elif frame == 'global':
        #     self.route = Route(frame=ObjectFrameEnum.GLOBAL,points=waypoints.tolist())
        # elif frame == 'cartesian':
        #     self.route = Route(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,points=waypoints.tolist())
        # self.inspection_area = bounding_box
        self.geofence_area = geofence_area

    def state_inputs(self):
        return ['mission', 'vehicle']

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 1.0

    def update(self, mission, vehicle):
        if mission == "IDLE":
            points_found = False
            points_found, pts = check_point_exists()
            if points_found:
                self.bounding_box = pts
                print(self.state_list[self.index+1])
                self.mission = self.state_list[self.index+1]
                self.index += 1
                print("CHANGING STATES", self.mission)
                self.start = [vehicle.pose.x,vehicle.pose.y]
            self.circle_center = [(self.inspection_area[0][0]+self.inspection_area[1][0])/2, (self.inspection_area[0][1]+self.inspection_area[1][1])/2]
            self.radius = ((self.inspection_area[0][0]+self.inspection_area[1][0])**2 + (self.inspection_area[0][1]+self.inspection_area[1][1])**2)**0.5/2
            self.inspection_route = max_visible_arc(self.circle_center, self.radius, geofence_area)
        elif mission == "NAV":
            planner.rrt_final_point(self.inspection_route[0])
        elif mission == "INSPECT":
            planner.route_as_input(self.inspection_route)
        elif mission == "FINISH":
            planner.rrt_final_point(self.start)
    
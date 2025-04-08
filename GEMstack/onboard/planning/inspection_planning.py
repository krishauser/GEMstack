import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Union
from ..component import Component
from ...state import (
    AllState,
    VehicleState,
    EntityRelation,
    EntityRelationEnum,
    Path,
    Trajectory,
    Route,
    ObjectFrameEnum,
    AgentState,
)
from ...utils import serialization
from ...mathutils.transforms import vector_madd
from ...mathutils.quadratic_equation import quad_root
import requests


import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
from scipy.optimize import minimize
from .longitudinal_planning import (
    longitudinal_plan,
    detect_collision,
    longitudinal_brake,
)


def is_inside_geofence(x, y, xmin, xmax, ymin, ymax):
    return xmin + 0.2 < x < xmax - 0.2 and ymin + 0.2 < y < ymax - 0.2


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
    return max_arc


def plot_circle_with_geofence_and_arc(circle_center, radius, geofence, rem_pts):
    fig, ax = plt.subplots()

    # Plot geofence
    (xmin, ymin), (xmax, ymax) = geofence
    rect = plt.Rectangle(
        (xmin, ymin),
        xmax - xmin,
        ymax - ymin,
        linewidth=2,
        edgecolor="red",
        facecolor="none",
    )
    ax.add_patch(rect)

    rem_pts = np.array(rem_pts)
    ax.plot(
        rem_pts[:, 0], rem_pts[:, 1], color="blue", linewidth=3, label="Max Valid Arc"
    )

    ax.set_aspect("equal")
    ax.grid(True)
    plt.legend()
    plt.show()


circle_center = (2, 6)
radius = 2
geofence = ((0, 0), (10, 10))


# rem_points = max_visible_arc(circle_center, radius, geofence)
# plot_circle_with_geofence_and_arc(circle_center, radius, geofence, rem_points)


def check_point_exists(server_url="http://localhost:8000"):
    try:
        response = requests.get(f"{server_url}/api/inspect")
        response.raise_for_status()
        points = response.json().get("coords", [])

        for point in points:
            return True, [point["lng"], point["lat"]]
        return False, []

    except requests.exceptions.RequestException as e:
        print("Error contacting server:", e)
        return False, []


class InspectionPlanner(Component):
    """Follows the given route.  Brakes if you have to yield or
    you are at the end of the route, otherwise accelerates to
    the desired speed.
    """

    def __init__(self, mode: str = "real", params: dict = {"state_machine": []}):
        self.route_progress = None
        self.t_last = None
        self.acceleration = 2
        self.desired_speed = 1
        self.deceleration = 2.0

        self.min_deceleration = 1.0
        self.max_deceleration = 8.0

        self.mode = mode
        self.planner = "dt"
        self.state_list = params["state_machine"]
        self.goal = [10, 0]
        self.inspect_goal = [15, 0]
        self.index = 0
        self.mission = self.state_list[self.index]
        self.x = 0
        print(self.mission)

    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ["trajectory"]

    def rate(self):
        return 10

    def update(self, state: AllState):
        start_time = time.time()

        vehicle = state.vehicle  # type: VehicleState
        route = state.route  # type: Route
        t = state.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last

        # Position in vehicle frame (Start (0,0) to (15,0))
        curr_x = vehicle.pose.x
        curr_y = vehicle.pose.y
        curr_v = vehicle.v

        abs_x = curr_x + state.start_vehicle_pose.x
        abs_y = curr_y + state.start_vehicle_pose.y

        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist, closest_parameter = state.route.closest_point_local(
            (curr_x, curr_y), [self.route_progress - 5.0, self.route_progress + 5.0]
        )
        self.route_progress = closest_parameter

        lookahead_distance = max(10, curr_v**2 / (2 * self.deceleration))
        route_with_lookahead = route.trim(
            closest_parameter, closest_parameter + lookahead_distance
        )
        print("Lookahead distance:", lookahead_distance)
        print(self.mission)

        route_to_end = route.trim(closest_parameter, len(route.points) - 1)

        if self.mission == "IDLE":
            points_found = False
            points_found, pts = check_point_exists()
            if points_found:
                self.goal = pts
                print(self.state_list[self.index + 1])
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)

        elif self.mission == "NAV":
            print("Top left corner of bounding box: ", self.goal)
            print(abs_x, abs_y)

            if abs(abs_x - self.goal[0]) <= 0.1 and abs(abs_y - self.goal[1]) <= 0.1:
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)

            should_yield = False
            yield_deceleration = 0.0

            print("Current Speed: ", curr_v)

            for r in state.relations:
                if r.type == EntityRelationEnum.YIELDING and r.obj1 == "":
                    # get the object we are yielding to
                    obj = state.agents[r.obj2]

                    detected, deceleration = detect_collision(
                        abs_x,
                        abs_y,
                        curr_v,
                        obj,
                        self.min_deceleration,
                        self.max_deceleration,
                        self.acceleration,
                        self.desired_speed,
                    )
                    if isinstance(deceleration, list):
                        print("@@@@@ INPUT", deceleration)
                        time_collision = deceleration[1]
                        distance_collision = deceleration[0]
                        b = 3 * time_collision - 2 * curr_v
                        c = curr_v**2 - 3 * distance_collision
                        desired_speed = (-b + (b**2 - 4 * c) ** 0.5) / 2
                        deceleration = 1.5
                        print("@@@@@ YIELDING", desired_speed)
                        route_yield = route.trim(
                            closest_parameter, closest_parameter + distance_collision
                        )
                        traj = longitudinal_plan(
                            route_yield,
                            self.acceleration,
                            deceleration,
                            desired_speed,
                            curr_v,
                            self.planner,
                        )
                        return traj
                    else:
                        if detected and deceleration > 0:
                            yield_deceleration = deceleration
                            should_yield = True

                    print("should yield: ", should_yield)

            should_accelerate = not should_yield and curr_v < self.desired_speed

            # choose whether to accelerate, brake, or keep at current velocity
            if should_accelerate:
                traj = longitudinal_plan(
                    route_to_end,
                    self.acceleration,
                    self.deceleration,
                    self.desired_speed,
                    curr_v,
                    self.planner,
                )
            elif should_yield:
                traj = longitudinal_brake(route_to_end, yield_deceleration, curr_v)
            else:
                traj = longitudinal_plan(
                    route_to_end,
                    0.0,
                    self.deceleration,
                    self.desired_speed,
                    curr_v,
                    self.planner,
                )

            return traj

        elif self.mission == "INSPECT":

            if (
                abs(abs_x - self.inspect_goal[0]) <= 0.1
                and abs(abs_y - self.inspect_goal[1]) <= 0.1
            ):
                self.mission = self.state_list[self.index + 1]
                self.index += 1
                print("CHANGING STATES", self.mission)

            traj = longitudinal_plan(
                route_to_end,
                self.acceleration,
                self.deceleration,
                self.desired_speed,
                curr_v,
                self.planner,
            )

            return traj

from ...state import AgentState, AgentEnum, EntityRelation, EntityRelationEnum, ObjectFrameEnum, VehicleState
from ..component import Component
from typing import List, Dict

import numpy as np


DEBUG = False    # Set to False to disable debug output

""" Vehicle Configuration """
GEM_MODEL = 'e4'    # e2 or e4
buffer_x, buffer_y = 3, 1
if GEM_MODEL == 'e2':
    # e2 axle dimensions, refer to GEMstack/knowledge/vehicle/model/gem_e2.urdf
    size_x, size_y = 2.53, 1.35         # measured from base_link.STL
    lx_f2r = 0.88 + 0.87                # distance between front axle and rear axle, from gem_e2.urdf wheel_links
    ly_axle = 0.6 * 2                   # length of axle
    l_rear_axle_to_front = 1.28 + 0.87  # measured from base_link.STL
    l_rear_axle_to_rear = size_x - l_rear_axle_to_front
elif GEM_MODEL == 'e4':
    # e4 axle dimensions, refer to GEMstack/knowledge/vehicle/model/gem_e4.urdf
    size_x, size_y = 3.35, 1.35         # measured from base_link.STL
    lx_f2r = 1.2746 + 1.2904            # distance between front axle and rear axle, from gem_e4.urdf wheel_links
    ly_axle = 0.6545 * 2                # length of front and rear axles, from gem_e4.urdf wheel_links
    l_rear_axle_to_front = 1.68 + 1.29  # measured from base_link.STL
    l_rear_axle_to_rear = size_x - l_rear_axle_to_front
# add buffer to outer dimensions, defined by half of the x, y buffer area dimensions
buffer_front = l_rear_axle_to_front + buffer_x
buffer_rear = -(l_rear_axle_to_rear + buffer_x)
buffer_left = size_y / 2 + buffer_y
buffer_right = -(size_y / 2 + buffer_y)
# comfortable deceleration for lookahead time calculation
comfort_decel = 2.0


class PedestrianYielder(Component):
    """Yields for all pedestrians in the scene.

    Result is stored in the relations graph.
    """

    def rate(self):
        return None

    def state_inputs(self):
        return ['agents', 'vehicle']

    def state_outputs(self):
        return ['relations']

    def update(self, agents: Dict[str, AgentState], vehicle: VehicleState) -> List[EntityRelation]:
        if DEBUG:
            print("PedestrianYielder vehicle pose:", vehicle.pose, vehicle.v)
        res = []
        for n, a in agents.items():
            if DEBUG:
                print(f"[DEBUG] PedestrianYielder.update: Agent:", a.pose, a.velocity)

            """ collision estimation based on agent states in vehicle frame """
            if a.type == AgentEnum.PEDESTRIAN:
                check, t_min, min_dist, pt_min = check_collision_in_vehicle_frame(a, vehicle)
                if DEBUG:
                    print(
                        f"[DEBUG] ID {n}, relation:{check}, minimum distance:{min_dist}, time to min_dist: {t_min}, point of min_dist:{pt_min}")
                # collision may occur, slow down
                if check == 'YIELD':
                    res.append(EntityRelation(type=EntityRelationEnum.YIELDING, obj1='', obj2=n))
                # collision in a short time, emergency stop
                elif check == 'STOP':
                    res.append(EntityRelation(type=EntityRelationEnum.STOPPING_AT, obj1='', obj2=n))

        return res


""" Planning in vehicle frame without waypoints """
def check_collision_in_vehicle_frame(agent: AgentState, vehicle: VehicleState):
    xp, yp = agent.pose.x, agent.pose.y
    vx, vy = agent.velocity[:2]
    xv = vehicle.pose.x
    yv = vehicle.pose.y
    yaw = vehicle.pose.yaw
    vel = vehicle.v
    # time to stop from current velocity with comfortable deceleration
    t_look = vel / comfort_decel
    # calculate relative pedestrian position and velocity in vehicle frame
    if agent.pose.frame == ObjectFrameEnum.CURRENT:
        # xp, yp, vx, vy are already in vehicle frame
        pass
    elif agent.pose.frame == ObjectFrameEnum.START:
        # convert xp, yp, vx, vy to vehicle frame
        R = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]], dtype=np.float32)
        dx, dy = xp - xv, yp - yv
        dvx, dvy = vx - vel * np.cos(yaw), vy - vel * np.sin(yaw)
        # pedestrian pose and velocity in vehicle frame
        xp, yp = np.dot(R, np.array([dx, dy]))
        vx, vy = np.dot(R, np.array([dvx, dvy]))

    # If pedestrian already in buffer area
    if buffer_rear <= xp <= buffer_front and buffer_right <= yp <= buffer_left:
        return 'STOP', 0, 0, (xp, yp)
    t_min, min_dist, pt_min = find_min_distance_and_time(xp, yp, vx, vy)
    # if the minimum distance between the position and the buffer area is less than 0, than a collision is expected
    if min_dist is not None:
        if min_dist <= 0:
            if t_min <= 0:
                check = 'STOP'
            elif t_min <= t_look:
                check = 'YIELD'
            else:
                check = 'RUN'
        else:
            check = 'RUN'
    else:
        check = 'RUN'
    return check, t_min, min_dist, pt_min


def find_min_distance_and_time(xp, yp, vx, vy):
    # path function: Ax + By + C = vy * x - vx * y + (yp * vx - xp * vy) = 0
    vx = vx if vx != 0 else 1e-6
    vy = vy if vy != 0 else 1e-6
    A = vy
    B = -vx
    C = yp * vx - xp * vy

    def point_to_line(x0, y0, A, B, C):
        # calculate the shortest distance from a point (x0, y0) to the line Ax + By + C = 0 """
        numerator = abs(A * x0 + B * y0 + C)
        denominator = np.sqrt(A ** 2 + B ** 2)
        x_foot = x0 - (A * (A * x0 + B * y0 + C)) / denominator
        y_foot = y0 - (B * (A * x0 + B * y0 + C)) / denominator
        dist = numerator / denominator if denominator != 0 else np.inf
        return dist, (x_foot, y_foot)

    def point_dist(x1, y1, x2, y2):
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    """ Compute intersections at the front, left and right edge """
    if xp >= buffer_front:
        # front edge intersection: x = x_buff = xt = xp + vx * t_f
        t_f = (buffer_front - xp) / vx
        yt = yp + vy * t_f
        if t_f < 0:  # object moving away with higher speed than vehicle, start point has minimum distance
            t_min = 0
            pt_min = xp, yp
            if buffer_right <= yp <= buffer_left:
                min_dist = xp - buffer_front  # the distance to the front edge
            elif yt > buffer_left:
                min_dist = point_dist(xp, yp, buffer_front, buffer_left)  # the distance to front left corner
            else:
                min_dist = point_dist(xp, yp, buffer_front, buffer_right)  # the distance to front right corner
        else:
            if buffer_right <= yt <= buffer_left:  # intersect at front edge, is collision
                t_min = t_f
                min_dist = 0
                pt_min = buffer_front, yt
            elif yt > buffer_left:  # intersect at front left
                if yp <= yt:
                    min_dist, pt_min = point_to_line(buffer_front, buffer_left, A, B, C)
                    t_min = (pt_min[0] - xp) / vx
                else:  # left edge interaction: y = y_buff = yt = yp + vy * t_l
                    t_l = (buffer_left - yp) / vy
                    xt = xp + vx * t_l
                    if buffer_rear <= xt <= buffer_front:  # intersect at left edge, is collision
                        t_min = t_l
                        min_dist = 0
                        pt_min = xt, buffer_left
                    else:
                        min_dist, pt_min = point_to_line(buffer_rear, buffer_left, A, B, C)
                        t_min = (pt_min[0] - xp) / vx
            else:  # intersect at front right
                if yp >= yt:
                    min_dist, pt_min = point_to_line(buffer_front, buffer_right, A, B, C)
                    t_min = (pt_min[0] - xp) / vx
                else:  # right edge interaction: y = y_buff = yt = yp + vy * t_l
                    t_r = (buffer_right - yp) / vy
                    xt = xp + vx * t_r
                    if buffer_rear <= xt <= buffer_front:  # intersect at right edge, is collision
                        t_min = t_r
                        min_dist = 0
                        pt_min = xt, buffer_right
                    else:
                        min_dist, pt_min = point_to_line(buffer_rear, buffer_right, A, B, C)
                        t_min = (pt_min[0] - xp) / vx
    else:
        if yp >= buffer_left:
            # left edge interaction: y = y_buff = yt = yp + vy * t_l
            t_l = (buffer_left - yp) / vy
            xt = xp + vx * t_l
            if t_l < 0:  # object moving away, start point has minimum distance
                t_min = 0
                pt_min = xp, yp
                if buffer_rear <= xp <= buffer_front:
                    min_dist = yp - buffer_left  # the distance to the left edge
                else:
                    min_dist = point_dist(xp, yp, buffer_rear, buffer_left)  # the distance to rear right corner
            else:
                if buffer_rear <= xt <= buffer_front:  # intersect at left edge, is collision
                    t_min = t_l
                    min_dist = 0
                    pt_min = xt, buffer_left
                elif xt > buffer_front:
                    min_dist, pt_min = point_to_line(buffer_front, buffer_left, A, B, C)
                    t_min = (pt_min[0] - xp) / vx
                else:
                    min_dist, pt_min = point_to_line(buffer_rear, buffer_left, A, B, C)
                    t_min = (pt_min[0] - xp) / vx
        elif yp <= buffer_right:
            # right edge interaction: y = -y_buff = yt = yp + vy * t_l
            t_r = (buffer_right - yp) / vy
            xt = xp + vx * t_r
            if t_r < 0:  # object moving away, start point has minimum distance
                t_min = 0
                pt_min = xp, yp
                if buffer_rear<= xp <= buffer_front:
                    min_dist = -yp - buffer_right  # the distance to the right edge
                else:
                    min_dist = point_dist(xp, yp, buffer_rear, buffer_right)  # the distance to rear right corner
            else:
                if buffer_rear <= xt <= buffer_front:  # intersect at left edge, is collision
                    t_min = t_r
                    min_dist = 0
                    pt_min = xt, buffer_right
                elif xt > buffer_front:
                    min_dist, pt_min = point_to_line(buffer_front, buffer_right, A, B, C)
                    t_min = (pt_min[0] - xp) / vx
                else:
                    min_dist, pt_min = point_to_line(buffer_rear, buffer_right, A, B, C)
                    t_min = (pt_min[0] - xp) / vx
        elif xp >= buffer_rear:
            t_min = 0
            min_dist = -1
            pt_min = xp, yp
        else:  # rear position, should not be seen by the front camera
            t_min = None
            min_dist = None
            pt_min = None

    return t_min, min_dist, pt_min

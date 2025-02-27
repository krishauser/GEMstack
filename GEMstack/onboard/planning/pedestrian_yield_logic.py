from ...state import AgentState, AgentEnum, EntityRelation, EntityRelationEnum, ObjectFrameEnum, VehicleState
from ..component import Component
from typing import List, Dict

import numpy as np


DEBUG = True    # Set to False to disable debug output

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


# def find_min_distance_and_time_by_optimizer(xp, yp, vx, vy, buffer, time_scale):
#     from scipy.optimize import minimize_scalar
#     def pos_t(t):
#         xt, yt = xp + vx * t, yp + vy * t
#         return shortest_distance_to_buffer_in_vehicle_frame((xt, yt), buffer)
#
#     res = minimize_scalar(pos_t, bounds=(0, time_scale))
#     if res.success:
#         t_min = res.x
#         min_dist = res.fun
#         return t_min, min_dist
#     else:
#         return None, None


""" Planning in start frame with waypoints """
# # TODO: to get further unreached waypoints only
# def get_waypoint_arc_and_new_yaw(curr_x, curr_y, curr_yaw, waypoint):
#     """
#     Input:
#         curr_x, curr_y, curr_yaw: vehicle current pose
#         waypoint: next waypoint
#     Output:
#         len_arc, radius, (x_c, y_c): length, radius and the center of the arc-shaped path
#         next_yaw: vehicle yaw at the waypoint.
#     """
#     # TODO: check if it works when the angles cross zero or delta greater than pi
#     wp_x, wp_y = waypoint
#     dist_curr2wp = np.sqrt((wp_x - curr_x) ** 2 + (wp_y - curr_y) ** 2)
#     alpha = np.arctan2(wp_y - curr_y, wp_x - curr_x) - curr_yaw
#     delta = 2 * alpha
#     radius = dist_curr2wp / 2 / np.sin(alpha)
#     len_arc = delta * radius
#     x_c, y_c = curr_x - radius * np.sin(curr_yaw), curr_y + radius * np.cos(curr_yaw)
#     next_yaw = curr_yaw + delta
#     return len_arc, radius, (x_c, y_c), next_yaw
#
#
# def check_collision_with_waypoints(pose, velocity, yaw_rate, radius, center, expand,
#                                    start_point, end_point, time_vehicle_in, time_vehicle_out):
#     """
#     Given an arc as a path and a point with velocity, check if the point is crossing the path.
#     Input:
#         pose, velocity, yaw_rate: pedestrian pose, velocity and angular velocity
#         radius, center, start_point, end_point: parameters of the path as an arc
#         expand: enlarge the arc as the vehicle moving area
#         start_point, end_point: start point and end point of the path
#         time_vehicle_in, time_vehicle_out: the time that the vehicle is in the section of the path
#     Output:
#         is_crossing: boolean, if the pedestrian is crossing the path
#         arc_dist_collision: distance from start_point to the closest pedestrian enter or exit point mapped to the path
#     """
#     # TODO: consider pedestrian path with yaw_rate
#     xp, yp = pose.x, pose.y
#     vx, vy = velocity
#     xc, yc = center
#     x1, y1 = start_point
#     x2, y2 = end_point
#     theta1 = np.arctan2(y1 - yc, x1 - xc)
#     theta2 = np.arctan2(y2 - yc, x2 - xc)
#     r_inner = max(radius - expand, 0)
#     r_outer = radius + expand
#
#     def is_angle_between(pt, start_angle, end_angle):
#         xp, yp = pt
#         angle = np.arctan2(yp - yc, xp - xc)
#         if start_angle < end_angle:
#             return start_angle <= angle <= end_angle
#         else:
#             return end_angle <= angle <= start_angle
#
#     def is_in_ring(pt, xc, yc, r_inner, r_outer):
#         xp, yp = pt
#         dist = np.sqrt((xp - xc) ** 2 + (yp - yc) ** 2)
#         return r_inner <= dist <= r_outer
#
#     def find_arc_intersection(xc, yc, xp, yp, vx, vy, r):
#         # solve equations: (xt - xc)^2 + (yt - yc)^2 = (R+/-b)^2
#         A = vx ** 2 + vy ** 2
#         B = 2 * (vx * (xp - xc) + vy * (yp - yc))
#         C = (xp - xc) ** 2 + (yp - yc) ** 2 - r ** 2
#         root = B ** 2 - 4 * A * C
#         t_list = []         # time
#         pt_list = []        # intersection point
#         if root < 0:
#             return t_list, pt_list  # no intersection
#         else:
#             t1 = (-B - np.sqrt(root)) / (2 * A)
#             t2 = (-B + np.sqrt(root)) / (2 * A)
#             # t should be larger than 0
#             if t1 > 0:
#                 pt1 = (xp + vx * t1, yp + vy * t1)
#                 if is_angle_between(pt1, theta1, theta2):
#                     t_list.append(t1)
#                     pt_list.append(pt1)
#             if t2 > 0:
#                 pt2 = (xp + vx * t2, yp + vy * t2)
#                 if is_angle_between(pt2, theta1, theta2):
#                     t_list.append(t2)
#                     pt_list.append(pt2)
#             return t_list, pt_list
#
#     def find_edge_intersection(xc, yc, xp, yp, vx, vy, theta, r_inner, r_outer):
#         x_inner, y_inner = xc + r_inner * np.cos(theta), yc + r_inner * np.sin(theta)
#         x_outer, y_outer = xc + r_outer * np.cos(theta), yc + r_outer * np.sin(theta)
#         t_list = []         # time
#         pt_list = []        # intersection point
#
#         dx = x_outer - x_inner
#         dy = y_outer - y_inner
#         # solve t from: xt = xp + vx * t, yt = xp + vy * t and dx * (yt - y_inner) = dy * (xt - x_inner)
#         if dx * vy - dy * vx == 0:
#             return t_list, pt_list  # parallel, no intersection
#         else:
#             t = (dy * xp - dx * yp + dx * y_inner - dy * x_inner) / (dx * vy - dy * vx)
#             pt = xp + vx * t, yp + vy * t
#             if is_in_ring(pt, xc, yc, r_inner, r_outer):
#                 t_list.append(t)
#                 pt_list.append(pt)
#             return t_list, pt_list
#
#     # find the time and points a pedestrian in and out of the path
#     t_list = []
#     # Case pedestrian in the path at the beginning
#     if is_in_ring((xp, yp), xc, yc, r_inner, r_outer) and is_angle_between((xp, yp), theta1, theta2):
#         t_list.append(0)
#     # Case pedestrian cross the arcs and the start and end edges of the path section
#     t_inner, _ = find_arc_intersection(xc, yc, xp, yp, vx, vy, r_inner) if r_inner > 0 else [], []
#     t_outer, _ = find_arc_intersection(xc, yc, xp, yp, vx, vy, r_outer)
#     t_theta1, _ = find_edge_intersection(xc, yc, xp, yp, vx, vy, theta1, r_inner, r_outer)
#     t_theta2, _ = find_edge_intersection(xc, yc, xp, yp, vx, vy, theta2, r_inner, r_outer)
#     # Combine all points together, The elements in both lists correspond one by one in order
#     t_list = t_list + t_inner + t_outer + t_theta1 + t_theta2
#
#     def arc_length_from_start_point(pt, center, radius, start_angle):
#         """
#         Calculate the arc length between the point mapping to the arc and the start point as the distance along the path
#         Assume angle difference of the arc is not greater than 180 degrees
#         """
#         x, y = pt
#         xc, yc = center
#         angle = np.arctan2(y - yc, x - xc)
#         delta_angle = abs(angle - start_angle)
#         if delta_angle > np.pi:
#             delta_angle = 2 * np.pi - delta_angle
#         return radius * delta_angle
#
#     is_collision = False
#     arc_dist_collision = None
#     # collision if there is intersection and the time is between time_vehicle_in and time_vehicle_out
#     if min(t_list) <= time_vehicle_out or max(t_list) >= time_vehicle_in:
#         is_collision = True
#         t_min = max(time_vehicle_in, min(t_list))
#         t_max = min(time_vehicle_out, max(t_list))
#         # map the point to the arc of path for calculating distance from the vehicle follow the path
#         pt_min = xp + vx * t_min, yp + vy * t_min
#         pt_max = xp + vx * t_max, yp + vy * t_max
#         arc_time_min = arc_length_from_start_point(pt_min, center, radius, theta1)
#         arc_time_max = arc_length_from_start_point(pt_max, center, radius, theta2)
#         arc_dist_collision = min(arc_time_min, arc_time_max)
#
#     return is_collision, arc_dist_collision
#
#
# def yield_in_start_frame(path_further, state, agents: Dict[str,AgentState]):
#     """
#     All calculations are in start frame, origin reference: rear_axle_center (refer to GEMstack/knowledge/calibration)
#     """
#     # current states
#     vehicle = state.vehicle
#     curr_x = vehicle.pose.x
#     curr_y = vehicle.pose.y
#     curr_yaw = vehicle.pose.yaw
#     curr_v = vehicle.v
#
#     # determine lookahead distance using current velocity and a decent deceleration
#     t_brake = curr_v / comfort_decel  # time to brake down to zero
#     dist_lookahead = curr_v * t_brake
#
#     distance = 0
#     temp_x, temp_y, temp_yaw = curr_x, curr_y, curr_yaw
#     is_collision = False
#     dist_collision = None
#     for waypoint in path_further:
#         # the path to next waypoint and vehicle yaw at next waypoint
#         len_arc, radius, center, next_yaw = get_waypoint_arc_and_new_yaw(temp_x, temp_y, temp_yaw, waypoint)
#
#         # the time of vehicle go in and out of the section to next waypoint in current velocity
#         time_vehicle_in = (distance - l_rear_axle_to_front) / curr_v    # consider distance from the center of rear axle to the front
#         time_vehicle_out = (distance + len_arc + l_rear_axle_to_rear) / curr_v    # consider distance from the center of rear axle to the rear
#
#         # check all the pedestrian, get their time and position if they are going to cross the path
#         arc_dist_collision_list = []
#         for n, ped in agents.items():
#             if ped.type == AgentEnum.PEDESTRIAN:
#                 # check collision: pedestrian is in the section of the path between time_vehicle_in and time_vehicle_out
#                 is_collision, arc_dist_collision = check_collision_with_waypoints(ped.pose, ped.velocity,ped.yaw_rate,
#                                                                                   radius, center, buffer_y,
#                                                                                   (temp_x, temp_y), waypoint,
#                                                                                   time_vehicle_in, time_vehicle_out)
#                 if is_collision:
#                     arc_dist_collision_list.append(arc_dist_collision)
#
#         if is_collision:
#             # use the minimum collision distance to yield
#             dist_collision = distance + min(arc_dist_collision_list)
#             break
#         # update total distance by add the arc length of the section to the next waypoint, update pose at next waypoint
#         distance += len_arc
#         temp_x, temp_y = waypoint
#         temp_yaw = next_yaw
#         # end the loop if total distance is larger than lookahead distance
#         if distance > dist_lookahead:
#             break
#
#     return is_collision, dist_collision

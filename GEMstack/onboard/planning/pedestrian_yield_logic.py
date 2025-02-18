from ...state import AgentState, AgentEnum, EntityRelation, EntityRelationEnum, ObjectFrameEnum
from ..component import Component
from typing import List, Dict

import numpy as np
from scipy.optimize import minimize_scalar

##### Hardcoded for testing #####
from ...state import ObjectPose
ped1_start_pose_abs = ObjectPose(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN, t=0, x=15, y=2)
ped2_start_pose_abs = ObjectPose(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN, t=0, x=15, y=2)  # Not sure the transform ~! The exact pos is (30,2)
#################################

DEBUG = True  # Set to False to disable debug output
# DEBUG = False

class PedestrianYielder(Component):
    """Yields for all pedestrians in the scene.

    Result is stored in the relations graph.
    """

    def rate(self):
        return None

    def state_inputs(self):
        return ['agents']

    def state_outputs(self):
        return ['relations']

    def update(self, agents: Dict[str, AgentState]) -> List[EntityRelation]:
        res = []
        for n, a in agents.items():
            if DEBUG:
                print(
                    f"[DEBUG] PedestrianYielder.update: Agent:{n} frame:{a.pose.frame}, (x,y):{a.pose.x, a.pose.y}, velocity:{a.velocity}")

            """ collision estimation based on agent states in vehicle frame """
            ##### Hardcoded for testing #####
            if n == 'ped1':
                a = a.to_frame(ObjectFrameEnum.START, current_pose=a.pose, start_pose_abs=ped1_start_pose_abs)
            elif n == 'ped2':
                a = a.to_frame(ObjectFrameEnum.START, current_pose=a.pose, start_pose_abs=ped2_start_pose_abs)
            #################################
            print(f"[SIM] PedestrianYielder.update: Agent:{n} frame:{a.pose.frame}, (x,y):{a.pose.x, a.pose.y}, velocity:{a.velocity}")
            # TODO: check how to convert to vehicle frame. May perception update agent state?
            if a.type == AgentEnum.PEDESTRIAN:
                check, t_min, min_dist = check_collision_in_vehicle_frame(a)
                if DEBUG:
                    print(f"[DEBUG] is_collision:{check}, t_min:{t_min}, min_dist:{min_dist}, distance to buffer area:", shortest_distance_to_buffer_in_vehicle_frame((a.pose.x, a.pose.y), (2.53 / 2 + 3, 1.35 / 2 + 1)))
                # if check == 'STOP':
                #     # relation: ego-vehicle yields to pedestrian
                #     res.append(EntityRelation(type=EntityRelationEnum.STOPPING_AT, obj1='', obj2=n))
                # elif check == 'YIELD':
                #     res.append(EntityRelation(type=EntityRelationEnum.YIELDING, obj1='', obj2=n))
                # else:
                #     continue
                if check:
                    res.append(EntityRelation(type=EntityRelationEnum.YIELDING, obj1='', obj2=n))
        return res


# """ Vehicle Dimensions """
# vehicle_model = "e2"  # e2 or e4
# # vehicle dimensions and buffer area
# if vehicle_model == 'e2':
#     # e2 axle dimensions, refer to GEMstack/knowledge/vehicle/model/gem_e2.urdf
#     size_x, size_y = 2.53, 1.35         # measured from base_link.STL
#     lx_f2r = 0.88 + 0.87                # distance between front axle and rear axle, from gem_e2.urdf wheel_links
#     ly_axle = 0.6 * 2                   # length of axle
#     l_rear_axle_to_front = 1.28 + 0.87  # measured from base_link.STL
#     l_rear_axle_to_rear = size_y - l_rear_axle_to_front
# elif vehicle_model == 'e4':
#     # e4 axle dimensions, refer to GEMstack/knowledge/vehicle/model/gem_e4.urdf
#     size_x, size_y = 3.35, 1.35         # measured from base_link.STL
#     lx_f2r = 1.2746 + 1.2904            # distance between front axle and rear axle, from gem_e4.urdf wheel_links
#     ly_axle = 0.6545 * 2                # length of front and rear axles, from gem_e4.urdf wheel_links
#     l_rear_axle_to_front = 1.68 + 1.29  # measured from base_link.STL
#     l_rear_axle_to_rear = size_y - l_rear_axle_to_front
# # add buffer to outer dimensions, defined by half of the x, y buffer area dimensions
# buffer_x, buffer_y = 3, 1
# buffer = size_x / 2 + buffer_x, size_y / 2 + buffer_y


""" Planning in vehicle frame without waypoints """
def check_collision_in_vehicle_frame(agent_state: AgentState):
    buffer = 2.53 / 2 + 3, 1.35 / 2 + 1  # e2 buffer area
    time_scale = 100  # TODO: adjust if necessary
    xp, yp = agent_state.pose.x, agent_state.pose.y
    vx, vy = agent_state.velocity[:2]
    
    # print(f"[TEST] check_collision_in_vehicle_frame: xp:{xp}, yp:{yp}, vx:{vx}, vy:{vy}")

    # use optimization method to find minimum distance and the time at that point
    t_min, min_dist = find_min_distance_and_time(xp, yp, vx, vy, buffer, time_scale)
    # if the minimum distance between the position and the buffer area is less than 0, than a collision is expected
    check = False
    if min_dist <= 3:   #TODO: adjust time threshold if necessary
        check = True
    # elif min_dist > 0 and t_min <= 5:   #TODO: adjust time threshold if necessary
    #     check = 'YIELD'
    # else:
    #     check = None
    return check, t_min, min_dist


def find_min_distance_and_time(xp, yp, vx, vy, buffer, time_scale):
    def pos_t(t):
        xt, yt = xp + vx * t, yp + vy * t
        # print(f"[TEST] post_t: xt:{xt}, yt:{yt}, t:{t}")
        return shortest_distance_to_buffer_in_vehicle_frame((xt, yt), buffer)

    res = minimize_scalar(pos_t, bounds=(0, time_scale))
    # print(f"[TEST] find_min_distance_and_time: res:{res}")
    if res.success:
        t_min = res.x
        min_dist = res.fun
        return t_min, min_dist
    else:
        return None, None


# Calculate the shortest distance from an object to vehicle buffer area in vehicle frame
def shortest_distance_to_buffer_in_vehicle_frame(position, buffer):
    """
    Calculate the distance between a pedestrian's position and the vehicle with buffer
    """
    x_buff, y_buff = buffer
    # consider 2D geometry
    x_p, y_p = position
    # initiate distance
    dist = 0

    # calculate distance
    # front
    if -y_buff <= y_p <= y_buff and x_p > x_buff:
        dist = x_p - x_buff
    # left front
    elif y_p > y_buff and x_p > x_buff:
        dist = np.sqrt((x_p - x_buff) ** 2 + (y_p - y_buff) ** 2)
    # right front
    elif y_p < -y_buff and x_p > x_buff:
        dist = np.sqrt((x_p - x_buff) ** 2 + (y_p + y_buff) ** 2)
    # left
    elif y_p > y_buff and -x_buff <= x_p <= x_buff:
        dist = y_p - y_buff
    # right
    elif y_p < -y_buff and -x_buff <= x_p <= x_buff:
        dist = abs(y_p) + abs(y_buff)                       ### Modified
    # rear
    # elif -y_buff <= y_p <= y_buff and x_p < -x_buff:
    #     dist = abs(x_p) + abs(x_buff)                       ### Modified
    # # left rear
    # elif y_p > y_buff and x_p < -x_buff:
    #     dist = np.sqrt((x_p + x_buff) ** 2 + (y_p - y_buff) ** 2)
    # # right rear
    # elif y_p < -y_buff and x_p < -x_buff:
    #     dist = np.sqrt((x_p + x_buff) ** 2 + (y_p + y_buff) ** 2)
    # intersect
    else:
        dist = 100                                          ### Modified
        # Note : Sometimes the minimize_scalar() is not returning correct value causing the car to keep stopping.
        #        So, Added a large value to make sure the car doesn't stop.

    # print(f"[TEST] Shortest Dist: position:{position}, buffer:{buffer}, distance:{dist}")
    return dist


""" Planning in start frame with waypoints """
# # TODO: how to get further unreached waypoints only
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
#     # TODO: compute pedestrian path with yaw_rate?
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
#     # TODO: confirm origin reference with calibration team
#     # current states
#     vehicle = state.vehicle
#     curr_x = vehicle.pose.x
#     curr_y = vehicle.pose.y
#     curr_yaw = vehicle.pose.yaw
#     curr_v = vehicle.v
#
#     # determine lookahead distance using current velocity and a decent deceleration
#     decel_decent = 2.0  # TODO: adjust for a decent deceleration if necessary
#     t_brake = curr_v / decel_decent  # time to brake down to zero
#     dist_lookahead = curr_v * t_brake
#
#     # TODO: check if it works with straight line waypoints
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


""" Assume the vehicle move straight forward """
# def compute_intersect(A1, B1, C1, A2, B2, C2):
#     """
#     calculate intersection point of two lines
#     A1*x + B1*y + C1 = 0 and A2*x + B2*y + C2 = 0
#     """
#     if A1*B2 - A2*B1 == 0:
#         if C1 == C2:
#             return "conincide"
#         else:
#             return "parallel"
#     else:
#         x_inter = (B1*C2 - B2*C1) / (A1*B2 - A2*B1)
#         y_inter = (C1*A2 - C2*A1) / (A1*B2 - A2*B1)
#         return x_inter, y_inter
#
# def yield_in_ego_frame(pedestrian_position_ego, pedestrian_velocity_ego, dist_lookahead, buffer):
#     """
#     Calculate the intersection of the pedestrian's path and vehicle path with buffer
#     Yield if the intersection is in front of the vehicle and within lookahead distance
#     """
#     x_buff, y_buff = buffer
#     # consider 2D geometry
#     x_p, y_p = pedestrian_position_ego[:2]
#     v_x, v_y = pedestrian_velocity_ego[:2]
#     # initiate yielding and distance
#     yielding = False
#     distance = None
#
#     # line expression: A*x + B*y + C = 0
#     # pedestrian's path: v_y * x - v_x * y + (v_x * y_p - v_y * x_p) = 0
#     A_p, B_p, C_p = v_y, -v_x, v_x * y_p - v_y * x_p
#     # buffer area: y = y_buff, y = - y_buff
#     A_left, B_left, C_left = 0, 1, -y_buff
#     A_right, B_right, C_right = 0, 1, y_buff
#
#     # deal with parallel first
#     if A_p*B_left - A_left*B_p == 0:
#         if -y_buff <= y_p <= y_buff:
#             yielding = True
#             distance = x_p
#         else:
#             yielding = False
#             distance = None
#     # compute intersection
#     else:
#         left_inter_x, left_inter_y = compute_intersect(A_p, B_p, C_p, A_left, B_left, C_left)
#         right_inter_x, right_inter_y = compute_intersect(A_p, B_p, C_p, A_right, B_right, C_right)
#         if -x_buff <= left_inter_x <= dist_lookahead or -x_buff <= right_inter_x <= dist_lookahead:
#             yielding = True
#         else:
#             yielding = False
#         distance = min(left_inter_x, right_inter_x)
#
#     return yielding, distance
#
# vehicle = state.vehicle
# curr_x = vehicle.pose.x
# curr_y = vehicle.pose.y
# curr_yaw = vehicle.pose.yaw
# curr_v = vehicle.v
#
# # 2D transformation from start frame to ego frame
# R_start2ego = np.array([[np.cos(curr_yaw),-np.sin(curr_yaw)],[np.sin(curr_yaw),np.cos(curr_yaw)]])
# t_start2ego = np.array([curr_x, curr_y])
#
# # check collision distance and break if going to collide
# dist_min = np.inf
# for ped in pedestrians:
#     # pedestrian states from perception
#     pos_p =  ped.state.pose    # both vectors are x, y in vehicle frame
#     v_p = ped.state.v
#     pos_start2ego = R_start2ego @ pos_p + t_start2ego
#     v_start2ego = v_p @ pos_p + t_start2ego
#     yielding, distance = yield_in_ego_frame(pos_start2ego,v_start2ego, dist_lookahead, buffer)
#     if yielding == True and distance is not None:
#         if distance < dist_min:
#             dist_min = distance

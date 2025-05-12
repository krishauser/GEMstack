from typing import List
import numpy as np
from typing import List
import reeds_shepp
from shapely.geometry import Polygon
from typing import List, Tuple
import math
import yaml


Pose   = Tuple[float, float, float]         # (x, y, yaw)
Dims   = Tuple[float, float]                # (width, length)
Obstacle = Tuple[float, float, float, Dims] # (x, y, yaw, (width, length))

class ReedsSheppParking:
    def __init__(self):
        
        self.detected_cones = []
        self.parked_cars = []
        self.objects_to_avoid_collisions = []
        
        yaml_path = "GEMstack/knowledge/defaults/ReedsShepp_param.yaml"
        with open(yaml_path,'r') as file:
            params = yaml.safe_load(file)

        self.static_horizontal_curb_xy_coordinates = None
        self.static_horizontal_curb_size  = params['reeds_shepp_parking']['static_horizontal_curb_size']
        self.add_static_vertical_curb_as_obstacle = params['reeds_shepp_parking']['add_static_vertical_curb_as_obstacle']

        self.static_vertical_curb_size = params['reeds_shepp_parking']['static_vertical_curb_size']
        self.static_vertical_curb_xy_coordinates = []
        self.add_static_horizontal_curb_as_obstacle = params['reeds_shepp_parking']['add_static_horizontal_curb_as_obstacle']

        self.all_parking_spots_in_parking_lot_var  = []

        self.vehicle_pose = [0,0,0] # default
        self.x_axis_of_search = self.vehicle_pose[0]
        

        self.vehicle_dims = params['vehicle']['vehicle_dim']
        self.vehicle_turning_radius = params['vehicle']['vehicle_turning_radius']
        self.compact_parking_spot_size = params['reeds_shepp_parking']['compact_parking_spot_size']
        self.shift_from_center_to_rear_axis = params['reeds_shepp_parking']['shift_from_center_to_rear_axis'] # TODO: Check
        self.search_step_size = params['reeds_shepp_parking']['search_step_size'] 
        self.parking_lot_axis_shift_margin = params['reeds_shepp_parking']['parking_lot_axis_shift_margin'] 
        self.search_bound_threshold = params['reeds_shepp_parking']['search_bound_threshold'] 
        # TODO: Add thrid option: park in the middle
        self.closest = params['reeds_shepp_parking']['closest']  # If True, the closest parking spot will be selected, otherwise the farthest one will be selected
        self.clearance_step = params['reeds_shepp_parking']['clearance_step'] 
        self.clearance = params['reeds_shepp_parking']['clearance'] 
        self.search_axis_direction_var = False

    

    def reeds_shepp_path(self,start_pose, final_pose, step_size, vehicle_turning_radius):# Runing 
        path = reeds_shepp.path_sample(start_pose, final_pose, vehicle_turning_radius, step_size)
        waypoints = [(x, y, yaw) for x, y, yaw, *_ in path]
        #waypoints = np.array(waypoints_for_obstacles_check)[:,:2]
        return waypoints
    
    def rectangle_polygon(self, center: Pose, dims: Dims) -> Polygon:
        """
        Build a shapely Polygon for an oriented rectangle.
        
        Args:
            center: (x, y, yaw) of rectangle center in world frame.
            dims:   (width, length) of rectangle.
            
        Returns:
            shapely Polygon of the 4 corners, in CCW order.
        """
        x, y, yaw = center
        w, L      = dims
        # corners in vehicle frame (forward is +x, left is +y)
        corners = np.array([
            [ +L/2, +w/2],
            [ +L/2, -w/2],
            [ -L/2, -w/2],
            [ -L/2, +w/2],
        ])
        # rotation matrix
        R = np.array([[np.cos(yaw), -np.sin(yaw)],
                    [np.sin(yaw),  np.cos(yaw)]])
        # rotate & translate
        world_corners = (R @ corners.T).T + np.array([x, y])
        return Polygon(world_corners)    
    
    def is_trajectory_collision_free(self,
        trajectory: List[Pose],
        vehicle_dims: Dims,
        obstacles: List[Obstacle]
    ) -> bool:
        """
        Check whether following `trajectory` will avoid all `obstacles`.
        
        Args:
            trajectory: List of (x, y, yaw) vehicle poses along planned path.
            vehicle_dims: (width, length) of your vehicle rectangle.
            obstacles: List of static obstacles, each as (x, y, yaw, (width, length)).
            
        Returns:
            True if no pose along the trajectory intersects any obstacle; False otherwise.
        """
        # Pre-build obstacle polygons once
        obstacle_polys = [
            self.rectangle_polygon((ox, oy, oyaw), dims)
            for ox, oy, oyaw, dims in obstacles
        ]
        
        # For each pose along the trajectory, test intersection
        for pose in trajectory:
            veh_poly = self.rectangle_polygon(pose, vehicle_dims)
            for obs_poly in obstacle_polys:
                if veh_poly.intersects(obs_poly):
                    # collision detected
                    return False
        # no collisions anywhere
        return True


    def find_parking_positions(self,
        obstacles: List[Tuple[float, float, float, Tuple[float, float]]],
        vehicle_dims: Tuple[float, float],
        margin: float = 0.2
        ) -> List[Tuple[float, float, float]]:
        """
        Compute all the parking‐slot center positions and orientations along a single lane
        where a vehicle of given dimensions can fit between existing obstacles.

        Parameters
        ----------
        obstacles : List of (x, y, theta, (w, l))
            Each obstacle is centered at (x,y), oriented by yaw theta (radians),
            and has width w (across lane) and length l (along lane).
        vehicle_dims : (w_v, l_v)
            Width and length of the vehicle (same convention as obstacles).
        margin : float
            Minimum clearance to leave between parked vehicles (in meters).

        Returns
        -------
        slots : List of (x_c, y_c, theta_c)
            Center‐points and yaw for each available parking slot.
        """
        if not obstacles:
            return []

        # Assume all obstacles lie along one straight lane with the same orientation
        lane_theta = obstacles[0][2]
        cos_t = math.cos(lane_theta)
        sin_t = math.sin(lane_theta)

        # Project each obstacle center onto the lane axis (s-coordinate)
        projected = []
        for x, y, theta, (w, l) in obstacles:
            # s = x*cos + y*sin
            s = x * cos_t + y * sin_t
            half_len = l / 2.0
            projected.append((s, half_len, (x, y)))

        # Sort by increasing s
        projected.sort(key=lambda e: e[0])

        slots: List[Tuple[float, float, float]] = []
        w_v, l_v = vehicle_dims

        # Examine gaps between consecutive obstacles
        for (s_i, half_i, (x_i, y_i)), (s_j, half_j, (x_j, y_j)) in zip(projected, projected[1:]):
            # compute start/end of free interval in s‐space
            free_start = s_i + half_i + margin/2
            free_end   = s_j - half_j - margin/2
            free_length = free_end - free_start

            # How many cars of length l_v can we fit (with margin between them)?
            if free_length >= l_v:
                # spacing = car length + margin
                spacing = l_v + margin
                n_fit = int(math.floor((free_length + margin) / spacing))
                for k in range(n_fit):
                    # center s‐coordinate for each slot
                    s_c = free_start + (spacing * k) + (l_v / 2)
                    # map back to (x,y): x = s*cos, y = s*sin (plus any perpendicular offset if needed)
                    x_c = s_c * cos_t
                    y_c = s_c * sin_t
                    slots.append((x_c, y_c, lane_theta))

        return slots

    def all_parking_spots_in_parking_lot(self,
        static_horizontal_curb: List[Tuple[float, float, float, Dims]],
        compact_parking_spot_size: Dims,
        yaw_of_parked_cars_var: float = 0.0,
    ) -> List[Pose]:
        """
        Computes uniformly spaced parking spot centers along a curb line.

        Args:
            static_horizontal_curb: List of 2 curb endpoints, each as (x, y, yaw, dims).
            compact_parking_spot_size: (width, length) of parking spot.
            yaw_of_parked_cars_var: orientation of parked cars (rad).
            margin: optional spacing between adjacent spots.

        Returns:
            List of (x, y, yaw) poses for each parking spot.
        """
        if len(static_horizontal_curb) != 2:
            raise ValueError("Exactly two curb endpoints are required.")

        # Extract center points of the curb rectangles
        (x0, y0, _, dims0), (x1, y1, _, dims1) = static_horizontal_curb
        L0 = dims0[1]
        L1 = dims1[1]

        # Start and end points shifted to actual line segment ends
        p0 = (x0 + math.cos(yaw_of_parked_cars_var) * L0 / 2,
            y0 + math.sin(yaw_of_parked_cars_var) * L0 / 2)
        p1 = (x1 - math.cos(yaw_of_parked_cars_var) * L1 / 2,
            y1 - math.sin(yaw_of_parked_cars_var) * L1 / 2)

        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        total_length = math.hypot(dx, dy)

        spot_length = compact_parking_spot_size[1]
        spacing = spot_length 

        if total_length < spot_length:
            raise ValueError("Insufficient curb length to place even one spot.")

        # Number of full spots that can fit along the curb
        num_spots = int(math.floor(total_length / spacing))
        dir_x = dx / total_length
        dir_y = dy / total_length

        parking_spots: List[Pose] = []
        for i in range(num_spots):
            dist = (i + 0.5) * spacing  # center of spot
            x = p0[0] + dir_x * dist
            y = p0[1] + dir_y * dist
            parking_spots.append((x, y, yaw_of_parked_cars_var))

        return parking_spots

    # def pick_parking_spot(available_parking_spots_var, x, y, yaw = 0.0, closest = True):

    #     def distance(spot):
    #         dx = spot[0] - x
    #         dy = spot[1] - y
    #         return math.hypot(dx, dy)
        
    #     if closest:
    #         return min(available_parking_spots_var, key=distance)
    #     else:
    #         return max(available_parking_spots_var, key=distance)
    # 

    def pick_parking_spot(self,
        available_spots: List[Tuple[float, float, float]],
        all_spots: List[Tuple[float, float, float]],
        vehicle_pose: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """
        Maneuver-complexity-aware spot selection strategy
        
        Pick a parking spot based on how easily it can be entered/exited:
        Priority 1: sandwiched between two available spots
        Priority 2: adjacent to one available spot
        Priority 3: isolated

        Break ties using distance to current vehicle pose.
        """

        # Helper: distance to current pose
        def dist_to_vehicle(self, spot):
            dx = spot[0] - vehicle_pose[0]
            dy = spot[1] - vehicle_pose[1]
            return math.hypot(dx, dy)

        # Build availability map (assume order of spots is spatially sorted)
        spot_indices = {spot: i for i, spot in enumerate(all_spots)}
        available_set = set(available_spots)

        ranked_spots = []

        for spot in available_spots:
            idx = spot_indices[spot]

            # Check neighbors
            left_free  = idx - 1 >= 0 and all_spots[idx - 1] in available_set
            right_free = idx + 1 < len(all_spots) and all_spots[idx + 1] in available_set

            if left_free and right_free:
                priority = 1
            elif left_free or right_free:
                priority = 2
            else:
                priority = 3

            ranked_spots.append((priority, dist_to_vehicle(spot), spot))

        # Sort: lower priority (1 best) and then by proximity
        ranked_spots.sort()

        # Return the best one
        return ranked_spots[0][2], ranked_spots[0][0]

    def search_axis_direction(self, parking_spot_to_go, vehicle_pose):
        dx = parking_spot_to_go[0][0] - vehicle_pose[0]
        if dx > 0:
            return True
        else:
            return False     

    # def available_parking_spots(all_parking_spots_in_parking_lot, parked_cars, compact_parking_spot_size, yaw_of_parked_cars_var=0.0):
    #     available_parking_spots = []
    #     for spot in all_parking_spots_in_parking_lot:
    #         # Check if the parking spot is occupied by a parked car
    #         is_occupied = False
    #         for parked_car in parked_cars:
    #             if (abs(parked_car[0] - spot[0]) < compact_parking_spot_size[1] / 2) and (abs(parked_car[1] - spot[1]) < compact_parking_spot_size[0] / 2):
    #                 is_occupied = True
    #                 break

    #         # If the parking spot is not occupied, add it to the available list
    #         if not is_occupied:
    #             available_parking_spots.append(spot)

    #     return available_parking_spots  

    def available_parking_spots(self,
        all_parking_spots: List[Pose],
        parked_cars: List[Obstacle],
        spot_dims: Dims
    ) -> List[Pose]:
        """
        Returns unoccupied parking spots, using full geometric rectangle checks.

        Args:
            all_parking_spots: List of (x, y, yaw) tuples.
            parked_cars: List of (x, y, yaw, dims) tuples.
            spot_dims: (width, length) of parking spots.
            clearance: Extra buffer (in meters) to apply around cars when checking overlap.

        Returns:
            List of available parking spot poses.
        """
        clearance = self.clearance
        
        spot_polygons = [
            self.rectangle_polygon(spot, spot_dims)
            for spot in all_parking_spots
        ]

        car_polygons = [
            self.rectangle_polygon((x, y, yaw), dims)
            for x, y, yaw, dims in parked_cars
        ]

        available = []
        for spot, poly in zip(all_parking_spots, spot_polygons):
            # Check for collision with any parked car
            is_occupied = any(poly.intersects(car_poly.buffer(clearance)) for car_poly in car_polygons)
            if not is_occupied:
                available.append(spot)

        return available
    
    def yaw_of_parked_cars(self, curb_0, curb_1):
        # Compute the vector v from p1 to p2
        # v_x = x2 - x1, v_y = y2 - y1
        v = (curb_1[0] - curb_0[0], curb_1[1] - curb_0[1])
        angle_rad = math.atan2(v[1], v[0]) # TODO: Double check if CCW is the positive direction
        return angle_rad

    def shift_points_perpendicular_ccw(self, p1, p2, shift_amount):
        """
        Shift points p1 and p2 by a given amount perpendicular (to the left)
        of the vector from p1 to p2.

        Args:
            p1 (tuple of float): First point (x1, y1)
            p2 (tuple of float): Second point (x2, y2)
            shift_amount (float): Distance to shift perpendicular to the left of vector v

        Returns:
            p1_shifted (tuple of float): Shifted first point
            p2_shifted (tuple of float): Shifted second point
            dir_unit (tuple of float): Normalized direction vector v̂ = (v_x, v_y) / |v|
            shift_vec (tuple of float): Actual shift vector applied = perp_unit * shift_amount
        """
        x1, y1 = p1
        x2, y2 = p2

        # 1) Compute connecting vector v = p2 - p1
        v_x = x2 - x1
        v_y = y2 - y1

        # 2) Compute its magnitude |v|
        length = math.hypot(v_x, v_y)
        if length == 0:
            raise ValueError("p1 and p2 must be distinct points to define a direction.")
        
        # 3) Normalize v to get unit direction v̂ = (v_x, v_y) / |v|
        dir_x = v_x / length
        dir_y = v_y / length

        v_norm = (dir_x, dir_y)
        
        # 4) Compute the left-perpendicular unit vector: perp = (-dir_y, dir_x)
        perp_x = -dir_y
        perp_y =  dir_x

        # 5) Scale this perpendicular by the desired shift_amount
        shift_x = perp_x * shift_amount
        shift_y = perp_y * shift_amount

        # 6) Apply shift to both points
        p1_shifted = (x1 + shift_x, y1 + shift_y)
        p2_shifted = (x2 + shift_x, y2 + shift_y)

        # Return shifted points
        return p1_shifted, p2_shifted, v_norm 
    


    def project_point_on_axis(self, p1, p2, p3):
        """
        Project point p3 orthogonally onto the line (axis) defined by p1 -> p2.

        Args:
            p1 (tuple of float): First point on the axis (x1, y1)
            p2 (tuple of float): Second point on the axis (x2, y2)
            p3 (tuple of float): The point to be projected (x3, y3)

        Returns:
            p_proj (tuple of float): Coordinates of the projection of p3 onto the line p1–p2
            t (float): The parameter along the line (0 at p1, 1 at p2, can be outside [0,1])
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        # Compute vector along the axis v = p2 - p1
        v_x = x2 - x1
        v_y = y2 - y1

        # Compute vector from p1 to p3: u = p3 - p1
        u_x = x3 - x1
        u_y = y3 - y1

        # Compute dot products
        dot_uv = u_x * v_x + u_y * v_y      # u · v
        dot_vv = v_x * v_x + v_y * v_y      # v · v

        if dot_vv == 0:
            raise ValueError("p1 and p2 must be distinct to define an axis.")

        # Parameter t gives the position along the line: p_proj = p1 + t * v
        t = dot_uv / dot_vv

        # Compute projected point coordinates
        proj_x = x1 + t * v_x
        proj_y = y1 + t * v_y

        return (proj_x, proj_y)   
    


    def move_point_along_vector(self, p0, direction, step, positive_direction=True):
        """
        Move the point p0 by a fixed step along the given direction vector.

        Args:
            p0 (tuple of float): The starting point (x0, y0).
            direction (tuple of float): The direction vector (dx, dy).
            step (float): Distance to move along the direction (default 0.1).

        Returns:
            tuple of float: The new point (x_new, y_new) after moving.
        Raises:
            ValueError: if the direction vector has zero length.
        """
        x0, y0 = p0
        dx, dy = direction

        # Compute the length of the direction vector
        length = math.hypot(dx, dy)
        if length == 0:
            raise ValueError("Direction vector must be non-zero to define a movement direction.")

        # Normalize the direction vector to unit length
        ux = dx / length
        uy = dy / length

        # Move the point by 'step' along the unit direction
        if positive_direction:
            x_new = x0 + ux * step
            y_new = y0 + uy * step
        else:
            x_new = x0 - ux * step
            y_new = y0 - uy * step    

        return (x_new, y_new)


    def stitch_paths(self, pose_list: List[Pose], step_size: float, turning_radius: float) -> List[Pose]:
        """
        Given a list of poses, compute and stitch Reeds-Shepp paths between consecutive poses.

        Args:
            pose_list (List[Pose]): List of poses [(x0, y0, yaw0), (x1, y1, yaw1), ...]
            step_size (float): Step size for sampling Reeds-Shepp path
            turning_radius (float): Vehicle turning radius

        Returns:
            List[Pose]: Concatenated list of all waypoints across all segments.
        """
        stitched_path = []
        for i in range(len(pose_list) - 1):
            segment = self.reeds_shepp_path(
                pose_list[i],
                pose_list[i+1],
                step_size=step_size,
                vehicle_turning_radius=turning_radius
            )
            # Avoid duplicating overlapping pose except for the first segment
            if i > 0:
                segment = segment[1:]
            stitched_path.extend(segment)
        return stitched_path


    def find_available_parking_spots_and_search_vector(self, detected_cones=[], vehicle_pose=(0.0, 0.0, 0.0)):
        # Update detected cones and vehicle pose
        self.detected_cones = detected_cones
        self.vehicle_pose = vehicle_pose



        # Compute yaw of parked cars based on static horizontal curb direction
        self.yaw_of_parked_cars_var = self.yaw_of_parked_cars(self.static_horizontal_curb_xy_coordinates[0], self.static_horizontal_curb_xy_coordinates[1])

        # Construct curb obstacle representations with yaw
        self.static_horizontal_curb = [
            (
                self.static_horizontal_curb_xy_coordinates[0][0],
                self.static_horizontal_curb_xy_coordinates[0][1],
                self.yaw_of_parked_cars_var,
                self.static_horizontal_curb_size
            ),
            (
                self.static_horizontal_curb_xy_coordinates[1][0],
                self.static_horizontal_curb_xy_coordinates[1][1],
                self.yaw_of_parked_cars_var,
                self.static_horizontal_curb_size
            )
        ]
        if self.add_static_vertical_curb_as_obstacle:
            self.static_vertical_curb = [
                (
                    self.static_vertical_curb_xy_coordinates[0][0],
                    self.static_vertical_curb_xy_coordinates[0][1],
                    self.yaw_of_parked_cars_var,
                    self.static_vertical_curb_size
                )
            ]

        # Convert cones to parked cars if any are detected
        if self.detected_cones:
           self.parked_cars = [(x, y, self.yaw_of_parked_cars_var, self.vehicle_dims) for x, y in self.detected_cones]
        else:
            self.parked_cars = self.detected_cones

        # Add all obstacles to the collision-checking list
        self.objects_to_avoid_collisions += self.parked_cars
        if self.add_static_horizontal_curb_as_obstacle:
            self.objects_to_avoid_collisions += self.static_horizontal_curb
        if self.add_static_vertical_curb_as_obstacle:
            self.objects_to_avoid_collisions += self.static_vertical_curb

        # Compute full set of parking spots along the curb if not already available
        if not self.all_parking_spots_in_parking_lot_var:
            self.all_parking_spots_in_parking_lot_var = self.all_parking_spots_in_parking_lot(
                self.static_horizontal_curb,
                self.compact_parking_spot_size,
                yaw_of_parked_cars_var=self.yaw_of_parked_cars_var
            )

        # Filter out occupied spots
        self.available_parking_spots_var = self.available_parking_spots(
            self.all_parking_spots_in_parking_lot_var,
            self.parked_cars,
            self.compact_parking_spot_size,
        )
        print("self.available_parking_spots_var", self.available_parking_spots_var)
        if not self.available_parking_spots_var:
            return
            #raise ValueError("No parking spot available.")

        # Select the best available parking spot
        self.parking_spot_to_go, self.priority = self.pick_parking_spot(
                self.available_parking_spots_var,
                self.all_parking_spots_in_parking_lot_var,
                self.vehicle_pose
            )
        
        self.parking_spot_to_go = [self.parking_spot_to_go]
        # Adjust target position for rear-axle-centered model
        x_shift = self.parking_spot_to_go[0][0] - self.shift_from_center_to_rear_axis
        self.parking_spot_to_go[0] = (
            x_shift,
            self.parking_spot_to_go[0][1],
            self.parking_spot_to_go[0][2]
        )

        # Determine direction of search axis (forward or backward)
        self.x_axis_of_search_direction_positive = self.search_axis_direction(
            self.parking_spot_to_go,
            self.vehicle_pose
        )

        # Build the shifted search axis and compute bounds
        # TODO: Find search direction using yaw and vehicle pose
        self.curb_0_xy_shifted, self.curb_1_xy_shifted, self.search_axis_direction_var = self.shift_points_perpendicular_ccw(
            self.static_horizontal_curb_xy_coordinates[0],
            self.static_horizontal_curb_xy_coordinates[1],
            self.parking_lot_axis_shift_margin
        )

        # Horizontal axis search direction
        _ , _, self.horizontal_search_axis_direction = self.shift_points_perpendicular_ccw(
            self.static_horizontal_curb_xy_coordinates[0],
            self.curb_0_xy_shifted,
            self.parking_lot_axis_shift_margin
        )

        # Project vehicle pose onto the search axis
        self.vehicle_pose_proj = self.project_point_on_axis(
            self.curb_0_xy_shifted,
            self.curb_1_xy_shifted,
            self.vehicle_pose[0:2]
        )

        # Compute bounds for the search axis
        self.upper_bound_xy = self.move_point_along_vector(
            self.curb_1_xy_shifted,
            self.search_axis_direction_var,
            step=2 * self.compact_parking_spot_size[1],
            positive_direction=True
        )

        self.lower_bound_xy = self.move_point_along_vector(
            self.curb_0_xy_shifted,
            self.search_axis_direction_var,
            step=2 * self.compact_parking_spot_size[1],
            positive_direction=False
        )




    
    def find_collision_free_trajectory_to_park(self, detected_cones=[], vehicle_pose=(0.0, 0.0, 0.0), update_pose=False):
        # Update detected cones and optionally vehicle pose
        self.detected_cones = detected_cones
        if update_pose:
            self.vehicle_pose = vehicle_pose

        if not self.available_parking_spots_var:    
           self.waypoints_to_go = []
           print("No parking spot available.")
           return
        # Try in both directions
        directions = [self.x_axis_of_search_direction_positive, not self.x_axis_of_search_direction_positive]  
        for direction_flag in directions:
            self.x_axis_of_search_direction_positive = direction_flag
            self.vehicle_pose_proj = self.project_point_on_axis(
            self.curb_0_xy_shifted,
            self.curb_1_xy_shifted,
            self.vehicle_pose[0:2]
        )


            while True:
                # Move projected pose along the search axis
                self.vehicle_pose_proj = self.move_point_along_vector(
                    self.vehicle_pose_proj,
                    self.search_axis_direction_var,
                    step=self.search_step_size,
                    positive_direction=self.x_axis_of_search_direction_positive
                )

                # Compute the projected vehicle pose
                start_proj = (
                    self.vehicle_pose_proj[0],# - self.shift_from_center_to_rear_axis,
                    self.vehicle_pose_proj[1],
                    self.yaw_of_parked_cars_var
                )
                
                # Plan path in segments:
                # If 3 parking spots are available, park with clearance
                if self.priority  == 1:
                   clearance_step = self.clearance_step
                else:
                   clearance_step = 0.0    

                # Compute the parking spot minus some clearance
                self.parking_spot_to_go_minus_clearance = self.move_point_along_vector(
                    self.parking_spot_to_go[0][0:2],
                    self.search_axis_direction_var,
                    step=clearance_step, # TODO: Pass as an input
                    positive_direction=False
                )

                # Adding yaw
                self.parking_spot_to_go_minus_clearance = (self.parking_spot_to_go_minus_clearance[0],
                                                           self.parking_spot_to_go_minus_clearance[1], 
                                                           self.yaw_of_parked_cars_var)
                
                # Poses to connect with reeds-shepp paths
                waypoints_to_connect = [self.vehicle_pose, 
                                        start_proj, 
                                        self.parking_spot_to_go_minus_clearance, 
                                        self.parking_spot_to_go[0]]
                
                # Computing the reeds-shepp paths
                self.waypoints_for_obstacles_check = self.stitch_paths(
                        waypoints_to_connect,
                        step_size=self.search_step_size,
                        turning_radius=self.vehicle_turning_radius
                    )
                # Extract waypoints to pass (removing yaw)
                self.waypoints_to_go = np.array(self.waypoints_for_obstacles_check)[:, :2]

                # Check if trajectory is collision-free
                if self.is_trajectory_collision_free(
                    self.waypoints_for_obstacles_check,
                    self.vehicle_dims,
                    self.objects_to_avoid_collisions
                ):
                    return

                # Stop search if bounds are reached
                dist_to_upper_bound = np.linalg.norm(np.array(self.vehicle_pose_proj) - np.array(self.upper_bound_xy))
                dist_to_lower_bound = np.linalg.norm(np.array(self.vehicle_pose_proj) - np.array(self.lower_bound_xy))
                if dist_to_upper_bound < self.search_bound_threshold or dist_to_lower_bound < self.search_bound_threshold:
                    
                    # TODO: Also implement the horizontal search axis direction by accumulating points in "waypoints_to_connect".
                    # If car can fit in the parking spot, then parking path should exits if there are
                    # no obstacles across holonomic paths.
                    # Use self.horizontal_search_axis_direction_var
                    break  # Give up in this direction

        # If both directions fail        
        raise ValueError("No collision-free trajectory available in either direction for parking.")   

                                                                            

    def find_collision_free_trajectory_to_unpark(self, detected_cones=[], vehicle_pose=(0.0, 0.0, 0.0), update_pose=False):
        # Update detected cones and optionally vehicle pose
        self.detected_cones = detected_cones
        if update_pose:
            self.vehicle_pose = vehicle_pose 

        # Find current vehicle pose projected on the search axis
        self.vehicle_pose_proj = self.project_point_on_axis(
            self.curb_0_xy_shifted,
            self.curb_1_xy_shifted,
            self.vehicle_pose[0:2]
        )           

        # Move projected pose along the search axis
        self.vehicle_pose_proj = self.move_point_along_vector(
            self.vehicle_pose_proj,
            self.search_axis_direction_var,
            step=self.compact_parking_spot_size[1]*2,
            positive_direction=True
        )

        # Compute the projected vehicle pose by adding yaw
        start_proj = (
            self.vehicle_pose_proj[0],# - self.shift_from_center_to_rear_axis,
            self.vehicle_pose_proj[1],
            self.yaw_of_parked_cars_var
        )
        
        
        while True:
            # Move projected pose along the search axis
            self.vehicle_pose_proj = self.move_point_along_vector(
                self.vehicle_pose_proj,
                self.search_axis_direction_var,
                step=self.search_step_size,
                positive_direction=False
            )

            # Compute the projected vehicle pose
            start_proj = (
                self.vehicle_pose_proj[0],# - self.shift_from_center_to_rear_axis,
                self.vehicle_pose_proj[1],
                self.yaw_of_parked_cars_var
            )

            waypoints_1 = self.reeds_shepp_path(
                    self.vehicle_pose,
                    start_proj,
                    step_size=self.search_step_size,
                    vehicle_turning_radius = self.vehicle_turning_radius 
                )
            self.waypoints_for_obstacles_check = waypoints_1
            self.waypoints_to_go = np.array(self.waypoints_for_obstacles_check)[:, :2]

            # Check if trajectory is collision-free
            if self.is_trajectory_collision_free(
                self.waypoints_for_obstacles_check,
                self.vehicle_dims,
                self.objects_to_avoid_collisions
            ):
                return
            
            # Stop search if bounds are reached
            dist_to_upper_bound = np.linalg.norm(np.array(self.vehicle_pose_proj) - np.array(self.upper_bound_xy))
            dist_to_lower_bound = np.linalg.norm(np.array(self.vehicle_pose_proj) - np.array(self.lower_bound_xy))
            if dist_to_upper_bound < self.search_bound_threshold or dist_to_lower_bound < self.search_bound_threshold:
                break  # Give up in this direction

        # If both directions fail        
        raise ValueError("No collision-free trajectory available for unparking.")       
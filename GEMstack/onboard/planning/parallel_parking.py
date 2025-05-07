from ..component import Component
from ...utils import serialization
from ...state import Route,ObjectFrameEnum, AllState, VehicleState, Roadgraph, MissionObjective
from ..interface.gem import GEMInterface,GEMVehicleCommand,GEMVehicleReading
from ..planning.reeds_shepp_parking import ReedsSheppParking
import os
import numpy as np
from typing import List

class SummoningParkingRoutePlanner(Component):
    def __init__(self, routefn: str, vehicle_interface: GEMInterface, frame: str = 'start'):
        self.vehicle_interface = vehicle_interface
        self.routefn = routefn
        base, ext = os.path.splitext(routefn)
        if ext in ['.json', '.yml', '.yaml']:
            with open(routefn, 'r') as f:
                self.route = serialization.load(f)
        elif ext == '.csv':
            waypoints = np.loadtxt(routefn, delimiter=',', dtype=float)
            if waypoints.shape[1] == 3:
                waypoints = waypoints[0:300, :2]
            if frame == 'start':
                self.route = Route(frame=ObjectFrameEnum.START, points=waypoints.tolist())
            elif frame == 'global':
                self.route = Route(frame=ObjectFrameEnum.GLOBAL, points=waypoints.tolist())
            elif frame == 'cartesian':
                self.route = Route(frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN, points=waypoints.tolist())
            else:
                raise ValueError("Unknown route frame {} must be start, global, or cartesian".format(frame))
        else:
            raise ValueError("Unknown route file extension", ext)
        
        # SLOT 1 (2.69, -2.44), SLOT 2  (7.57, -2.44)
        # SLOT 3 (12.45, -2.44), SLOT 4 (17.33, -2.44)
        # SLOT 5 (22.11, -2.44) 

        self.parked_cars = [
            (2.69, -2.44),
            (22.11, -2.44) 
        ]

        self.reedssheppparking = ReedsSheppParking()
        # self.reedssheppparking.closest = True
        # self.reedssheppparking.add_static_vertical_curb_as_obstacle = False
        # self.reedssheppparking.add_static_horizontal_curb_as_obstacle = False
        self.reedssheppparking.static_horizontal_curb_xy_coordinates = [(0.0, -2.44),(24.9, -2.44)]
        self.reedssheppparking.find_available_parking_spots_and_search_vector(self.parked_cars)
        self.reedssheppparking.find_collision_free_trajectory(self.parked_cars)

        


    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 10.0

    def update(self, vehicle: VehicleState, x=0.0):
        self.current_pose = vehicle.pose
        self.waypoints_to_go = self.reedssheppparking.waypoints_to_go
        self.route = Route(frame=ObjectFrameEnum.START, points=self.waypoints_to_go.tolist())
        return self.route
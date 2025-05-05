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
        
        # SLOT 1 (2.69, -2.44), SLOT 2  (7.47, -2.44)
        # SLOT 3 (12.25, -2.44), SLOT 4 (17.33, -2.44)
        # SLOT 5 (22.11, -2.44) 

        self.parked_cars = [
            
            (7.47, -2.44),
            (12.25, -2.44),
            (17.33, -2.44),
            (22.11, -2.44)
             
        ]
        self.all_parking_spots_in_parking_lot = [
            (3.94, -2.44, 0.0, (2.44, 4.88)),
            (7.47, -2.44, 0.0, (2.44, 4.88)),
            (12.25, -2.44, 0.0, (2.44, 4.88)),
            (17.33, -2.44, 0.0, (2.44, 4.88)),
            (22.11, -2.44, 0.0,(2.44, 4.88))
        ]
        self.reedssheppparking = ReedsSheppParking()
        self.reedssheppparking.closest = False
        self.reedssheppparking.all_parking_spots_in_parking_lot = self.all_parking_spots_in_parking_lot
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
        #self.reedssheppparking.find_collision_free_trajectory(self.parked_cars)
        self.waypoints_to_go = self.reedssheppparking.waypoints_to_go
        self.route = Route(frame=ObjectFrameEnum.START, points=self.waypoints_to_go.tolist())
        return self.route
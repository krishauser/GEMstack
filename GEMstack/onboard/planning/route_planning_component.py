import os
from typing import Dict, List
import numpy as np
from GEMstack.onboard.component import Component
from GEMstack.state.agent import AgentState
from GEMstack.state.all import AllState
from GEMstack.state.physical_object import ObjectFrameEnum
from GEMstack.state.route import PlannerEnum, Route, Path
from GEMstack.state.trajectory import Trajectory
from .rrt_star import RRTStar
from .parking_route_planner import ParkingPlanner
from .parking_scanning import StraightLineMotion
from .longitudinal_planning import longitudinal_plan
import time
import datetime

class RoutePlanningComponent(Component):
    """Reads a route from disk and returns it as the desired route."""
    def __init__(self):
        print("Route Planning Component init")
        self.route = None
        self.planner = None
        self.compute_parking_route = False
        self.done_computing = False
        self.already_computed = False
        self.planning_time = 0.0
        self.last_planning_time = 0.0
        
        # Create logs directory if it doesn't exist
        self.logs_dir = "logs"
        if not os.path.exists(self.logs_dir):
            os.makedirs(self.logs_dir)
            
        # Create metrics log file with timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.metrics_file = os.path.join(self.logs_dir, f"parking_metrics_{timestamp}.csv")
        
        # Write header to metrics file
        with open(self.metrics_file, 'w') as f:
            f.write("Timestamp,Planning Time,Previous Planning Time,Time Difference," + 
                   "Goal X,Goal Y,Goal Yaw," +
                   "Final X,Final Y,Final Yaw," +
                   "Position Error,Orientation Error\n")
        
    def log_metrics(self, state: AllState, final_state=None):
        """Log metrics to file with timestamp"""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        
        # Calculate position and orientation errors if we have final state
        position_error = 0.0
        orientation_error = 0.0
        final_x = 0.0
        final_y = 0.0
        final_yaw = 0.0
        
        if final_state is not None:
            final_x = final_state[0]
            final_y = final_state[1]
            final_yaw = final_state[2]
            
            # Calculate position error (Euclidean distance)
            position_error = np.sqrt(
                (final_x - state.mission_plan.goal_x)**2 + 
                (final_y - state.mission_plan.goal_y)**2
            )
            
            # Calculate orientation error (absolute difference)
            orientation_error = abs(final_yaw - state.mission_plan.goal_orientation)
            # Normalize orientation error to [-pi, pi]
            while orientation_error > np.pi:
                orientation_error -= 2 * np.pi
            orientation_error = abs(orientation_error)
        
        metrics = [
            timestamp,
            f"{self.planning_time:.3f}",
            f"{self.last_planning_time:.3f}",
            f"{self.planning_time - self.last_planning_time:.3f}",
            f"{state.mission_plan.goal_x:.3f}",
            f"{state.mission_plan.goal_y:.3f}",
            f"{state.mission_plan.goal_orientation:.3f}",
            f"{final_x:.3f}",
            f"{final_y:.3f}",
            f"{final_yaw:.3f}",
            f"{position_error:.3f}",
            f"{orientation_error:.3f}"
        ]
        
        with open(self.metrics_file, 'a') as f:
            f.write(','.join(metrics) + '\n')
        
    def state_inputs(self):
        return ["all"]

    def state_outputs(self) -> List[str]:
        return ['route']

    def rate(self):
        return 10.0 # very high for our computation ability

    def update(self, state: AllState):
        if state.mission_plan.planner_type.name == "PARKING" and not self.compute_parking_route:
            print("I am in PARKING mode")
            desired_points = [(state.vehicle.pose.x, state.vehicle.pose.y),
                              (state.vehicle.pose.x, state.vehicle.pose.y)]
            desired_path = Path(state.vehicle.pose.frame, desired_points)

            self.compute_parking_route = True
            return desired_path
        elif state.mission_plan.planner_type.name == "PARKING" and self.compute_parking_route and not self.done_computing:
            state.vehicle.pose.yaw = 0 # needed this to avoid a weird error in the parking planner
            
            if not self.already_computed:
                print("\n=== Starting Parking Route Planning ===")
                start_time = time.time()
                
                self.planner = ParkingPlanner()
                self.done_computing = True
                self.route = self.planner.update(state)
                
                # Calculate planning time
                self.planning_time = time.time() - start_time
                
                # Get final state from the route
                final_state = None
                if self.route and len(self.route.points) > 0:
                    # Get the last point from the route
                    final_point = self.route.points[-1]
                    # Get the final orientation from the planner's last state
                    if hasattr(self.planner, 'planner') and hasattr(self.planner.planner, 'last_state'):
                        final_state = self.planner.planner.last_state
                
                # Log metrics to file
                self.log_metrics(state, final_state)
                
                print("=== Planning Complete ===\n")
                self.planner.visualize_trajectory(self.route)
                self.already_computed = True
            return self.route
        elif state.mission_plan.planner_type.name == "RRT_STAR":
            print("I am in RRT mode")
        elif state.mission_plan.planner_type.name == "SCANNING":
            print("I am in SCANNING mode")
            desired_points = [(state.vehicle.pose.x, state.vehicle.pose.y),
                              (state.vehicle.pose.x + 1, state.vehicle.pose.y)]
            desired_path = Path(state.vehicle.pose.frame, desired_points)
            return desired_path
        
        return None
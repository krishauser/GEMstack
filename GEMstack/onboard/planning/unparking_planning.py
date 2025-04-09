from typing import List
from enum import Enum
from ..component import Component
from ...state import AllState, VehicleIntent, VehicleIntentEnum
import math
import time

class DrivingLogic(Component):
    """Component that handles driving logic with specific focus on unparking functionality."""
    
    # Unparking state machine states
    STATE_PARKED = 0
    STATE_PREPARING_EXIT = 1
    STATE_EXITING_SPOT = 2
    STATE_JOINING_TRAFFIC = 3
    STATE_NORMAL_DRIVING = 4
    
    def rate(self) -> float:
        """Returns the rate in Hz at which this component should be updated."""
        return 2.0  # Higher rate for more responsive driving
    
    def state_inputs(self) -> List[str]:
        """Returns the list of AllState inputs this component requires."""
        return ['all', 'perception', 'localization']
    
    def state_outputs(self) -> List[str]:
        """Returns the list of AllState outputs this component generates."""
        return ['intent', 'trajectory']
    
    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True
    
    def initialize(self):
        """Initialize the component. This is called once before the first update."""
        self.unparking_state = self.STATE_PARKED
        self.state_change_time = time.time()
        self.exit_path = None
        self.debug_event("DrivingLogic initialized in parked state")
        return
    
    def cleanup(self):
        """Cleans up resources used by the component."""
        self.debug_event("DrivingLogic cleanup")
        pass
    
    def update(self, state: AllState):
        """Update the component based on current state."""
        # Update vehicle intent based on unparking state
        if self.unparking_state == self.STATE_PARKED:
            state.intent.intent = VehicleIntentEnum.PARKED
            
            # Check if we should start unparking
            if self._should_start_unparking(state):
                self._enter_preparing_exit_state(state)
        
        elif self.unparking_state == self.STATE_PREPARING_EXIT:
            state.intent.intent = VehicleIntentEnum.EXITING
            
            # Check if exit preparation is complete (e.g. path planning done)
            if self._exit_preparation_complete(state):
                self._enter_exiting_spot_state(state)
                
        elif self.unparking_state == self.STATE_EXITING_SPOT:
            state.intent.intent = VehicleIntentEnum.EXITING
            
            # Check if vehicle has exited the spot
            if self._spot_exit_complete(state):
                self._enter_joining_traffic_state(state)
                
        elif self.unparking_state == self.STATE_JOINING_TRAFFIC:
            state.intent.intent = VehicleIntentEnum.LEAVING_PARKING
            
            # Check if vehicle has successfully joined traffic
            if self._traffic_join_complete(state):
                self._enter_normal_driving_state(state)
                
        elif self.unparking_state == self.STATE_NORMAL_DRIVING:
            state.intent.intent = VehicleIntentEnum.NORMAL_DRIVING
        
        # Debug current state
        self.debug("unparking_state", self.unparking_state)
        self.debug("intent", state.intent.intent)
        
        # Set trajectory if available
        if hasattr(state, 'trajectory') and self.unparking_state != self.STATE_PARKED:
            self._update_trajectory(state)
    
    def _should_start_unparking(self, state: AllState) -> bool:
        """Determine if the vehicle should start unparking."""
        # In a real implementation, this could be triggered by:
        # - User request
        # - Scheduled departure time
        # - External command
        
        # For simplicity, trigger after a fixed delay
        return time.time() - self.state_change_time > 5.0
    
    def _exit_preparation_complete(self, state: AllState) -> bool:
        """Check if exit preparation is complete."""
        # In a real implementation:
        # - Path planning would be done
        # - Surroundings would be checked for obstacles
        # - Signals would be activated (turn signals)
        
        # Generate a simple exit path if none exists
        if not self.exit_path and hasattr(state, 'localization'):
            self.exit_path = self._plan_exit_path(state)
            
        # Simulated delay for preparation
        return time.time() - self.state_change_time > 2.0
    
    def _spot_exit_complete(self, state: AllState) -> bool:
        """Check if the vehicle has exited the parking spot."""
        # In a real implementation, would check:
        # - Distance moved from initial position
        # - Orientation change
        # - Proximity to obstacles
        
        # Simulated time-based exit
        return time.time() - self.state_change_time > 4.0
    
    def _traffic_join_complete(self, state: AllState) -> bool:
        """Check if the vehicle has successfully joined traffic."""
        # In a real implementation, would check:
        # - Alignment with travel lane
        # - Speed matching with traffic flow
        # - Safe distance to other vehicles
        
        # Simulated time-based join
        return time.time() - self.state_change_time > 3.0
    
    def _plan_exit_path(self, state: AllState) -> dict:
        """Plan path for exiting the parking spot."""
        # In a real implementation, would use:
        # - Current position and orientation
        # - Parking spot type (parallel, perpendicular)
        # - Obstacle detection
        # - Path planning algorithms
        
        # Simplified example exit path
        current_x = state.localization.x
        current_y = state.localization.y
        current_heading = state.localization.heading
        
        # Simple path with waypoints
        return {
            'type': 'parallel',  # or 'perpendicular'
            'waypoints': [
                {'x': current_x, 'y': current_y, 'speed': 0.0},  # start position
                {'x': current_x - 1.0, 'y': current_y, 'speed': 0.5},  # back up
                {'x': current_x - 1.0, 'y': current_y + 1.0, 'speed': 1.0},  # turn out
                {'x': current_x + 2.0, 'y': current_y + 2.0, 'speed': 2.0}   # join lane
            ]
        }
    
    def _update_trajectory(self, state: AllState):
        """Update vehicle trajectory based on current state and exit path."""
        if not self.exit_path:
            return
            
        # In a real implementation, would:
        # - Calculate current position in the path
        # - Set appropriate speed and steering
        # - Handle obstacle avoidance
        
        # Simplified example: set speed based on state
        if self.unparking_state == self.STATE_PREPARING_EXIT:
            state.trajectory.target_speed = 0.0  # stopped, preparing
        elif self.unparking_state == self.STATE_EXITING_SPOT:
            state.trajectory.target_speed = 0.5  # slow exit
        elif self.unparking_state == self.STATE_JOINING_TRAFFIC:
            state.trajectory.target_speed = 2.0  # accelerating
        elif self.unparking_state == self.STATE_NORMAL_DRIVING:
            state.trajectory.target_speed = 5.0  # normal speed
    
    # State transition methods
    def _enter_preparing_exit_state(self, state: AllState):
        """Enter the preparing exit state."""
        self.unparking_state = self.STATE_PREPARING_EXIT
        self.state_change_time = time.time()
        self.debug_event("Preparing to exit parking spot")
    
    def _enter_exiting_spot_state(self, state: AllState):
        """Enter the exiting spot state."""
        self.unparking_state = self.STATE_EXITING_SPOT
        self.state_change_time = time.time()
        self.debug_event("Executing exit maneuver")
    
    def _enter_joining_traffic_state(self, state: AllState):
        """Enter the joining traffic state."""
        self.unparking_state = self.STATE_JOINING_TRAFFIC
        self.state_change_time = time.time()
        self.debug_event("Joining traffic flow")
    
    def _enter_normal_driving_state(self, state: AllState):
        """Enter the normal driving state."""
        self.unparking_state = self.STATE_NORMAL_DRIVING
        self.state_change_time = time.time()
        self.exit_path = None
        self.debug_event("Resumed normal driving")
    
    def debug(self, item, value):
        """Debugs a streaming value within this component"""
        if hasattr(self, 'debugger'):
            self.debugger.debug(item, value)
    
    def debug_event(self, label):
        """Debugs an event within this component"""
        if hasattr(self, 'debugger'):
            self.debugger.debug_event(label)
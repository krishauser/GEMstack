from typing import List
from ..component import Component
from ...state import AllState, VehicleIntent
import time
import math

class ParkingLogic(Component):
    """
    Component that handles parking logic for GEM vehicles.
    This includes finding a parking spot and executing parking maneuvers.
    """
    # Intent codes
    INTENT_NORMAL_DRIVING = 0
    INTENT_SEARCHING = 1
    INTENT_PARKING = 2
    INTENT_PARKED = 3
    INTENT_EXITING = 4

    # Parking state machine states
    STATE_IDLE = 0
    STATE_SEARCHING = 1
    STATE_APPROACH = 2
    STATE_ALIGNING = 3
    STATE_BACKING = 4
    STATE_ADJUSTING = 5
    STATE_PARKED = 6
    STATE_EXITING = 7

    def rate(self) -> float:
        """Returns the rate in Hz at which this component should be updated."""
        return 10.0  # Higher rate for more responsive parking

    def state_inputs(self) -> List[str]:
        """Returns the list of AllState inputs this component requires."""
        return ['all', 'perception', 'localization']

    def state_outputs(self) -> List[str]:
        """Returns the list of AllState outputs this component generates."""
        return ['intent', 'trajectory']

    def healthy(self):
        """Returns True if the element is in a stable state."""
        return self.error_count < 5  # Consider unhealthy after multiple errors

    def initialize(self):
        """Initialize the component. This is called once before the first update."""
        self.parking_state = self.STATE_IDLE
        self.spot_found = False
        self.error_count = 0
        self.start_time = time.time()
        self.last_state_change = time.time()
        
        # Parking spot characteristics
        self.min_spot_length = 5.0  # meters
        self.min_spot_width = 2.5   # meters
        
        # Vehicle characteristics (example values)
        self.vehicle_length = 3.5   # meters
        self.vehicle_width = 1.5    # meters
        
        # Parking maneuver parameters
        self.approach_distance = 1.0  # meters ahead of spot
        self.backing_speed = 0.5      # m/s
        self.forward_speed = 1.0      # m/s
        
        # For detecting a spot
        self.last_obstacles = []
        self.potential_spots = []
        
        # Parking spot once found
        self.target_spot = None
        
        # Debug information
        self.debug("initialized", True)
        self.debug_event("ParkingLogic initialized")
        
    def cleanup(self):
        """Cleans up resources used by the component."""
        self.debug_event("ParkingLogic cleanup")
        pass

    def update(self, state: AllState):
        """Update the component based on current state."""
        try:
            if self.parking_state == self.STATE_IDLE:
                # Enter searching state
                self.parking_state = self.STATE_SEARCHING
                self.debug_event("Starting to search for parking spot")
                state.intent.intent = self.INTENT_SEARCHING
                
            elif self.parking_state == self.STATE_SEARCHING:
                # Logic to search for a parking spot using perception data
                if self._find_parking_spot(state):
                    self.parking_state = self.STATE_APPROACH
                    self.debug_event(f"Parking spot found at {self.target_spot}")
                    state.intent.intent = self.INTENT_PARKING
                else:
                    # Continue searching
                    self._maintain_search_pattern(state)
                    
            elif self.parking_state == self.STATE_APPROACH:
                # Approach the parking spot
                if self._approach_parking_spot(state):
                    self.parking_state = self.STATE_ALIGNING
                    self.debug_event("Aligned with parking spot")
                
            elif self.parking_state == self.STATE_ALIGNING:
                # Align the vehicle parallel to the parking spot
                if self._align_with_spot(state):
                    self.parking_state = self.STATE_BACKING
                    self.debug_event("Starting to back into spot")
                
            elif self.parking_state == self.STATE_BACKING:
                # Back into the parking spot
                if self._back_into_spot(state):
                    self.parking_state = self.STATE_ADJUSTING
                    self.debug_event("Backed into spot, making final adjustments")
                
            elif self.parking_state == self.STATE_ADJUSTING:
                # Make final adjustments to center in the spot
                if self._adjust_position(state):
                    self.parking_state = self.STATE_PARKED
                    self.debug_event("Successfully parked")
                    state.intent.intent = self.INTENT_PARKED
                
            elif self.parking_state == self.STATE_PARKED:
                # Stay parked until requested to exit
                if self._should_exit_parking(state):
                    self.parking_state = self.STATE_EXITING
                    self.debug_event("Starting to exit parking spot")
                    state.intent.intent = self.INTENT_EXITING
                
            elif self.parking_state == self.STATE_EXITING:
                # Exit the parking spot
                if self._exit_parking_spot(state):
                    self.parking_state = self.STATE_IDLE
                    self.debug_event("Exited parking spot, resuming normal operation")
                    state.intent.intent = self.INTENT_NORMAL_DRIVING
                    
            # Log current state for debugging
            self.debug("parking_state", self.parking_state)
            self.debug("intent", state.intent.intent)
            
        except Exception as e:
            self.error_count += 1
            self.debug("error", str(e))
            self.debug_event(f"Error in parking logic: {str(e)}")
            
    def _find_parking_spot(self, state: AllState) -> bool:
        """
        Use perception data to find a suitable parking spot.
        Returns True if a spot is found, False otherwise.
        """
        # Example implementation using perception data
        # In a real implementation, you would use perception data to identify open spaces
        
        if not hasattr(state, 'perception') or not hasattr(state.perception, 'obstacles'):
            return False
            
        current_obstacles = state.perception.obstacles
        
        # Simple spot detection logic: look for gaps between obstacles
        # that are large enough for the vehicle
        potential_spots = []
        
        # For parallel parking, look for gaps along the side of the road
        if hasattr(state, 'localization') and hasattr(state.localization, 'lane_position'):
            # Get current lane information
            current_lane = state.localization.lane_position
            
            # Example code to detect gaps - this would be much more sophisticated in reality
            # using advanced perception and mapping
            gaps = self._detect_gaps(current_obstacles, current_lane)
            
            for gap in gaps:
                if gap['length'] >= self.min_spot_length and gap['width'] >= self.min_spot_width:
                    potential_spots.append(gap)
        
        # If we found at least one spot, select the best one
        if potential_spots:
            # For this demo, just select the first valid spot
            self.target_spot = potential_spots[0]
            return True
            
        return False
        
    def _detect_gaps(self, obstacles, lane_info):
        """
        Analyze obstacle data to find gaps that could be parking spots.
        This is a simplified implementation.
        """
        # In a real implementation, this would use sophisticated algorithms
        # to detect gaps between vehicles or in designated parking areas
        
        # Placeholder example
        gaps = []
        
        # Simulate finding a gap after a certain time for demo purposes
        if time.time() - self.start_time > 10.0:  # After 10 seconds, "find" a spot
            # Generate a simulated spot relative to current position
            example_gap = {
                'length': self.min_spot_length + 0.5,  # A bit larger than minimum
                'width': self.min_spot_width + 0.2,
                'position': {
                    'x': lane_info.x + 5.0,  # 5 meters ahead
                    'y': lane_info.y + 2.0,  # 2 meters to the right
                    'heading': lane_info.heading
                },
                'type': 'parallel'  # Could also be 'perpendicular'
            }
            gaps.append(example_gap)
            
        return gaps
        
    def _maintain_search_pattern(self, state: AllState):
        """Maintain a driving pattern while searching for parking spots."""
        # In a real implementation, this would follow a search route
        # For the demo, just continue with normal driving
        state.intent.intent = self.INTENT_SEARCHING
        
    def _approach_parking_spot(self, state: AllState) -> bool:
        """
        Move the vehicle to the approach position for the target parking spot.
        Returns True when in position.
        """
        if not self.target_spot:
            return False
            
        # Calculate approach position (would use proper path planning in real implementation)
        target_x = self.target_spot['position']['x'] - self.approach_distance
        target_y = self.target_spot['position']['y']
        
        # Check if we've reached the approach position
        current_x = state.localization.x
        current_y = state.localization.y
        
        distance_to_approach = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # Update trajectory to approach position
        if hasattr(state, 'trajectory'):
            # Set trajectory to approach position
            # (This is a simplified placeholder - real implementation would
            # generate proper trajectory points)
            state.trajectory.target_speed = self.forward_speed
            
        # Return True if we're close enough to approach position
        return distance_to_approach < 0.3  # meters
        
    def _align_with_spot(self, state: AllState) -> bool:
        """
        Align the vehicle parallel to the parking spot.
        Returns True when aligned.
        """
        if not self.target_spot:
            return False
            
        target_heading = self.target_spot['position']['heading']
        current_heading = state.localization.heading
        
        # Calculate heading difference (accounting for circular values)
        heading_diff = abs((target_heading - current_heading + 180) % 360 - 180)
        
        # Update trajectory for alignment
        if hasattr(state, 'trajectory'):
            # Would set a trajectory that aligns with target heading
            state.trajectory.target_speed = 0.5  # Slow for precise alignment
            
        # Return True if aligned within threshold
        return heading_diff < 5.0  # degrees
        
    def _back_into_spot(self, state: AllState) -> bool:
        """
        Execute backing maneuver into the parking spot.
        Returns True when in the spot.
        """
        if not self.target_spot:
            return False
            
        # Calculate spot center
        spot_center_x = self.target_spot['position']['x']
        spot_center_y = self.target_spot['position']['y']
        
        # Current position
        current_x = state.localization.x
        current_y = state.localization.y
        
        # Distance to spot center
        distance_to_center = math.sqrt((spot_center_x - current_x)**2 + (spot_center_y - current_y)**2)
        
        # Update trajectory for backing
        if hasattr(state, 'trajectory'):
            # Would set reverse trajectory into spot
            state.trajectory.target_speed = -self.backing_speed  # Negative for reverse
            
        # Check if we've reached an appropriate position in the spot
        # In real implementation, would use more sophisticated positioning
        return distance_to_center < 0.5  # meters
        
    def _adjust_position(self, state: AllState) -> bool:
        """
        Make final adjustments to center in the parking spot.
        Returns True when properly centered.
        """
        # Simple implementation - in reality would use precise positioning
        # For demo, just assume adjustment is complete after a delay
        adjustment_time = 3.0  # seconds
        
        if time.time() - self.last_state_change > adjustment_time:
            return True
            
        # Would set small adjustment trajectories
        if hasattr(state, 'trajectory'):
            state.trajectory.target_speed = 0.2  # Very slow for fine adjustments
            
        return False
        
    def _should_exit_parking(self, state: AllState) -> bool:
        """
        Determine if the vehicle should exit the parking spot.
        In a real implementation, this could be triggered by user input
        or a scheduled departure time.
        """
        # For demo purposes, exit after 10 seconds of being parked
        parked_duration = time.time() - self.last_state_change
        return parked_duration > 10.0
        
    def _exit_parking_spot(self, state: AllState) -> bool:
        """
        Execute maneuver to exit the parking spot.
        Returns True when back on the road.
        """
        # Simple implementation - in reality would plan a path out
        # For demo, just assume exit is complete after a delay
        exit_time = 5.0  # seconds
        
        if time.time() - self.last_state_change > exit_time:
            # Reset parking state
            self.target_spot = None
            return True
            
        # Would set exit trajectory
        if hasattr(state, 'trajectory'):
            state.trajectory.target_speed = self.forward_speed
            
        return False
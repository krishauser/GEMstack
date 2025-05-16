from ..component import Component
import rospy
from gazebo_msgs.msg import ContactsState
from collections import deque
import time

class GazeboCollisionLogger(Component):
    """Logs collision data from Gazebo simulation."""
    
    def __init__(self, vehicle_interface):
        super().__init__()
        self.vehicle_interface = vehicle_interface
        self.collision_messages = deque(maxlen=100)
        self.last_collision_time = None
        self.last_collision_pair = None

        
    def initialize(self):
        """Initialize the collision logger."""
        self.contact_sub = rospy.Subscriber('/contact_sensor', ContactsState, self.contact_callback)
        rospy.loginfo("Gazebo collision logger initialized")
    
    def simplify_collision_name(self, name):
        """Simplify collision names to be more readable."""
        if "base_link" in name:
            return "vehicle body"
        elif "front_rack" in name:
            return "vehicle front bumper"
        elif "rear_rack" in name:
            return "vehicle rear bumper"
        elif "::" in name:
            return name.split("::")[0]
        return name
    
    def contact_callback(self, msg):
        """Callback for processing collision messages."""
        if not msg.states:
            return
            
        current_time = time.time()
        
        # Process all collision states
        for state in msg.states:
            collision1 = self.simplify_collision_name(state.collision1_name)
            collision2 = self.simplify_collision_name(state.collision2_name)
            
            # Reorder collisions to put vehicle parts first
            if "vehicle" in collision2:
                collision1, collision2 = collision2, collision1
            
            collision_pair = tuple(sorted([collision1, collision2]))
            
            # Only log if it's a new collision or enough time has passed
            if (self.last_collision_time is None or 
                current_time - self.last_collision_time > 0.25 or  # Different time
                collision_pair != self.last_collision_pair):  # Different collision pair
                
                pos = state.contact_positions[0]
                normal = state.contact_normals[0]
                
                message = (
                    f"Collision between: {collision1} and {collision2}\n"
                    f"Contact Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}\n"
                    f"Contact Normal: x={normal.x:.3f}, y={normal.y:.3f}, z={normal.z:.3f}\n"
                    f"Contact Depth: {state.depths[0]:.3f}\n"
                    f"{'-' * 50}"
                )
                
                self.collision_messages.append(message)
                self.last_collision_time = current_time
                self.last_collision_pair = collision_pair
    
    def update(self):
        """Output collision messages to the console/log."""
        while self.collision_messages:
            print(self.collision_messages.popleft()) 
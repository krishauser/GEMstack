from ...utils import settings
from ..component import Component
import rospy
from gazebo_msgs.msg import ContactsState
from datetime import datetime
import os

class GazeboCollisionLogger(Component):
    """Component that logs collision data from Gazebo simulation."""
    
    def __init__(self, vehicle_interface):
        super().__init__()
        self.vehicle_interface = vehicle_interface
        self.log_file = None
        self.initialized = False
        
    def initialize(self):
        """Initialize the collision logger component."""
        if not self.initialized:
            # Get log folder from settings
            log_settings = settings.get('run.log', {})
            topfolder = log_settings.get('folder', 'logs')
            prefix = log_settings.get('prefix', '')
            suffix = log_settings.get('suffix', datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
            logfolder = os.path.join(topfolder, prefix + suffix)
            
            # Create collision log file
            self.log_file = os.path.join(logfolder, 'collision_log.txt')
            os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
            
            # Subscribe to contact sensor topic
            rospy.Subscriber('/contact_sensor', ContactsState, self.contact_callback)
            
            self.initialized = True
            rospy.loginfo("Collision logger initialized. Logging to: %s", self.log_file)
            rospy.loginfo("Waiting for collision events...")
    
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
        if not msg.states:  # If no contacts
            return
            
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        # Create a dictionary to store unique collisions
        unique_collisions = {}
        
        # Process all collision states
        for state in msg.states:
            collision_key = tuple(sorted([state.collision1_name, state.collision2_name]))
            if collision_key not in unique_collisions or state.depths[0] > unique_collisions[collision_key].depths[0]:
                unique_collisions[collision_key] = state
        
        # Log collision information
        rospy.loginfo("\n=== Collision Detected at %s ===", timestamp)
        
        with open(self.log_file, 'a') as f:
            f.write(f"\n=== Collision Detected at {timestamp} ===\n")
            
            for i, (collision_key, state) in enumerate(unique_collisions.items(), 1):
                collision1 = self.simplify_collision_name(state.collision1_name)
                collision2 = self.simplify_collision_name(state.collision2_name)
                
                # Reorder collisions to put vehicle parts first
                if "vehicle" in collision2:
                    collision1, collision2 = collision2, collision1
                
                # Log collision details
                rospy.loginfo("\nContact %d:", i)
                rospy.loginfo("Collision between: %s and %s", collision1, collision2)
                
                pos = state.contact_positions[0]
                pos_str = f"Contact Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}"
                rospy.loginfo(pos_str)
                
                normal = state.contact_normals[0]
                normal_str = f"Contact Normal: x={normal.x:.3f}, y={normal.y:.3f}, z={normal.z:.3f}"
                rospy.loginfo(normal_str)
                
                depth_str = f"Contact Depth: {state.depths[0]:.3f}"
                rospy.loginfo(depth_str)
                
                # Write to file
                f.write(f"\nContact {i}:\n")
                f.write(f"Collision between: {collision1} and {collision2}\n")
                f.write(f"{pos_str}\n")
                f.write(f"{normal_str}\n")
                f.write(f"{depth_str}\n")
                f.write("-" * 50 + "\n")
            
            f.write("\n")
            rospy.loginfo("-" * 50)
    
    def update(self):
        """Update method required by Component base class."""
        pass  # All work is done in the ROS callback 
from ...state import VehicleState, ObjectPose, ObjectFrameEnum, ObstacleState, ObstacleMaterialEnum, ObstacleStateEnum
from ..interface.gem import GEMInterface
from ..component import Component
from typing import Dict
import yaml
import rospy
import time
from ...utils import settings


OBSTACLE_TYPE_TO_ENUM = {
    'cone': ObstacleMaterialEnum.TRAFFIC_CONE,
}

OBSTACLE_DIMENSIONS = {
    'cone': (0.5, 0.5, 1.0),
}


class FakeObstacleDetector(Component):
    def __init__(self, vehicle_interface: GEMInterface):
        """
        Args:
            vehicle_interface: GEMInterface instance
        """
        self.vehicle_interface = vehicle_interface
        
        # Load path from the settings, with a default if not present
        self.scene_path = settings.get("run.drive.perception.obstacle_detection.largs.scene_path", None)

        if not self.scene_path:
            raise ValueError("Scene path not specified in the YAML configuration under `largs.scene_path`")
        
        self.t_start = None
        self.obstacles = self.load_obstacles(self.scene_path)

    def load_obstacles(self, scene_path: str) -> Dict[str, ObstacleState]:
        """
        Loads obstacles from a YAML scene file and initializes them as ObstacleState objects.
        """
        print(f"[FakeObstacleDetector] Loading obstacles from: {scene_path}")
        
        with open(scene_path, 'r') as file:
            data = yaml.safe_load(file)

        obstacles = {}
        for name, attributes in data['obstacles'].items():
            pose = ObjectPose(
                frame=ObjectFrameEnum.ABSOLUTE_CARTESIAN,
                t=0.0,  # Time is dynamically assigned during simulation
                x=attributes['position'][0],
                y=attributes['position'][1],
                yaw=attributes.get('yaw', 0)
            )

            # Add velocity if it exists, else default to (0, 0, 0)

            obstacle_state = ObstacleState(
                type=OBSTACLE_TYPE_TO_ENUM[attributes['type']],
                activity=ObstacleStateEnum.STANDING,
                pose=pose,
                dimensions=OBSTACLE_DIMENSIONS[attributes['type']],
                outline=None
            )

            # Add the velocity attribute directly

            obstacles[name] = obstacle_state
        
        return obstacles

    def rate(self):
        return 4.0

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return ['obstacles']

    def update(self, vehicle: VehicleState) -> Dict[str, ObstacleState]:
        """
        Called every simulation step, updates the timestamp and publishes the obstacle states.
        """
        if self.t_start is None:
            self.t_start = self.vehicle_interface.time()
        
        t = self.vehicle_interface.time() - self.t_start
        
        # Update all obstacle poses with the current simulation time
        res = {}
        for name, obstacle in self.obstacles.items():
            obstacle.pose.t = t
            res[name] = obstacle
            rospy.loginfo(f"[FakeObstacleDetector] Simulated Obstacle Detected: {name} at {obstacle.pose.x}, {obstacle.pose.y}")
        
        return res

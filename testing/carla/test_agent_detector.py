import glob
import os
import sys
import logging
from math import tan, radians, pi

import carla
import numpy as np
import pygame
import queue as queue_lib
from PIL import Image, ImageDraw
import random

# Constants
WIDTH, HEIGHT = 1152, 720
TIMEOUT = 10.2
HOST, PORT = 'localhost', 2000
FOV = 90.0
FONT_SIZE = 20
LOG_FILE_PATH = "testing/carla/output/simulation.log"

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE_PATH),
        logging.StreamHandler(sys.stdout),
    ],
)
logger = logging.getLogger(__name__)

# Ensure the CARLA Python API can be found
try:
    carla_api_path = glob.glob(
        f"../carla/dist/carla-*{sys.version_info.major}.{sys.version_info.minor}-{'win-amd64' if os.name == 'nt' else 'linux-x86_64'}.egg")[0]
    sys.path.append(carla_api_path)
except IndexError:
    logger.error("CARLA API not found. Ensure it is available in the specified path.")

sys.path.append(os.getcwd())
sys.path.append(os.path.join(os.path.dirname(__file__), "../GEMstack"))
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from GEMstack.onboard.perception.road_agent_detection import AgentDetector

def initialize_pygame():
    """Initialize Pygame display and font."""
    pygame.init()
    display = pygame.display.set_mode((WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = pygame.font.SysFont("Arial", FONT_SIZE)
    logger.info(f"Pygame initialized with display resolution {WIDTH}x{HEIGHT}")
    return display, font

def setup_carla_client():
    """Setup and connect to the CARLA client."""
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    world = client.get_world()
    logger.info(f"Connected to CARLA server with a timeout of {TIMEOUT} seconds")
    return world

def carla_image_to_rgb_array(carla_image):
    """Convert CARLA image to RGB NumPy array."""
    array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
    array = np.reshape(array, (carla_image.height, carla_image.width, 4))[:, :, :3]
    return array[:, :, ::-1]

def setup_sensors(world, vehicle):
    """Setup camera and LiDAR sensors on the vehicle."""
    blueprint_library = world.get_blueprint_library()
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(WIDTH))
    camera_bp.set_attribute("image_size_y", str(HEIGHT))
    camera_bp.set_attribute("fov", str(FOV))

    camera_location = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera_sensor = world.spawn_actor(camera_bp, camera_location, attach_to=vehicle)

    lidar_bp = blueprint_library.find("sensor.lidar.ray_cast")
    lidar_bp.set_attribute("range", "400")
    lidar_location = carla.Transform(carla.Location(x=0, z=2.5))
    lidar_sensor = world.spawn_actor(lidar_bp, lidar_location, attach_to=vehicle)

    logger.info("Camera and LiDAR sensors set up and attached to the vehicle")
    return camera_sensor, lidar_sensor

def convert_lidar_data_to_numpy(raw_data):
    """Convert raw LiDAR data to NumPy point cloud."""
    dtype = np.float32
    num_floats = len(raw_data) // 4
    num_points = num_floats // 4  # XYZI
    point_cloud = np.frombuffer(raw_data, dtype=dtype).reshape((num_points, 4))[:, :3]
    return point_cloud

def calculate_intrinsic_matrix():
    """Calculate intrinsic camera matrix."""
    calibration = np.identity(3)
    calibration[0, 2] = WIDTH / 2.
    calibration[1, 2] = HEIGHT / 2.
    calibration[0, 0] = calibration[1, 1] = WIDTH / (2.0 * tan(radians(FOV / 2)))
    return calibration

def transformation_matrix(transform):
    """Generate transformation matrix using carla.Transform."""
    rotation = transform.rotation
    location = transform.location
    matrix = np.array([
        [
            np.cos(radians(rotation.pitch)) * np.cos(radians(rotation.yaw)),
            np.cos(radians(rotation.yaw)) * np.sin(radians(rotation.pitch)) * np.sin(radians(rotation.roll)) - np.sin(radians(rotation.yaw)) * np.cos(radians(rotation.roll)),
            -np.cos(radians(rotation.yaw)) * np.sin(radians(rotation.pitch)) * np.cos(radians(rotation.roll)) - np.sin(radians(rotation.yaw)) * np.sin(radians(rotation.roll)),
            location.x
        ],
        [
            np.sin(radians(rotation.yaw)) * np.cos(radians(rotation.pitch)),
            np.sin(radians(rotation.yaw)) * np.sin(radians(rotation.pitch)) * np.sin(radians(rotation.roll)) + np.cos(radians(rotation.yaw)) * np.cos(radians(rotation.roll)),
            -np.sin(radians(rotation.yaw)) * np.sin(radians(rotation.pitch)) * np.cos(radians(rotation.roll)) + np.cos(radians(rotation.yaw)) * np.sin(radians(rotation.roll)),
            location.y
        ],
        [
            np.sin(radians(rotation.pitch)),
            -np.cos(radians(rotation.pitch)) * np.sin(radians(rotation.roll)),
            np.cos(radians(rotation.pitch)) * np.cos(radians(rotation.roll)),
            location.z
        ],
        [0, 0, 0, 1]
    ])
    return matrix

def draw_agent_info(display, font, agent_info, x=10, y=50):
    """Draw agent information on the Pygame display."""
    offset_y = 20
    for idx, (name, state) in enumerate(agent_info.items()):
        text = (
            f"{name}: Location = (x:{state.pose.x:.2f}, y:{state.pose.y:.2f}, z:{state.pose.z:.2f}), "
            f"Velocity = {state.velocity[0]}"
        )

        surface = font.render(text, True, (255, 0, 0))  # dark green
        display.blit(surface, (x, HEIGHT - (y + idx * offset_y)))
        
def draw_bounding_boxes(display, agent_info, font):
    """Draw bounding boxes and agent information based on detected data."""
    for agent_id, agent in agent_info.items():
        # Extracting agent data
        pos = (agent.pose.x, agent.pose.y)
        pos = carla.Transform(carla.Location(x=pos[0], y=pos[1], z=0))
        pos = (pos.location.x, pos.location.y)
        dimensions = agent.dimensions
        if all(dimensions):  # Check if dimensions are non-zero
            rect = pygame.Rect(pos[0], pos[1], dimensions[0], dimensions[1])
            pygame.draw.rect(display, (255, 0, 0), rect, 2)  # Draw the bounding box
        else:
            # Default dimensions if none provided
            rect = pygame.Rect(pos[0], pos[1], 50, 20)  # Placeholder dimensions
            pygame.draw.rect(display, (255, 0, 0), rect, 2)  # Draw the bounding box

        # Displaying agent information
        info_text = f'{agent_id}: Velocity={agent.velocity[0]}'
        text_surface = font.render(info_text, True, (255, 255, 255))
        display.blit(text_surface, (pos[0], pos[1] - 20))  # Display above the bounding box
        
class CarlaSyncMode:
    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get("fps", 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(
            carla.WorldSettings(
                no_rendering_mode=False,
                synchronous_mode=True,
                fixed_delta_seconds=self.delta_seconds,
            )
        )

        def make_queue(register_event):
            q = queue_lib.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data
            
def run_simulation():
    try:
        display, font = initialize_pygame()
        world = setup_carla_client()
        vehicle_bp = world.get_blueprint_library().find("vehicle.audi.tt")
        start_pose = world.get_map().get_spawn_points()[15]
        vehicle = world.spawn_actor(vehicle_bp, start_pose)
        camera_sensor, lidar_sensor = setup_sensors(world, vehicle)
        vehicle.set_autopilot(True)
        agent_detector = AgentDetector(vehicle_bp)
        logger.info("Vehicle spawned at a random location and set to autopilot mode")
        
        intrinsic_matrix = calculate_intrinsic_matrix()
        camera_transform_matrix = transformation_matrix(camera_sensor.get_transform())
        
        with CarlaSyncMode(world, camera_sensor, lidar_sensor, fps=30) as sync_mode:
            frame_count = 0
            while True:
                snapshot, image_rgb, lidar_data = sync_mode.tick(timeout=2.0)
                frame_count += 1
                if frame_count % 5 != 0:
                    continue

                

                lidar_array = convert_lidar_data_to_numpy(lidar_data.raw_data)
                if not lidar_array.size:
                    logger.warning(
                        "No points detected within the specified range for this frame"
                    )
                    continue

                pil_image = carla_image_to_rgb_array(image_rgb)

                agent_detector.image_callback(pil_image)
                print(lidar_array)
                agent_detector.lidar_callback(lidar_array)

                detected_data = agent_detector.update(vehicle)
                print(pil_image.shape)

               

                if detected_data:
                    logger.info(f"Detected : {len(detected_data)} objects")
                    draw_agent_info(display, font, detected_data)
                    draw_bounding_boxes(display, detected_data, font)
                    pygame.image.save(display, f"output/frame_{image_rgb.frame:06d}.png")

                pygame.display.flip()
                surface = pygame.surfarray.make_surface(pil_image.swapaxes(0, 1))
                display.blit(surface, (0, 0))
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        logger.info("Simulation stopped by user")
                        return
    except Exception as e:
        logger.error("Error during simulation: %s", str(e), exc_info=True)
    finally:
        try:
            camera_sensor.destroy()
            lidar_sensor.destroy()
            vehicle.destroy()
            pygame.quit()
            logger.info("Resources successfully released and simulation ended")
        except Exception as e:
            logger.error("Error during resource cleanup: %s", str(e))
            
if __name__ == "__main__":
    try:
        run_simulation()
    finally:
        logger.info("Simulation ended.")


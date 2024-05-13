import glob
import os
import sys
import logging
from math import radians
from scipy.optimize import fsolve
import numpy as np
import pygame
import queue
from PIL import Image, ImageDraw
import carla
import random

# Configuration constants
WIDTH, HEIGHT = 800, 600
TIMEOUT = 10.0
SERVER_HOST = "localhost"
SERVER_PORT = 2000
FPS = 30
PROCESS_EVERY_NTH_FRAME = 5
FONT_SIZE = 15
LOG_FILE_PATH = "output/sign_detection_simulation.log"
CARLA_DIST_PATTERN = "../carla/dist/carla-*{major}.{minor}-{arch}.egg"
LIDAR_RANGE = 30
IMAGE_SIZE_X = "800"
IMAGE_SIZE_Y = "600"
CAMERA_X = 1.5
CAMERA_Z = 2.4
LIDAR_Z = 2.5
X_RANGE = (0, 10)  # xmin, xmax for polynomial intersection
Y_RANGE = (0, 10)  # ymin, ymax for polynomial intersection
Z_RANGE = (0, 10)  # zmin, zmax for polynomial intersection

sys.path.append(os.getcwd())

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
logger.info("Simulation started...")
logger.info("Logging to file: sign_detection_simulation.log")


# Ensure the CARLA Python API can be found
try:
    carla_api_path = glob.glob(
        f"../carla/dist/carla-*{sys.version_info.major}.{sys.version_info.minor}-{'win-amd64' if os.name == 'nt' else 'linux-x86_64'}.egg")[0]
    sys.path.append(carla_api_path)
except IndexError:
    logger.error("CARLA API not found. Ensure it is available in the specified path.")


sys.path.append(os.path.join(os.path.dirname(__file__), "../GEMstack"))

from GEMstack.onboard.perception.lane_detection import LaneDetector

def initialize_display():
    pygame.init()
    display = pygame.display.set_mode((WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = pygame.font.SysFont("Arial", FONT_SIZE)
    logger.info(f"Display initialized: {WIDTH}x{HEIGHT}")
    return display, font

def connect_to_carla_server():
    client = carla.Client(SERVER_HOST, SERVER_PORT)
    client.set_timeout(TIMEOUT)
    world = client.get_world()
    logger.info(f"Connected to CARLA server at {SERVER_HOST}:{SERVER_PORT} with timeout {TIMEOUT}")
    return world

def convert_image(carla_image):
    array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
    return np.reshape(array, (carla_image.height, carla_image.width, 4))[:, :, :3][:, :, ::-1]

def setup_vehicle_sensors(world, vehicle):
    blueprint_library = world.get_blueprint_library()
    camera_blueprint = blueprint_library.find("sensor.camera.rgb")
    camera_blueprint.set_attribute("image_size_x", IMAGE_SIZE_X)
    camera_blueprint.set_attribute("image_size_y", IMAGE_SIZE_Y)
    camera_transform = carla.Transform(carla.Location(x=CAMERA_X, z=CAMERA_Z))
    camera_sensor = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle)

    lidar_blueprint = blueprint_library.find("sensor.lidar.ray_cast")
    lidar_blueprint.set_attribute("range", str(LIDAR_RANGE))
    lidar_transform = carla.Transform(carla.Location(z=LIDAR_Z))
    lidar_sensor = world.spawn_actor(lidar_blueprint, lidar_transform, attach_to=vehicle)

    logger.info("Vehicle sensors initialized.")
    return camera_sensor, lidar_sensor

def convert_lidar_data_to_numpy(raw_data):
    """Convert raw LiDAR data to NumPy point cloud."""
    dtype = np.float32
    num_floats = len(raw_data) // 4
    num_points = num_floats // 4  # XYZI
    point_cloud = np.frombuffer(raw_data, dtype=dtype).reshape((num_points, 4))[:, :3]
    return point_cloud

def filter_points_by_range(points):
    conditions = (
        (X_RANGE[0] < points[:, 0]) & (points[:, 0] < X_RANGE[1]) &
        (Y_RANGE[0] < points[:, 1]) & (points[:, 1] < Y_RANGE[1]) &
        (Z_RANGE[0] < points[:, 2]) & (points[:, 2] < Z_RANGE[1])
    )
    return points[conditions]

def find_intersections(poly1, poly2, x_range):
    def equations(x):
        return poly1(x) - poly2(x)
    return fsolve(equations, x_range)

def draw_lane_lines(image, right_lane_poly, left_lane_poly, num_points=500):
    image = Image.fromarray(image)
    draw = ImageDraw.Draw(image)
    x_values = np.linspace(0, image.width - 1, num_points)
    right_y_values = right_lane_poly(x_values)
    left_y_values = left_lane_poly(x_values)
    intersections = find_intersections(right_lane_poly, left_lane_poly, x_values)
    valid_intersections = [
        (x, right_lane_poly(x))
        for x in intersections
        if 0 <= x < image.width
        and 0 <= right_lane_poly(x) < image.height
        and 0 <= left_lane_poly(x) < image.height
    ]
    right_lane_points = [
        (x, y)
        for x, y in zip(x_values, right_y_values)
        if image.height // 2 <= y < image.height
    ]
    draw.line(right_lane_points, fill="blue", width=5)
    left_lane_points = [
        (x, y)
        for x, y in zip(x_values, left_y_values)
        if image.height // 2 <= y < image.height
    ]
    draw.line(left_lane_points, fill="red", width=5)
    return np.array(image)

def process_lidar_data(raw_data):
    lidar_array = convert_lidar_data_to_numpy(raw_data)
    lidar_array = filter_points_by_range(lidar_array)
    return lidar_array


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
            q = queue.Queue()
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
            

class run_simulation():
    def __init__(self):
        self.display, self.font = initialize_display()
        self.world = connect_to_carla_server()
        self.vehicle_bp = self.world.get_blueprint_library().find("vehicle.audi.tt")
        self.vehicle = self.world.spawn_actor(self.vehicle_bp, random.choice(self.world.get_map().get_spawn_points()))
        self.camera_sensor, self.lidar_sensor = setup_vehicle_sensors(self.world, self.vehicle)
        self.vehicle.set_autopilot(True)
        self.lane_detector = LaneDetector(self.vehicle)
        self.frame_count = 0

    def run(self):
        with CarlaSyncMode(self.world, self.camera_sensor, self.lidar_sensor, fps=FPS) as sync_mode:
            while True:
                snapshot, image_rgb, lidar_data = sync_mode.tick(timeout=2.0)
                self.frame_count += 1
                if self.frame_count % PROCESS_EVERY_NTH_FRAME != 0:
                    continue
                lidar_array = process_lidar_data(lidar_data.raw_data)
                lidar_array = filter_points_by_range(lidar_array)
                if not lidar_array.size:
                    logger.warning("No points detected within the specified range for this frame")
                    continue
                pil_image = convert_image(image_rgb)
                self.lane_detector.image_callback(pil_image)
                self.lane_detector.lidar_callback(lidar_array)
                right_lane_poly, left_lane_poly = self.lane_detector.detect_lane()
                if right_lane_poly is None or left_lane_poly is None:
                    logger.warning("Lane detection failed for this frame")
                    image_rgb = convert_image(image_rgb)
                else:
                    image_rgb = draw_lane_lines(
                        pil_image, right_lane_poly, left_lane_poly, num_points=400
                    )
                surface = pygame.surfarray.make_surface(image_rgb.swapaxes(0, 1))
                self.display.blit(surface, (0, 0))
                pygame.display.flip()
                pygame.image.save(self.display, f"output/frame_{1}.png")
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        logger.info("Simulation stopped by user")
                        return

    def __del__(self):
        self.camera_sensor.destroy()
        self.lidar_sensor.destroy()
        self.vehicle.destroy()
        pygame.quit()
        logger.info("Resources successfully released and simulation ended")
        

if __name__ == "__main__":
    try:
        sim = run_simulation()
        sim.run()
    finally:
        logger.info("Simulation ended.")
        os.rename("sign_detection_simulation.log", "output/sign_detection_simulation.log")
        logging.shutdown()
        sys.exit(0)
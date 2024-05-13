
import glob
import os
import sys
import carla
import random
import numpy as np
import cv2
import pygame
import queue
from PIL import Image
from matplotlib import pyplot as plt
import logging


# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("sign_detection_simulation.log"),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)


# Set the paths to required modules
sys.path.append(os.getcwd())
sys.path.append(os.path.join(os.path.dirname(__file__), '../GEMstack'))

# Ensure that the CARLA Python API can be found by Python interpreter
try:
    carla_api = glob.glob(
        '../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'
        )
    )[0]
    sys.path.append(carla_api)
except IndexError:
    logger.error('CARLA API not found. Ensure it is available in the specified path.')

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from GEMstack.onboard.perception.sign_detection import SignDetector


def initialize_pygame():
    """Initialize Pygame for visualization purposes."""
    pygame.init()
    display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = pygame.font.SysFont('Arial', 15)
    logger.info('Pygame initialized with display resolution 800x600')
    return display, font


def setup_carla_client(timeout=10.0):
    """Setup CARLA client and return the connected world object."""
    client = carla.Client('localhost', 2000)
    client.set_timeout(timeout)
    world = client.get_world()
    logger.info('Connected to CARLA server with a timeout of %s seconds', timeout)
    return world


def carla_image_to_array(carla_image):
    """Converts a CARLA image to an RGB NumPy array suitable for Pygame."""
    array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
    array = np.reshape(array, (carla_image.height, carla_image.width, 4))
    array = array[:, :, :3]  # Convert from BGRA to RGB
    return array[:, :, ::-1]  # Reverse channels


def setup_sensors(world, vehicle):
    """Setup and return the CARLA camera and LiDAR sensors."""
    blueprint_library = world.get_blueprint_library()

    cam_bp = blueprint_library.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '800')
    cam_bp.set_attribute('image_size_y', '600')
    cam_location = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera_sensor = world.spawn_actor(cam_bp, cam_location, attach_to=vehicle)

    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range', '30')
    lidar_location = carla.Transform(carla.Location(x=0, z=2.5))
    lidar_sensor = world.spawn_actor(lidar_bp, lidar_location, attach_to=vehicle)

    logger.info('Sensors (Camera and LiDAR) set up and attached to the vehicle')
    return camera_sensor, lidar_sensor


def bytes_to_numpy_lidar(raw_data):
    """Convert raw LiDAR data to a NumPy point cloud."""
    point_size = 4  # XYZI
    xyz_size = 3  # XYZ only
    dtype = np.float32

    num_floats = len(raw_data) // 4
    num_points = num_floats // point_size
    trimmed_raw_data = raw_data[:num_points * xyz_size * 4]

    point_cloud = np.frombuffer(trimmed_raw_data, dtype=dtype).reshape((num_points, xyz_size))
    return point_cloud


def filter_lidar_by_range(point_cloud):
    """Filter a LiDAR point cloud to include only points within a specific 3D bounding box."""
    xmin, xmax = 0, 20
    ymin, ymax = 0, 20
    zmin, zmax = 0, 20

    idxs = np.where(
        (point_cloud[:, 0] > xmin) & (point_cloud[:, 0] < xmax) &
        (point_cloud[:, 1] > ymin) & (point_cloud[:, 1] < ymax) &
        (point_cloud[:, 2] > zmin) & (point_cloud[:, 2] < zmax)
    )
    return point_cloud[idxs]


def run_simulation():
    """Run the CARLA simulation with camera and LiDAR sensors for sign detection."""
    try:
        display, _ = initialize_pygame()
        world = setup_carla_client()
        vehicle_bp = world.get_blueprint_library().find('vehicle.audi.tt')

        start_pose = random.choice(world.get_map().get_spawn_points())
        second_start_pose = random.choice(world.get_map().get_spawn_points())

        vehicle = world.spawn_actor(vehicle_bp, start_pose)
        vehicle_2 = world.spawn_actor(vehicle_bp, second_start_pose)

        camera_sensor, lidar_sensor = setup_sensors(world, vehicle)

        vehicle_2.set_autopilot(True)
        vehicle.set_autopilot(True)
        sign_detector = SignDetector(vehicle_bp)

        # change the traffic light state to every 5
        
        logger.info('Two vehicles spawned at random locations and set to autopilot mode')

        with CarlaSyncMode(world, camera_sensor, lidar_sensor, fps=30) as sync_mode:
            frame_count = 0
            while True:
                snapshot, image_rgb, lidar_data = sync_mode.tick(timeout=2.0)

                frame_count += 1
                if frame_count % 5 != 0:  # Process every 5th frame
                    continue

                lidar_array = bytes_to_numpy_lidar(lidar_data.raw_data)
                lidar_array = filter_lidar_by_range(lidar_array)

                if not lidar_array.size:
                    logger.warning('No points detected within the specified range for this frame')
                    continue

                pil_image = carla_image_to_array(image_rgb)

                sign_detector.image_callback(pil_image)
                sign_detector.lidar_callback(lidar_array)
              
                detected_signs = sign_detector.update(vehicle, None)
                logger.info("Number of detected signs: %d", len(detected_signs))
                
                detected_signs, names = sign_detector.detect_signs()
                logger.info("Number of detected signs: %d", len(detected_signs))
                logger.info("Detected signs: %s", names)
                image_array = carla_image_to_array(image_rgb)
                surface = pygame.surfarray.make_surface(image_array.swapaxes(0, 1))
                display.blit(surface, (0, 0))
                pygame.display.flip()

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        logger.info('Simulation stopped by user')
                        return
    except Exception as e:
        logger.error("Error during simulation: %s", str(e), exc_info=True)
    finally:
        try:
            camera_sensor.destroy()
            lidar_sensor.destroy()
            vehicle.destroy()
            vehicle_2.destroy()
            pygame.quit()
            logger.info('Resources successfully released and simulation ended')
        except Exception as e:
            logger.error('Error during resource cleanup: %s', str(e), exc_info=True)


class CarlaSyncMode:
    """Context manager for CARLA simulation synchronization."""

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds
        ))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        """Advance simulation to the next frame, and retrieve sensor data."""
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        """Restore the world's settings when leaving the context."""
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data


if __name__ == '__main__':
    try:
        run_simulation()
    finally:
        logger.info('Simulation ended.')

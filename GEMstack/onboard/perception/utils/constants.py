import yaml

# Ignore certain yaml tag
def ignore_relative_path(loader, node):
    return loader.construct_scalar(node)
yaml.SafeLoader.add_constructor('!relative_path', ignore_relative_path)

# Load vehicle geometry
vehicle_geometry = None
with open('./GEMstack/knowledge/vehicle/gem_e4_geometry.yaml', 'r') as f:
    vehicle_geometry = yaml.safe_load(f)

# Vehicle geometry constants
GEM_E4_LENGTH = 3.2
GEM_E4_WIDTH = 1.7
if vehicle_geometry:
    GEM_E4_LENGTH = vehicle_geometry['length']
    GEM_E4_WIDTH = vehicle_geometry['width']

# Parking constants
GROUND_THRESHOLD = -0.15
NUM_CONES_PER_PARKING_SPOT = 4
VEHICLE_FRAME = "vehicle"

# Visualization constants
VEHICLE_FRAME_ORIGIN = [0.0, 0.0, 0.0, 0.0]
VEHICLE_MARKER_DIM = [0.8, 0.5, 0.3]
VEHICLE_MARKER_COLOR = (0.0, 0.0, 1.0, 1)
OBSTACLE_MARKER_COLOR = (1.0, 0.0, 0.0, 0.4)
LIDAR_PC_COLOR = (255, 0, 0)
CONE_CENTER_PC_COLOR = (255, 0, 255)
MAX_POLYGON_MARKERS = 1
MAX_PARKING_SPOT_MARKERS = 1
MAX_OBSTACLE_MARKERS = 5
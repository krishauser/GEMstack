# Entity Detection in GEMstack

This module contains implementations for detecting entities (pedestrians, vehicles, obstacles, etc.) in the environment. The available detectors are:

## Agent Detection

### OmniscientAgentDetector

The `OmniscientAgentDetector` works with the basic simulator to obtain agent states that are being simulated. It receives updates from the simulator and maintains a thread-safe dictionary of all detected agents.

Example usage:

```python
from onboard.perception.agent_detection import OmniscientAgentDetector
from onboard.interface.gem_simulator import GEMDoubleIntegratorSimulationInterface

# Create the simulator interface
simulator = GEMDoubleIntegratorSimulationInterface()

# Create the agent detector with the simulator interface
agent_detector = OmniscientAgentDetector(simulator)

# Initialize and start
agent_detector.initialize()
simulator.start()

# Later, when you need agent data:
agent_states = agent_detector.update()
```

### GazeboAgentDetector

The `GazeboAgentDetector` works specifically with the Gazebo simulator, subscribing to the model states topic and converting Gazebo models into `AgentState` objects. It can be configured to track specific model prefixes.

Example usage:

```python
from onboard.perception.agent_detection import GazeboAgentDetector
from onboard.interface.gem_gazebo import GEMGazeboInterface

# Create the Gazebo interface
gazebo_interface = GEMGazeboInterface()

# Define the model prefixes you want to track as agents
tracked_prefixes = ['pedestrian', 'person', 'car', 'vehicle']

# Create the agent detector with the Gazebo interface and the tracked prefixes
agent_detector = GazeboAgentDetector(gazebo_interface, tracked_prefixes)

# Initialize and start
agent_detector.initialize()
gazebo_interface.start()

# Later, when you need agent data:
agent_states = agent_detector.update()
```

### Agent State Format

Agent detectors provide `AgentState` objects with the following properties:

- `pose`: The position and orientation of the agent
- `dimensions`: The physical dimensions (length, width, height) of the agent
- `type`: The type of agent (car, pedestrian, bicyclist, etc.)
- `activity`: The activity state of the agent (stopped, moving, fast)
- `velocity`: The velocity vector in the agent's local frame
- `yaw_rate`: The rate of rotation around the vertical axis 

## Obstacle Detection

### GazeboObstacleDetector

The `GazeboObstacleDetector` works with the Gazebo simulator to detect obstacles such as traffic cones. It subscribes to the model states topic and converts Gazebo models into `ObstacleState` objects based on specific model prefixes.

Example usage:

```python
from onboard.perception.obstacle_detection import GazeboObstacleDetector
from onboard.interface.gem_gazebo import GEMGazeboInterface

# Create the Gazebo interface
gazebo_interface = GEMGazeboInterface()

# Define the model prefixes you want to track as obstacles
tracked_obstacle_prefixes = ['cone']

# Create the obstacle detector with the Gazebo interface
obstacle_detector = GazeboObstacleDetector(gazebo_interface, tracked_obstacle_prefixes)

# Initialize and start
obstacle_detector.initialize()
gazebo_interface.start()

# Later, when you need obstacle data:
obstacle_states = obstacle_detector.update()
```

### Obstacle State Format

Obstacle detectors provide `ObstacleState` objects with the following properties:

- `pose`: The position and orientation of the obstacle
- `dimensions`: The physical dimensions of the obstacle
- `type`: The material type of obstacle (traffic cone, barrier, etc.)
- `activity`: The state of the obstacle (standing, left, right)

For traffic cones in particular, the orientation is analyzed to determine if the cone is:
- `STANDING`: Upright within normal thresholds
- `LEFT`: Tipped to the left side
- `RIGHT`: Tipped to the right side

## Configuration

Both detectors can be configured through settings in the configuration file:

```yaml
simulator:
  agent_tracker:
    model_prefixes: 
      - pedestrian
      - person
      - bicycle
      - bike
      - car
      - vehicle
      - truck
    rate: 10.0  # Hz - how frequently to process model updates
  obstacle_tracker:
    model_prefixes:
      - cone
    rate: 10.0  # Hz - how frequently to process model updates
```

The model prefixes are strings that match the beginning of Gazebo model names. For example, a model named "pedestrian1" would match the "pedestrian" prefix, while "cone5" would match the "cone" prefix.

## Implementation Details

Both agent and obstacle detection are implemented in the `GEMGazeboInterface` class, which:

1. Monitors the Gazebo model states
2. Matches model prefixes to determine entity type
3. Transforms coordinates from Gazebo's global frame to the vehicle's START frame
4. Determines agent activity based on velocity or obstacle state based on orientation
5. Creates and maintains appropriate state objects
6. Provides detection through registered callbacks

Both GEM e2 and GEM e4 vehicle models are supported with proper coordinate transformations.
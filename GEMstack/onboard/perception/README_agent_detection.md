# Agent Detection in GEMstack

This module contains implementations for detecting agents (pedestrians, vehicles, etc.) in the environment. The available detectors are:

## OmniscientAgentDetector

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

## GazeboAgentDetector

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

## Configuration

The Gazebo agent detector can be configured through settings in the configuration file:

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
```

The model prefixes are strings that match the beginning of Gazebo model names. For example, a model named "pedestrian1" would match the "pedestrian" prefix.

## Agent State Format

Both detectors provide `AgentState` objects with the following properties:

- `pose`: The position and orientation of the agent
- `dimensions`: The physical dimensions (length, width, height) of the agent
- `type`: The type of agent (car, pedestrian, bicyclist, etc.)
- `activity`: The activity state of the agent (stopped, moving, fast)
- `velocity`: The velocity vector in the agent's local frame
- `yaw_rate`: The rate of rotation around the vertical axis 
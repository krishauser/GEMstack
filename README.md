# GEMstack: software structure for CS588 Autonomous Vehicle System Engineering

## Dependencies

Python 3.7+

PACMOD - low level Autonomoustuff's interface to vehicle

ROS (Noetic?) - messaging with cameras, simulator

Python dependencies:
- opencv-python
- numpy
- scipy
- torch
- shapely
- dacite
- pyyaml

## Package structure 

All packages are within the `GEMstack/` folder.  

`mathutils/`: Math utilities common to onboard / offboard use.
  - `transforms`: 2d and 3d rotations and rigid transforms.
  - `filters`: 1d signal processing.
  - `cameras`: Contains standard camera models.
  - `differences`: Finite differences for derivative approximation.
  - `dynamics`: Contains standard dynamics models.
  - `dubins`: Contains first- and second-order Dubins car dynamics models.
  - `control`: Contains standard control techniques, e.g., PID controller.
  - `collisions`: Provides collision detection and proximity detection.
  
`utils/`: Other utilities common to onboard / offboard use.
  - `logging`: Provides logging and log replay functionality.
  - `simulation`: Interfaces with the Gazebo (possibly other?) simulators.
  - `visualization`: Tools for converting internal data on knowledge, state, etc. to visualization apps.
  - `settings`: Tools for managing settings for onboard behaviour.  If you're tempted to write a magic parameter or global variable, it should be placed here instead.
  - `config`: Tools for loading config files.
  - `serialization`: Tools for serializing / deserializing objects.

`state/`: Representations of state of the vehicle and its environment, including internal state that persists from step to step.
  - `physical_object`: A generic physical object base class.
  - `trajectory`: Stores a generic path or trajectory.
  - `vehicle`: Ego-vehicle state.
  - `intent`: Ego-vehicle intent that may involve special logic or signaling behavior, e.g., lane change, take exit, shutting down.
  - `roadgraph`: A section of the roadmap around the ego-vehicle.
  - `roadmap`: A map created for offline use.
  - `environment`: Environmental conditions, e.g., weather, road conditions.
  - `obstacle`: A static obstacle or debris.
  - `sign`: A traffic sign.
  - `agent`: Another moving object, e.g., pedestrian, bicyclist, vehicle.
  - `scene`: All physical items that may be relevant to the current scene, i.e., vehicle, roadgraph, environment, obstacles, and agent states.
  - `agent_intent`: Maintains an estimate of agent intent.
  - `entity_relation`: Maintains an estimate of a relationship between entities, e.g. VISIBLE, FOLLOWING, PASSING, YIELDING.
  - `mission`: Stores the current mission objective, e.g., IDLE, DRIVE_ROUTE, ESTOP, used by routing, logic, planning, and execution.
  - `predicates`: Any items predicates that are estimated to be true in the current world.
  - `route`: Stores a 2d route, coming from the router.
  - `all`: State or the current scene, all intent and relation estimates, and the driving logic (objective, predicates, route).

`offboard/`: Creation and management of data and knowledge.
  - `calibration/`: Sensor calibration.
  - `log_management/`: Provides log management, browsing, and query functionality.
  - `detection_learning/`: Detection model learning.
  - `prediction_learning/`: Prediction model learning.
  - `heuristic_learning/`: Driving heuristic learning.

`knowledge/`: Models and parameters common to onboard / offboard use.  The file "current.py" in each directory will store the current model being used.
  - `vehicle/`: Vehicle geometry and physics.
  - `calibration/`: Calibrated sensor parameters.
  - `detection/`: Stores detection models.
  - `prediction/`: Stores prediction models.
  - `heuristics/`: Stores heuristic models.
  - `roadmaps/`: Stores roadmap knowledge, e.g., lanes, regions, obstacles, signs.
  - `routes/`: Stores precomputed routes.
  - `predicates/`: Stores named predicates that may be true in a world state.
  - `defaults/`: Stores the default settings.

`launch/`: Launch scripts are listed here.  Specify which configuration you want to use as an argument to `main.py`.

`onboard/`: All algorithms governing onboard behavior are located here.  These algorithms may make use of items in the `knowledge/` stack.
	`perception/`: Perception components.
	  - `state_estimation`: State estimators.
	  - `roadgraph_update`: Roadgraph updaters.
	  - `lane_detection`: Lane detection.
	  - `sign_detection`: Sign detection.
	  - `obstacle_detection`: Obstacle detction.
	  - `agent_detection`: Agent detection.
	  - `environment_detection`: Environment condition detection.
	  - `intent_estimation`: Agent intent estimation.
	  - `relation_estimation`: Entity relation estimation.
	  - `agent_prediction`: Agent motion prediction.

	`planning/`: Planning components.
	  - `route_planner`: Decides which route to drive from the roadgraph.
	  - `driving_logic`: Performs all necessary logic to develop a planning problem specification, e.g., select obstacles, design cost functions, etc.
	  - `heuristics`: Implements various planning heuristics.
	  - `motion_planning`: Implements one or more motion planners.
	  - `optimization`: Implements one or more trajectory optimizers.
	  - `selection`: Implements best-trajectory selection.
	  - `pure_pursuit`: Implements a pure pursuit controller.
	  - `recovery`: Implements recovery behavior.

	`execution/`: Executes the onboard driving behavior.
	  - `entrypoint`: The entrypoint that launches all onboard behavior.  Configured by settings in 'run'
	  - `executor`: Base classes for executors.
	  - `log_replay`: A generic component that replays from a log.

	`interface/`: Defines interfaces to vehicle hardware / simulators / external signals
	  - `gem.py`: Base class for the Polaris GEM e2 vehicle.
	  - `gem_hardware.py`: Interface to the real GEM vehicle.
	  - `gem_simulator.py`: Interfaces to simulated GEM vehicles.
	  - `teleop`: Teleoperator control signals.


## Launching the stack

You will launch a simulation using:

- `python main.py GEMstack/launch/LAUNCH_FILE.yaml` where `LAUNCH_FILE.yaml` is your preferred simulation launch file.  Inspect the simulator classes in `GEMstack/onboard/interface/gem_simulator/` for more information about configuring the simulator.

To launch onboard behavior you will open four terminal windows, and in each of them run:
- `roscore`
- `roslaunch basic_launch sensor_init.launch`
- `roslaunch basic_launch visualization.launch`
- `python main.py GEMstack/launch/LAUNCH_FILE.yaml` where `LAUNCH_FILE.yaml` is your preferred launch file. 


Note that if you try to use `import GEMstack` in a script or Jupyter notebook anywhere outside of this directory, Python will not know where the `GEMstack` module is.  If you wish to import `GEMstack` from a script located in a separate directory, you can put

```python
import sys
import os
sys.path.append(os.getcwd())   #or enter the absolute path of this directory

import GEMstack
```

at the top of your script.  Then, you can run the script from this directory via `python PATH/TO/SCRIPT/myscript.py`.  See the scripts in `testing` for an example of how this is done.

You can also install `GEMstack` into the system Python by calling `pip install .`, but this is not recommended because has a couple of drawbacks:
- You might make changes in this directory, e.g., via `git pull`, and then forget to reinstall, so the changes won't be reflected when you run your code.
- If you added model or roadgraph files, e.g., to the `knowledge` directory, they may not be installed.  You will need to edit `pyproject.toml` to include those files.



## Communication protocols

Sending commands to the vehicle is handled by the ROS-PACMOD interface.  Receiving sensor messages is handled through standard ROS sensor messages.

Generally speaking, the only onboard components that should be reading from sensors are the Perception components.  The only onboard components that should be sending commands to the vehicle are the Execution comopnents.

For internal state messages, which changes rapidly during development, we use raw Python dictionaries, lists, and primitives. This is also known as JSON format.  The `utils.serialization` library makes this easy for you.  We convert strings to and from Python classes that are annotated with the `@dataclass` decorator and you can add your own classes using the `@utils.serialization.register` decorator.  You can then use the `utils.serialization.serialize` and `utils.serialization.deserialize` functions to convert to/from strings or ROS `std_msgs/String` messages. 

Note that all registered class names must be unique.  Also, **versioning** is a major problem if you wish to use legacy logs.  If you gather some logs, change your class' attributes, and then attempt to view those logs again, you may encounter an error or missing data.  The `serialization` module will do as much as it can to fail silently and enter `None` into missing fields, but it can still fail.  If you wish to parse logs that contain legacy data, you can use the `version` keyword to `register`, as follows.

```python
from utils.serialization import register
from dataclasses import dataclass

@dataclass
@register(name="MyClass",version="1")
class MyClass_Original:
	x : float
	y : float

@dataclass
@register(name="MyClass",version="2")
class MyClass:
	x : float
	y : float
	time : float

```

Keep in mind that your functions will need to distinguish between the old and new classes.  It may be better in this case just to use a single class and tag `time` as having type `Optional[float]`.  Then, your functions can see whether `time` is `None`, and if so, invoke the old-style behavior.


## Settings

Magic parameters and global variables are a scourge and must be eliminated in production code.  Instead, you will declare parameters in configuration files.  In your code, you will access settings using the `utils.settings` module.  For example, 

```python
from GEMutils.utils import settings
settings.get('key1.key2.attribute')
```

The entrypoint will consume a run launch file and a settings file.

## Branches and submitting pull requests

To count as a contribution to the team, you will need to check in your code via pull requests (PRs).  PRs should be reviewed by at least one other approver.

- `main`: will contain content that persists between years.  Approver: Kris Hauser.
- `s2024`: is the "official class vehicle" for this semester's class.  Approver: instructor, TAs.
- `s2024_teamX`: will be your team's branch. Approver: instructor, TAs, team members.  

Guidelines:
- DO NOT check in large datasets.  Instead, keep these around on SSDs.
- DO check in trained models, named descriptively.  In your PR, describe how you evaluated the model and its results.  Choose which model you use in your tests in the settings.

## Homework assignments

HW1 (out 1/17, in 1/24): Distress signals
- Skills: Familiarization with ROS and GEMstack structure, Git
- Receive low-level messages from the sensors via ROS and print them
- Send low-level messages to flash distress to the vehicle via ROS
- Use Git to create a fork, create a behavior with your distress signal.  Push contributions, and run your fork on GEM.

HW2 (out 1/24, in 2/7): Stop for a pedestrian.
- Skills: Object detection, trajectory tracking, logic-based motion planning, safety driver training
- Use provided object detector to identify pedestrian from front camera. Run on the vehicle.
- Use a motion planner to create a trajectory that slows and stops at a safe distance when pedestrian appears.
- Modify the planner to resume slowly when the pedestrian disappears
- Run on the vehicle

HW3 (out 2/7, in 2/21): Pedestrian tracking
- Sensor calibration, scene state, coordinate conversions
- Perform sensor calibration and update current calibration in the knowledge directory
- Use pedestrian detector on multiple cameras to place agents in the scene
- Visualize the sensor data and the estimated agents in `rviz` using `visualization_msgs/MarkerArray`.
- Record the agent trajectories in the START coordinate frame into a log.
- Plot agent trajectories from the log as curves in matplotlib.

After HW3, the class will split into teams and work on different parts of the stack.  Your grades will be determined by your team presentations, individual contributions, within-team peer reviews, and between-team peer reviews.

Checkpoint 1 (out 2/21, in 3/6)
- Produce design document and present at design review on 3/6.
- Design document should: establish a list of goals.  Describe plan for implementation.  Assign personnel.  Show timeline to implementation and evaluation.
- Peer reviews due next day.

Checkpoint 2 (out 3/6, in 3/27)
- Progress report presentation on 3/27
- Presentation should describe implementation progress, unit testing results (concisely describe metrics used), changes in direction, demonstrations, review of code contributions.  Discuss integration plans.
- Peer reviews due next day.

Integration checkpoint 3 (out 3/27, in 4/10)
- Progress report presentation on 4/10
- Presentation should describe integration results (i.e., pull reviews, metrics), changes in direction, demonstrations, review of code contributions. 
- Peer reviews due next day.

Checkpoint 4 (out 4/10, in 4/24)
- Revise design document to mark achieved checkpoints, integration metrics, and changes of plans.  Present at design review on 4/24.

Final presentation (5/8)
- Pitch contribution of team to "investor" / "CEO".  Describe final integration results.
- Reflect on how design has evolved.  Review code contributions.


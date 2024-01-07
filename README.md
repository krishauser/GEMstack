# GEMstack: software for CS588 Autonomous Vehicle System Engineering

📖 [Online documentation](https://gemstack.readthedocs.org)

🚗 [About the GEM e2 vehicle](https://publish.illinois.edu/robotics-autonomy-resources/gem/)

🗎 [ROS code for launching vehicle](https://github.com/hangcui1201/POLARIS_GEM_e2_Real/tree/main)

## Dependencies

GEMstack uses Python 3.7+ and ROS Noetic.  (It is possible to do some offline and simulation work without ROS, but it is highly recommended to install it if you are working on any onboard behavior.)  

You should also have the following Python dependencies installed, which you can install from this folder using `pip install -r requirements.txt`:

- numpy
- scipy
- matplotlib
- opencv-python
- torch
- shapely
- dacite
- pyyaml


In order to interface with the actual GEM e2 vehicle, you will need [PACMOD2](https://github.com/astuff/pacmod2) - Autonomoustuff's low level interface to vehicle. You will also need Autonomoustuff's [sensor message packages](https://github.com/astuff/astuff_sensor_msgs).  The onboard computer uses Ubuntu 20.04 with Python 3.8, CUDA 11.6, and NVIDIA driver 515, so to minimize compatibility issues you should ensure that these are installed on your development system.

From a fresh Ubuntu 20.04 with ROS Noetic and [CUDA 11.6 installed](https://gist.github.com/ksopyla/bf74e8ce2683460d8de6e0dc389fc7f5), you can install these dependencies using:

```console
sudo apt update
sudo apt-get install git python3 python3-pip wget zstd
source /opt/ros/noetic/setup.bash

#install Zed SDK
wget https://download.stereolabs.com/zedsdk/4.0/cu121/ubuntu20 -O ZED_SDK_Ubuntu20_cuda11.8_v4.0.8.zstd.run
chmod +x ZED_SDK_Ubuntu20_cuda11.8_v4.0.8.zstd.run
./ZED_SDK_Ubuntu20_cuda11.8_v4.0.8.zstd.run -- silent

#create ROS Catkin workspace
mkdir catkin_ws
mkdir catkin_ws/src

#install ROS dependencies and packages
cd catkin_ws/src
git clone https://github.com/hangcui1201/POLARIS_GEM_e2_Real.git
git clone https://github.com/astuff/pacmod2.git
git clone https://github.com/astuff/astuff_sensor_msgs.git
git clone https://github.com/ros-perception/radar_msgs.git
cd radar_msgs; git checkout noetic; cd ..

cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

#install GEMstack Python dependencies
cd ..
git clone https://github.com/krishauser/GEMstack.git
cd GEMstack
python3 -m pip install -r requirements.txt
```

To build a Docker container with all these prerequisites, you can use the provided Dockerfile by running `docker build -t gem_stack .`. 


## In this folder

Your work will be typically confined to the `GEMstack/` folder, and you may use the `testing/`, `logs/`, `data/`, and `scenes/` folders.

- `GEMstack/`: the software package (see [below](#package-structure)).
- `main.py`: the standard entry point to running onboard behavior (see [below](#launching-the-stack)).
- `logs/`: logs will be placed here.  These will not be committed to the Github repo.
- `data/`: standard location to place datasets for training, i.e., downloaded or curated from other sources.  These will not be committed to the Github repo.
- `scenes/`: standard location to place scenes for simulation.
- `testing/`: test scripts to check whether GEMstack components are functioning.
- `README.md`: this file.
- `LICENSE`: MIT license.
- `.gitignore`: Git ignore file. All files that match these patterns will not be added to Git.
- `docs/`: ReadTheDocs documentation source files are placed here. Used by automated tools to build the [online documentation](https://gemstack.readthedocs.org).
- `.readthedocs.yaml`: ReadTheDocs configuration file.  
- `pyproject.toml`: Describes the GEMstack Python package for pip install. 
- `requirements.txt`: A list of Python dependencies for the software stack, used via `pip install -r requirements.txt`.

In addition, some tools (e.g., pip) will build temporary folders, such as `build` and `GEMstack.egg-info`. You can ignore these.

## Package structure 

All packages are within the `GEMstack/` folder.  

Legend:
- 🟥: TODO
- 🟧: early development (not usable)
- 🟨: in development (usable, but many features not complete or tested)
- 🟩: stable (most features complete and tested)
- 🟦: mature

`mathutils/`: 🧮 Math utilities common to onboard / offboard use.
  - 🟩 `transforms`: 2d and 3d rotations and rigid transforms.
  - 🟩 `filters`: 1d signal processing.
  - 🟥 `cameras`: Contains standard camera models.
  - 🟦 `differences`: Finite differences for derivative approximation.
  - 🟦 `dynamics`: Contains standard dynamics models.
  - 🟦 `dubins`: Contains first- and second-order Dubins car dynamics models.
  - 🟩 `control`: Contains standard control techniques, e.g., PID controller.
  - 🟨 `collisions`: Provides collision detection and proximity detection.
  
`utils/`: 🛠️ Other utilities common to onboard / offboard use.
  - 🟩 `logging`: Provides logging and log replay functionality.
  - 🟨 `mpl_visualization`: Tools for plotting data on knowledge, state, etc. in Matplotlib.
  - 🟥 `gazebo_visualization`: Tools for converting data on knowledge, state, etc. to ROS messages used in Gazebo.
  - 🟦 `settings`: Tools for managing settings for onboard behaviour.  If you're tempted to write a magic parameter or global variable, it should be [placed in settings instead](#settings).
  - 🟦 `config`: Tools for loading config files. 
  - 🟦 `serialization`: Tools for serializing / deserializing objects.
  - 🟦 `loops`: Tools for writing timed loops.

`state/`: 💾 Representations of state of the vehicle and its environment, including internal state that persists from step to step.
  - 🟩 `physical_object`: A generic physical object base class.
  - 🟩 `trajectory`: Stores a generic path or trajectory. 
  - 🟩 `vehicle`: Ego-vehicle state. 
  - 🟨 `intent`: Ego-vehicle intent that may involve special logic or signaling behavior, e.g., lane change, take exit, shutting down. 
  - 🟨 `roadgraph`: A section of the roadmap around the ego-vehicle. 
  - 🟨 `roadmap`: A map created for offline use. 
  - 🟨 `environment`: Environmental conditions, e.g., weather, road conditions. 
  - 🟨 `obstacle`: A static obstacle or debris. 
  - 🟨 `sign`: A traffic sign. 
  - 🟨 `agent`: Another moving object, e.g., pedestrian, bicyclist, vehicle. 
  - 🟩 `scene`: All physical items that may be relevant to the current scene, i.e., vehicle, roadgraph, environment, obstacles, and agent states. 
  - 🟨 `agent_intent`: Maintains an estimate of agent intent. 
  - 🟨 `entity_relation`: Maintains an estimate of a relationship between entities, e.g. VISIBLE, FOLLOWING, PASSING, YIELDING. 
  - 🟨 `mission`: Stores the current mission objective, e.g., IDLE, DRIVE_ROUTE, ESTOP, used by routing, logic, planning, and execution. 
  - 🟩 `predicates`: Any items predicates that are estimated to be true in the current world. 
  - 🟩 `route`: Stores a 2d route, coming from the router. 
  - 🟩 `all`: State or the current scene, all intent and relation estimates, and the driving logic (objective, predicates, route). 

`offboard/`: 💻 Programs for creation and management of data and knowledge.
  - 🟥 `calibration/`: Sensor calibration.
  - 🟥 `log_management/`: Provides log management, browsing, and query functionality. 
  - 🟥 `detection_learning/`: Detection model learning. 
  - 🟥 `prediction_learning/`: Prediction model learning. 
  - 🟥 `heuristic_learning/`: Driving heuristic learning. 

`knowledge/`: 🧠 Models and parameters common to onboard / offboard use.  The file "current.py" in each directory will store the current model being used.
  - 🟨 `vehicle/`: Vehicle geometry and physics. (needs testing)
  - 🟨 `calibration/`: Calibrated sensor parameters.
  - 🟥 `detection/`: Stores detection models.
  - 🟥 `prediction/`: Stores prediction models.
  - 🟥 `heuristics/`: Stores heuristic models.
  - 🟥 `roadmaps/`: Stores roadmap knowledge, e.g., lanes, regions, obstacles, signs.
  - 🟨 `routes/`: Stores precomputed routes. 
  - 🟥 `predicates/`: Stores named predicates that may be true in a world state.
  - 🟩 `defaults/`: Stores the default settings. 

`launch/`: 🚀 Launch scripts are listed here.  Specify which configuration you want to use as an argument to `main.py`.

`onboard/`: 🚗 All algorithms governing onboard behavior are located here.  These algorithms may make use of items in the `knowledge/` stack.
  - `perception/`: Perception components.
    - 🟨 `state_estimation`: State estimators.
    - 🟨 `roadgraph_update`: Roadgraph updaters. 
    - 🟨 `perception_normalization`: Normalizes the scene before planning.  
    - 🟥 `lane_detection`: Lane detection.
    - 🟥 `sign_detection`: Sign detection. 
    - 🟥 `obstacle_detection`: Obstacle detction. 
    - 🟥 `agent_detection`: Agent detection. 
    - 🟥 `environment_detection`: Environment condition detection. 
    - 🟥 `intent_estimation`: Agent intent estimation. 
    - 🟥 `relation_estimation`: Entity relation estimation. 
    - 🟥 `agent_prediction`: Agent motion prediction. 

  - `planning/`: Planning components.
    - 🟨 `route_planner`: Decides which route to drive from the roadgraph. 
    - 🟥 `driving_logic`: Performs all necessary logic to develop a planning problem specification, e.g., select obstacles, design cost functions, etc. 
    - 🟥 `heuristics`: Implements various planning heuristics. 
    - 🟥 `motion_planning`: Implements one or more motion planners. 
    - 🟥 `optimization`: Implements one or more trajectory optimizers.  
    - 🟥 `selection`: Implements best-trajectory selection.
    - 🟨 `pure_pursuit`: Implements a pure pursuit controller.
    - 🟨 `recovery`: Implements recovery behavior.

  - `execution/`: Executes the onboard driving behavior.
    - 🟩 `entrypoint`: The entrypoint that launches all onboard behavior.  Configured by settings in 'run'.
    - 🟩 `executor`: Base classes for executors.
    - 🟩 `log_replay`: A generic component that replays from a log.
    - 🟨 `multiprocess_execution`: Component executors that work in separate process.  (Stdout logging not done yet. Still hangs on exception.)
  
  - `visualization/`: Visualization components on-board the vehicle
    - 🟨 `mpl_visualization`: Matplotlib visualization

  - `interface/`: Defines interfaces to vehicle hardware and simulators.
    - 🟩 `gem`: Base class for the Polaris GEM e2 vehicle.
    - 🟨 `gem_hardware`: Interface to the real GEM vehicle.
    - 🟨 `gem_simulator`: Interfaces to simulated GEM vehicles.


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



## Communication and serialization protocols

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

To override a setting temporarily (just for a few run), you can run your script with an optional `--key=value` command-line argument.  For example, to set the simulation scene, you can use `--simulator.scene=PATH/TO/SCENE/FILE`.  

To create new settings or override a setting more permanently, you should dive into `GEMstack/knowledge/defaults/current.yaml`.  This [YAML](https://yaml.org/) formatted configuration file specifies the entire configuration that can be accessed through the `utils.settings` module.  One of these files may `!include` other configuration files, so if you are adding a large number of related settings, e.g., for some component module, it would make sense to create that module's own YAML file.  For example, you may create a YAML file `mymodule_default_config.yaml` add it to `current.yaml` under the `mymodule` key, e.g., `mymodule: !include mymodule_default_config.yaml`.  (Of course, replace `mymodule` with a descriptive name of your module, duh.)

Note that there are settings that configure **an algorithm's behavior** that persist between runs, and there are settings that configure **a particular run**.  If you want to configure an algorithm, put it in `current.yaml`, a descendant configuration file, or elsewhere in `knowledge`.  If you want to configure a single run, you should place those options in the launch file.  The `main.py` entrypoint will consume a run launch file and a settings file, and will place all the run configurations in the `run` attribute of the global settings.  So if you wish to inspect run details or specify per-run behavior, e.g., see whether we are in a simulation run or a hardware run, your algorithm can check `settings.get('run.mode')`.  In general, you should try to minimize how dependent your algorithms are on run settings.

Another way to think about this is that we are trying to **evolve the onboard software stack to generate better behavior** by changing algorithms and their settings. The evolution mechanism is implemented by commits to the repository.  On a day to day level, you will be performing different types of runs, such as simulation tests, unit tests, and full integration tests.  You may be testing a lot of different conditions but the software stack should remain constant for that suite of tests.  If you wish to do an apples-to-apples comparison against a different version of the stack, you should git check out another commit ID, and then perform those same tests.  So if you are configuring the software stack, the setting changes should go into `knowledge`.  If you are configuring how the software stack works just for a single test, the setting changes should go into the launch script or a keyword argument.


## Launch files, pipeline state machine, and the computation graph

Onboard behavior begins by launching an executor, which maintains a *pipeline state machine* that can switch between different top-level behaviors.  Pipelines are usually switched depending on the health state of the system, and are not appropriate for handling driving logic.  For example, the `recovery` pipeline is a mandatory fallback pipeline in case an essential component fails on the vehicle.  For most cases, `drive` and `recovery` are sufficient.  

Each pipeline defines a *computation graph* consisting of `Component` subclasses (see `GEMstack.onboard.component`), such as state estimators, object detectors, routing, planners, etc. Each component operates in a loop on attributes of the `AllState` object (see `GEMstack.state.allstate`).  Each component defines a *rate* at which its loop should be executed, a set of *state inputs* (part or all of the `AllState`), a set of *state outputs*, and *initialize*, *update*, and *cleanup* callbacks.  The basic idea is that all components in the computation graph will be run in a loop as follows:

```python
state = [SHARED_STATE]
component = MyComponent()
component.initialize()
for every 1/component.rate() seconds, and while still active:
    inputs = [state.X for X in component.state_inputs()]
    outputs = component.update(*inputs)
    for Y,outY in zip(component.state_outputs(),outputs)
        state.Y = outY
component.cleanup()
```

The computation graph defines an execution order of components and a set of allowable inputs and outputs for each component. This structure is defined in the `run.computation_graph` setting and by default uses `GEMstack/knowledge/defaults/computation_graph.yaml`.

You should think of `AllState` as a strictly typed blackboard architecture in which items can be read from and written to.  If you need to pass data between components, you should add it to the state rather than use alternative techniques, e.g., global variables.  This will allow the logging / replay to save and restore system state.  Over a long development period, it would be best to be disciplined at versioning.

It is generally assumed that components will not maintain significant internal state.  If you implement a component that does update internal state, then the executor will not be able to reproduce prior behavior from logs. This causes headaches with replay tools and A/B testing.

If you wish to override the executor to add more pipelines, you will need to create a new executor by subclassing from `ExecutorBase`.  This will need to implement the pipeline switching and termination logic as detailed in the `begin`, `update`, `done`, and `end` callbacks.


## Branches and submitting pull requests

To count as a contribution to the team, you will need to check in your code via pull requests (PRs).  PRs should be reviewed by at least one other approver.

- `main`: will contain content that persists between years.  Approver: Kris Hauser.
- `s2024`: is the "official class vehicle" for this semester's class.  Approver: instructor, TAs.
- `s2024_groupX`: will be your group's branch. Approver: instructor, TAs, team members.  

Guidelines:
- DO NOT check in large datasets.  Instead, keep these around on SSDs.
- DO check in trained models, named descriptively.  In your PR, describe how you evaluated the model and its results.  Choose which model you use in your tests in the settings.


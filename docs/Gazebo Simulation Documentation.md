# Gazebo Simulation Documentation

## Available Sensors in Gazebo

The following sensors are currently available in the Gazebo simulation environment:

- **Front Camera**
- **GNSS**
- **Lidar**

---

## Gazebo Simulation Setup

To run your code within the Gazebo simulator, you'll first need to set up and install necessary dependencies.

### 1. Set Up the Simulator

Go to this [POLARIS GEM Simulator](https://github.com/harishkumarbalaji/POLARIS_GEM_Simulator/tree/main) repository to set up the Gazebo Docker container and environment for simulation.

Follow the instructions in the linked repo to build and run the Docker container. It also provides a simulation environment of highbay.

### 2. Install Dependencies

Install the required ROS packages:

```bash
sudo apt-get install -y ros-noetic-ackermann-msgs ros-noetic-gazebo-msgs
```

---

## Configuring Your Code for Simulation

To run your code with the Gazebo simulation, modify your launch file according to the following steps:

1. **Add a new variant:**
   Under the `variants` section, create a new variant called `"gazebo"`.

2. **Set the vehicle interface:**
   In the `"gazebo"` variant, set the `vehicle_interface` to use the Gazebo interface:
   ```yaml
   vehicle_interface:
     type: gem_gazebo.GEMGazeboInterface
   ```

3. **Add modifications (optional):**
   You can add any other configuration or parameters needed for your testing under the `"gazebo"` variant.

Setting the `vehicle_interface` to `gem_gazebo.GEMGazeboInterface` will automatically link the **GEMStack** sensor topics with the corresponding **Gazebo vehicle sensors**, allowing you to test your code in the simulation environment.

## Example Launch File

For a full example, refer to the [`launch/fixed_route.yaml`](launch/fixed_route.yaml) file, which includes a sample `gazebo` variant configuration.

---

## Running Gazebo

Follow these steps to run your GEMStack code with Gazebo:

### 1. Launch the Gazebo Simulator

In one terminal, run the Gazebo simulator (using the instructions provided in the [POLARIS GEM Simulator](https://github.com/harishkumarbalaji/POLARIS_GEM_Simulator/tree/main) repository). This will load the simulation environment with the vehicle and sensors.

### 2. Launch the GEM Stack

Open a second terminal and launch GEMStack with your configured launch file. Make sure to set the variant to `gazebo`.

#### For GEM e2 Vehicle:

```bash
python3 main.py --variant=gazebo --settings=GEMstack/knowledge/defaults/e2.yaml launch/fixed_route.yaml
```

#### For GEM e4 Vehicle:

```bash
python3 main.py --variant=gazebo --settings=GEMstack/knowledge/defaults/current.yaml launch/fixed_route.yaml
```

You can replace `fixed_route.yaml` with your specific launch file.

**Note:** By default, the system uses `GEMstack/knowledge/defaults/current.yaml` which contains GEM e4 vehicle configuration settings.

## Available Variants and Vehicle Types

**Variants:**
- `sim` - Simple simulation mode
- `gazebo` - 3D Gazebo simulation mode

**Vehicle Types:**
- `e2` - GEM e2 vehicle (uses Novatel GNSS)
- `e4` - GEM e4 vehicle (uses Septentrio GNSS)

## Available Configuration Files

GEMstack/knowledge/defaults/
- `current.yaml` - Default configuration (GEM e4)
- `e2.yaml` - GEM e2 configuration

## Agent Detection in Gazebo

The Gazebo simulation environment supports detection of various types of agents (pedestrians, vehicles, etc.) that can be spawned in the simulation world. You can configure GEMstack to detect and track these agents.

### Configuring Agent Detection in Launch File

To enable agent detection in your Gazebo simulation, add the following to your launch file's `gazebo` variant:

```yaml
drive:
    perception:
        state_estimation: GNSSStateEstimator  # Matches your Gazebo GNSS implementation
        agent_detection:
            type: agent_detection.GazeboAgentDetector
            args:
                tracked_model_prefixes: ['pedestrian', 'car', 'bicycle']
```

The `tracked_model_prefixes` parameter specifies which types of models will be tracked as agents. The prefixes listed will match against the model names in Gazebo. For example, a model named `pedestrian1` will be tracked if `pedestrian` is in the list.

### Spawning Agents in Gazebo

You can spawn agents in Gazebo using a YAML configuration file.

Follow the instructions in the [POLARIS GEM Simulator](https://github.com/harishkumarbalaji/POLARIS_GEM_Simulator/tree/main) repository to spawn agents in Gazebo.

### Important Naming Considerations

The `name` field in your agent configuration is crucial as it determines how models are identified in Gazebo's `model_states` topic. For agent detection to work:

1. The model name prefix must match one of the prefixes in your `tracked_model_prefixes` configuration
2. For example, a model named `pedestrian1` will be detected as a pedestrian if `pedestrian` is in your tracked prefixes
3. The model type is derived from the prefix, so naming conventions are important

When spawning models in Gazebo, ensure that their names have appropriate prefixes that match your GEMstack configuration.

### How Agent Detection Works

The `GazeboAgentDetector` component:
1. Subscribes to Gazebo's `model_states` topic to get positions of all models
2. Filters models based on the configured `tracked_model_prefixes`
3. Transforms model positions from Gazebo's coordinate frame to GEMstack's START frame
4. Creates `AgentState` objects for each detected agent
5. Makes these agents available to other GEMstack components for planning and visualization

The models in your simulation will appear in your GEMstack visualization and be available for planning and decision-making components to interact with.

---

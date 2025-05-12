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
sudo apt-get install -y ros-noetic-ackermann-msgs
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

---

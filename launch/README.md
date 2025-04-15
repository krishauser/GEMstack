Contains launch scripts for use in the standard executor, e.g., `python main.py launch/fixed_route_sim.yaml`.


# Gazebo Simulation Guide lines

## Gazebo Simulation Setup

Head over [here](https://github.com/harishkumarbalaji/POLARIS_GEM_Simulator/tree/main) to setup gazebo docker container to run the simulation

Install dependency packages

```
sudo apt-get install -y ros-noetic-ackermann-msgs

```

    


## Running GEM Stack with Gazebo

- Open two terminals
- Launch the Gazebo simulator in the first terminal
- In the other terminal navigate to GEM Stack folder and run the following command to launch GEM Stack

```
Sample command:

python3 main.py --variant=sim --vehicle=e4_gazebo launch/gazebo_simulation.yaml


Other variants:

python3 main.py --variant={variant_name} --vehicle={vehicle_name} launch/gazebo_simulation.yaml

Variants:
sim

Vehicle types:

e2
e4
e2_gazebo
e4_gazebo

```

- The gazebo_simulation.yaml can be editied to add your planning and perception code, more variants can be added on request.
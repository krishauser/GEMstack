# CARLA Sign, Agent, and Lane Detection Simulation

This Python script evaluate **Sign_Detector, Agent_Detector, Lane_Detector Components** using camera and LiDAR sensors in the CARLA simulator.



## Usage

Start carla simulator with the following command:

```{bash}
bash CarlaUE4.sh -prefernvidia -RenderOffScreen -quality-level=Low
```

Navigate the main directory and run the following command to start the simulation:

For  sign detector simulation:

```{bash}
python testing/carla/test_sign_detector.py
```

For agent detector simulation:

```{bash}
python testing/carla/test_agent_detector.py
```

For lane detector simulation:

```{bash}
python testing/carla/test_lane_detector.py
```


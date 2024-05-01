# Hybrid A* Planner

## Parameters
Stored in `GEMstack/knowledge/defaults/A_star_planner.yaml`
```yaml
    rate: 1.0 # Rate to run A*
    dt: 0.8 # Timestep
    velocity: 1.0 # Velocity of the car at each timestep
    max_wheel_angle: 0.6108
    N_sample_controls: 5 # Explored paths for each timestep
    resolution: 0.5 # Grid resolution
    angle_resolution: 0.25 # Angle resolution
    target_threshold: 1.0 # Terminate condition threshold
    RS_threshold: 12.0 # Below the threshold, will try Reeds-shepp path to reach the goal
    RS_resolution: 0.2 # Resolution of the generated Reeds-shepp path
    RS_prob: 0.1 # Probability to generate Reeds-shepp path
    gear_cost: 10.0 # Extra cost for changing gear
    backward_cost_scale: 1.5 # Scale the cost for going backward
    smooth_threshold: 0.5 # Path smoothing threshold
    precomputed: !relative_path ../heuristics/reeds_shepp.npy # Stores the shortest kinomatically feasible path to a set of three dimensional goal states from a start position(in this case 0,0,0)
```


## Parking Team / Curbside Pickup
Input component template: `GEMstack/onboard/perception/parking_slot_detection.py`




### How to launch
To launch in simulation, use this command:
```
python3 main.py --variant=sim launch/parking.yaml 
```




# MPC Planner

## Parameters
Stored in `GEMstack/knowledge/defaults/MPC_planner.yaml`
```yaml
  delta_min: -0.785 # wheel angle lower bound
  delta_max: 0.785 # wheel angle upper bound
  a_min: -0.5 # acceleration lower bound
  a_max: 0.5 # acceleration upper bound
  v_min: -1 # velocity lower bound
  v_max: 1 # velocity upper bound
  penalty:
    obstacle_penalty: 800 # penalty scale of approaching obstacles
    lane_bond_penalty: 2000 # penalty scale of crossing lane bound
    ellipse_a_squared: 2 # control the x direction in the ellipse obstacle penalty
    ellipse_b_squared: 0.25 # control the y direction in the ellipse obstacle penalty
  weight:
    w_g: 3 # objective weight of reaching goal
    w_o: 1 # objective weight of avoiding obstacles
    w_l: 4 # objective weight of avoiding crossing the lanes
  N: 10 # length of MPC prediction window
  dt: 0.5 # timestep
  safe_dist: 5.5 # distance threshold before braking
  rate: 10 # rate
```
## Normal Road Driving
Input component template: `GEMstack/onboard/perception/normal_road_detection.py`
### How to launch
To launch in simulation, use this command:
```
python3 main.py --variant=sim launch/road_driving.yaml 
```





# Changes to `GEMstack/state/all.py`
```python
# For Parking
parking_slot : Optional[ObjectPose] = None

# For Normal Road Driving
lane_goal: Optional[ObjectPose] = None
lane_bound: Optional[List[float]] = None
```
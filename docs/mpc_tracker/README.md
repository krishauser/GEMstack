# MPC Tracker

This MPC works out-of-box with any input routes, which we give me details in the following sections.

## Parameters
The following can be used in place of the default pure pursuit tracker:
```yaml
type: mpc_tracking.MPCTrajectoryTracker
args: {N: 5, dt: 1.0, Q: [1, 1, 50], R: [1, 1, 10, 10], fixed: True} 
```
Here we explain more on each parameter:

- $N$: the number of timesteps for the horizon in MPC
- $dt$: the time descretization in seconds
- $Q$: $(q_1, q_2, q_3)$ is the tracking error penalization parameter for the positional (x and y), heading, and velocity errors
- $R$: $(r_1, r_2, r_3, r_4)$ is the control cost penalization parameter where $r_1$ and $r_2$ penalize large control inputs and $r_3$ and $r_4$ penalize large deviations in consecutive controls (i.e., to make the path more smooth)
- fixed: an optional parameter, by setting this to true we will only compute the headings of the input trajectory once, thus saving some computation time

The parameter values shown above are already fine-tuned and we have found them to have decent performance on various routes that we have tested but please feel free to experiment with different parameter values.

In addition, a desired speed should also be specified in route to trajectory planner, and we have found 1.0 to perform decently well.
```yaml
motion_planning:
    type: RouteToTrajectoryPlanner
    args: [1.0]  #desired speed in m/s.  If null, this will keep the route untimed for the trajectory tracker
```

## Simulation
To launch a simple trajectory following example, run the following command
```
python3 main.py --variant=sim launch/fixed_route.yaml
```

## Evaluation
One can evaluate the performance of this MPC tracker, as well as any other controllers using this stand-alone script in ```utils/controller_evaluator.py```. It takes as inputs the actual and a reference trajectory and should be fairly straightforward to use.


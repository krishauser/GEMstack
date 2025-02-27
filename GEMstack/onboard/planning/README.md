# Planning-A HW1 Document

This document serves as an overview for integrating pedestrian yielder into longitudinal planner. It describes how parameters from `pedetrian_detecion.yaml` are used in the planning system along with references to the `longitudinal_planning.py` script and the `pedestrian_yield_logic.py` script.

Details on the algorithm and demos are availavle at the link below:<br>
https://drive.google.com/drive/folders/1aSOWIrqXjf9-j6i1S_MJksATMyyoP31_?usp=sharing


## Contribution
Each member contributed to:
### longitudinal_planning.py Contributions

| Algorithm  | Contributor     |
| :--------- | :-------------- |
| milestone  | Simon (sk106)   |
| dt         | Rohit (srm17)   |
| dx         | Hansen (hl58)   |

### pedestrian_yield_logic.py Contributions

| Algorithm  | Contributor                                 |
| :--------- | :------------------------------------------ |
| expert     | Patrick (bohaowu2), Animesh (animesh8)      |
| analytic   | Henry (weigang2)                            |
| simulation | Yudai (yyamada2)                            |


## Configuration Details
The `pedetrian_detecion.yaml` file includes:
```
args:
    mode: 'real'
    params: {
        'yielder':       'expart', # 'expart', 'analytic', or 'simulation'
        'planner':       'milestone', # 'milestone', 'dt', or 'dx'
        'desired_speed': 1.0,  # m/s
        'acceleration':  0.75  # m/s2
        }
```


## Usage
### pedetrian_detecion.yaml
Algorithm can be chosen from the above in this file. Also contains parameters for the logic.
### Execution: Simulator
`python3 main.py --variant=fake_sim launch/pedestrian_detection.yaml`
### Testing
- `testing/test_longitudinal_plan.py`
- `testing/test_yield_logic_analytic.ipynb`
- `testing/test_collision_detection.py`

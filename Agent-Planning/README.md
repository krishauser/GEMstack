# Agent-Driver-Planning
This repo provides a simplified version of [Agent-Driver](https://usc-gvl.github.io/Agent-Driver/). The original repo depends on Nuscene. The code is decoupled from the dataset and only supports inference.

## Installation
a. Clone this repository.

b. Install the dependent libraries as follows:

```
pip install -r requirements.txt 
```

c. Obtain the memory data from [Agent-Driver](https://usc-gvl.github.io/Agent-Driver/). The structure should be
```
Agent-Driver
├── data
│   ├── memory
|   |   |── database.pkl
├── agentdriver
├── ...
```

## Training

a. Follow the instructions in [Agent-Driver](https://usc-gvl.github.io/Agent-Driver/).

b. Put the API-key, oganization key and  `FINETUNE_PLANNER_NAME` in `agentdriver/llm_core/api_keys.py`, and 

```
OPENAI_ORG = "org-**"
OPENAI_API_KEY = "sk-**"
FINETUNE_PLANNER_NAME = "ft:gpt-3.5-turbo-0613:personal:***"
```

## Inference
a. Modify the input in `agentdriver/unit_test/test_lanuage_agent.py`. The format should follow:
```
working_memory = {'ego_data': {'ego_states': np.array([ 1.15924776e-01,  7.02820969e+00, 0, 0,
    1.64867798e-03,  4.08400011e+00,  1.85000002e+00,  7.03296375e+00,
-1.45668909e-01]), 'ego_hist_traj_diff': np.array([[0.1215029 , 3.2819965 ],
[0.08331873, 3.3391652 ],
[0.08207586, 3.4891284 ],
[0.02454591, 3.4980984 ]]), 'ego_hist_traj': np.array([[-3.11443388e-01, -1.36083889e+01],
[-1.89940497e-01, -1.03263922e+01],
[-1.06621765e-01, -6.98722696e+00],
[-2.45459061e-02, -3.49809837e+00],
[ 6.25466408e-14,  2.34422297e-14]]), 'goal': np.array([0., 0., 1.])}, 'ego_prompts': None}

# ego_states: [vx, vy, 0, 0, v_yaw (rad/s), ego_length, ego_width, v0 (vy from canbus), Kappa (steering)]
# goal: [right, left, forward]
```
```
perception_dict_sample = {
    "objects": [
       {"type": "car", "id": 1, "waypoints": "[(6.97, -24.46), (6.04, -23.50), (5.07, -22.19), (4.29, -20.46), (3.45, -18.45), (2.92, -15.82)]"},
        {"type": "pedestrian", "id": 2, "waypoints": "[(7.19, 16.00), (7.22, 16.71), (7.25, 17.46), (7.25, 18.18), (7.26, 18.93), (7.23, 19.63)]"},
        {"type": "car", "id": 3, "waypoints": "[(-3.70, 47.92), (-4.77, 51.01), (-5.86, 53.97), (-7.08, 56.77), (-8.36, 59.23), (-9.98, 61.47)]"}
    ],
    "distance_left": "1.0",
    "distance_right": "4.0"
}
```
a. `python -m agentdriver.unit_test.test_lanuage_agent`.

## Citation 

```
@article{agentdriver,
  title={A Language Agent for Autonomous Driving},
  author={Mao, Jiageng and Ye, Junjie and Qian, Yuxi and Pavone, Marco and Wang, Yue},
  year={2023}
}
```

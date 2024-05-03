import sys
from pathlib import Path
sys.path.append("../..")
from agentdriver.utils.ego_state_format import ego_state_to_ego_sample
from agentdriver.main.decoupled_language_agent import LanguageAgent
from agentdriver.llm_core.api_keys import OPENAI_ORG, OPENAI_API_KEY, FINETUNE_PLANNER_NAME
import numpy as np
import openai
openai.organization = OPENAI_ORG
openai.api_key = OPENAI_API_KEY

# path to memory
data_path = Path('/scratch/bbsg/zli138/Agent-Driver/data')
language_agent = LanguageAgent(data_path, planner_model_name=FINETUNE_PLANNER_NAME, verbose=True)
working_memory = {'ego_data': {'ego_states': np.array([ 0,  0,  0, 0,
    0,  4,  1.85,  7.03296375e+00,
0]), 'ego_hist_traj_diff': np.array([[0 , 0 ],
[0, 0],
[0, 0 ],
[0, 0]]), 'ego_hist_traj': np.array([[0, 0],
[0, 0],
[0, 0],
[0, 0],
[0, 0]]), 'goal': np.array([0., 0., 1.])}, 'ego_prompts': None}
perception_dict_sample = {
    "objects": [],
    "distance_left": "4.0",
    "distance_right": "4.0"
}

ego_dict_sample = ego_state_to_ego_sample(working_memory['ego_data'])


language_agent.inference_single(ego_state_dict = ego_dict_sample, perception_dict = perception_dict_sample,
working_memory=working_memory)
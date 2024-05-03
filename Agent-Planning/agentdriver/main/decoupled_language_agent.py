# Main language agent class
# Written by Jiageng Mao
import os
import json
from pathlib import Path
from tqdm import tqdm
import pickle

from agentdriver.memory.memory_agent import MemoryAgent
from agentdriver.reasoning.reasoning_agent import ReasoningAgent
from agentdriver.planning.planning_agent import PlanningAgent
from agentdriver.llm_core.templates import *
import numpy as np
class LanguageAgent:
    # the language agent detached from nuscene dataset
    def __init__(self, data_path, model_name = "gpt-3.5-turbo-0613", planner_model_name="", verbose=False) -> None:
        self.data_path = data_path
        self.verbose = verbose
        self.model_name = model_name
        self.planner_model_name = planner_model_name

    def inference_single(self, ego_state_dict, perception_dict, working_memory):
        r"""Inference single scenario"""
        ego_prompts = get_ego_prompts(ego_state_dict)
        working_memory['ego_prompts'] = ego_prompts
        perception_prompts = get_perception_prompts(perception_dict) 
        # perception_prompts: see template
        # need all ego states and goal here
        # Working Memory Template
        # https://github.com/openai/openai-cookbook/blob/main/examples/How_to_handle_rate_limits.ipynb
        memory_agent = MemoryAgent(data_path=self.data_path, model_name=self.model_name, verbose=self.verbose)
        
        commonsense_mem, experience_mem = memory_agent.run(working_memory)
        # commonsense_mem: see template
        # experience_mem: see template
        reasoning_agent = ReasoningAgent(model_name=self.model_name, verbose=self.verbose)
        reasoning = reasoning_agent.run(None, ego_prompts+perception_prompts, None)
        # the last argument must be false
        # reasoning: see template
        planning_agent = PlanningAgent(model_name=self.planner_model_name, verbose=self.verbose)
        
        # planning_target = planning_agent.generate_planning_target(perception_agent.data_dict)
        # print('planning target')
        # print(planning_target)# we don't need it
        data_sample = {
            "ego": ego_prompts,
            "perception": perception_prompts,
            "commonsense": commonsense_mem,
            "experiences": experience_mem,
            "chain_of_thoughts": reasoning,
            "reasoning": reasoning,
        }
        # 
        planning_traj = planning_agent.run_noreflect(
            data_sample=data_sample,
        )
        print('planning traj')
        print(planning_traj)
        if self.verbose:
            print(ego_prompts)
            print(perception_prompts)
            print(commonsense_mem)
            print(experience_mem)
            print(reasoning)
            print(planning_traj)
        return planning_traj
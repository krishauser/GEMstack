import pickle
from pathlib import Path
import warnings

from agentdriver.llm_core.timeout import timeout
from agentdriver.planning.motion_planning import (
    planning_single_inference_notoken,
)

class PlanningAgent:
    def __init__(self, model_name="", verbose=True) -> None:
        self.verbose = verbose
        self.model_name = model_name # Note: this model must be a **finetuned** GPT model
        if model_name == "" or model_name[:2] != "ft":
            warnings.warn(f"Input motion planning model might not be correct, \
                  expect a fintuned model like ft:gpt-3.5-turbo-0613:your_org::your_model_id, \
                  but get {self.model_name}", UserWarning)
    
    @timeout(15)
    def run_noreflect(self, data_sample):
        """Generate motion planning results for a single scene"""
        # without occupancy
        planning_traj = planning_single_inference_notoken(
            planner_model_id=self.model_name, 
            data_sample = data_sample, 
            data_dict=None, 
            verbose=self.verbose,
            self_reflection = False
        )
        return planning_traj
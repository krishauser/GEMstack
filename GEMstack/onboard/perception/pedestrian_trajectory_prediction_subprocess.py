from ...state import AllState,VehicleState,ObjectPose,ObjectFrameEnum,AgentState,AgentEnum,AgentActivityEnum
from ...utils import settings
from ...mathutils import transforms
from ..interface.gem import GEMInterface
from ..component import Component
from ultralytics import YOLO
import numpy as np
from typing import Dict,Tuple, List
import time
from numpy.linalg import inv
import subprocess
import sys, copy
import time
import torch
import pandas as pd
import numpy as np
from collections import defaultdict

# path = "GEMstack/onboard/prediction/agentformer"
sys.path.append('GEMstack/onboard/perception/agentformer')
from utils.torch import *
from utils.config import Config
from model.model_lib import model_dict
from utils.utils import prepare_seed, print_log, mkdir_if_missing
from dataloader_improved import data_generator

# 8 frames used to base future trajectories off of (current frame plus previous 7)
NUM_PREV_FRAMES = 7
NUM_FUTURE_FRAMES = 12
MODEL_SCRIPT_PATH = 'GEMstack/onboard/perception/agentformer/model/agentformer_process.py'
INPUT_FILE_PATH = 'GEMstack/onboard/perception/agentformer/model_input.txt'

CONFIG_FILE = 'GEMstack/onboard/perception/agentformer/model_cfg/inference.yml'

PEDESTRIAN_DIMS = (1, 1, 1.7)

class PedestrianTrajPrediction(Component):
    """Detects and tracks pedestrians."""
    def __init__(self,vehicle_interface : GEMInterface):
        print("initializing trajpredict CONSTRUCTOR")
        # Inferencing somehow takes 1 extra second on CPU
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.config = Config(CONFIG_FILE)
        self.frame_rate = 2.5

        self.model_process = self.start_model_process('')

    def start_model_process(self, model_path):
        # python_path = '/root/anaconda3/envs/AgentFormer/bin/python'
        return subprocess.Popen(
            ['python', model_path], #["conda", "run", "-n", env_path, "python", model_path],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            encoding="utf-8",
        )

    # Do NOT use until we get our model workinggnikrow
    # dict of pedestrian types
    # each is {agentenum.pedestrian/whatever: {framenum: {ped id:state}}}
    # TODO ANANYA MAKE SURE THIS ALWAYS RETURNS A NP ARRAY
    def convert_data_to_model_input(self, past_agent_states : Dict[AgentEnum, Dict[int, Dict[int, AgentState]]]) -> np.ndarray:
        # get tracked frames for pedestrian agents
        pedestrian_agent_states = past_agent_states[AgentEnum.PEDESTRIAN]
        if len(pedestrian_agent_states) == 0:
            return []


        # get the 8 most recent frames(highest frame number) from the past_agent_states
        past_agent_states = []


        print(len(past_agent_states), len(pedestrian_agent_states))

        # sort pedestrian agent states by frame number
        for frame in sorted(pedestrian_agent_states.keys(), reverse=True)[:NUM_PREV_FRAMES+1]:
            past_agent_states.append(pedestrian_agent_states[frame])
        # reverse past_agent_states so that the most recent frame is first
        past_agent_states = past_agent_states[::-1]
        # past_agent_states = [{ped_id: state}] * # of frames 


        # takes in list of dictionaries corresponding to past 8 frames
        past_frames = []
        recent_pids = {}

        # Dummy frames should be written for these pids
        valid_pids = set()
        for framenum in range(len(past_agent_states)):
            frame = past_agent_states[framenum]
            for ped_id, agent_state in frame.items():
                # create length 17 aray of -1
                row = np.full(17, -1.0)
                # get x, y
                x, y = agent_state.pose.x, agent_state.pose.y
                row[0] = framenum
                row[1] = ped_id
                
                pid_seen_count = recent_pids.get(ped_id, 0) + 1
                if pid_seen_count >= 8:
                    valid_pids.add(ped_id)
                        
                recent_pids[ped_id] = pid_seen_count 
                # we need to flip the x and y coordinates for the model
                row[-4] = y
                row[-2] = x
                
                past_frames.append(row)
                

        if len(valid_pids) == 0:
            return []

        for frame in range(NUM_PREV_FRAMES + 1, NUM_PREV_FRAMES + 1 + NUM_FUTURE_FRAMES):
            for rpid in valid_pids:
                # create length 17 aray of -1
                row = np.full(17, -1.0)
                # get x, y
                x, y = -1.0, -1.0
                row[0] = frame
                row[1] = rpid
                # we need to flip the x and y coordinates for the model
                row[-4] = y
                row[-2] = x
                past_frames.append(row)


        past_frames = np.array(past_frames).astype(str)
        past_frames[:,2] = "Pedestrian"

        np.savetxt(INPUT_FILE_PATH, past_frames, delimiter=" ", fmt="%s")

        return past_frames


    # Run the model on the data
    def run_model(self):
        self.model_process.stdin.write("GET PREDICTIONS" + "\n")
        self.model_process.stdin.flush()

        while True:
            start_up_message = self.model_process.stdout.readline().strip()
            print(start_up_message)
            if start_up_message == "READY":
                break
        
        # read results from file
        sample_motion_3D = torch.load('sample_motion.pt')
        frame = torch.load('frame.pt')
        valid_id = torch.load('valid_id.pt')
        return sample_motion_3D, valid_id, frame
        

    def rate(self):
        return 0.5 # once every 2 seconds
    
    def state_inputs(self):
        return ['tracking_frames']
    
    def state_outputs(self):
        return ['predicted_trajectories']
    
    def test_set_data(self, zed_image, point_cloud, camera_info='dummy'):
        self.zed_image = zed_image
        self.point_cloud = point_cloud
        self.camera_info = camera_info

    def initialize(self):
        print("initializing trajpredict")
        pass

    # May not need this if motion planning can just get the velocities themselves
    def estimate_velocity(self, past_values):
        # estimate velocity from past few frames
        self.cur_time = time.time()
        
        pass
    
    # Changing signature of model since we changed the output format of AgentFormer to return the actual tensor
    def convert_data_from_model_output(self, sample_model_3D, valid_id, frame) -> List[Dict[int,List[AgentState]]]:
        agent_list = []
        # sample_model_3D: 3 x ped_id x 12 x 2
        iterations = 0
        for traj in range(sample_model_3D.shape[0]):
            # starttime = time.time()
            # Create the dictionary of pedestrian-future AgentState lists for the current trajectory
            agent_dict = defaultdict(list) # Key: Pedestrian ID | Value: List of AgentStates for each future frame
            for ped_idx in range(sample_model_3D.shape[1]):
                for future_frame_id in range(sample_model_3D.shape[2]):
                    ped_id = valid_id[ped_idx]
                    # flip the x- and y-coordinates back to normal
                    # x = sample_model_3D[traj][ped_idx][future_frame_id][1]
                    # y = sample_model_3D[traj][ped_idx][future_frame_id][0]
                    y, x = sample_model_3D[traj][ped_idx][future_frame_id]

                    # convert the frame_id to time
                    frame_id = frame + future_frame_id + 1
                    frame_time = frame_id / self.frame_rate + self.cur_time

                    # create an AgentState object
                    pose = ObjectPose(t=frame_time, x=x, y=y, z=0, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.START)

                    # dimensions of a pedestrian (not accurate)
                    dims = PEDESTRIAN_DIMS
                    # velocity = esimate velocity from past few frames
                    # velocity = self.estimate_velocity(past few frames)
                    agent_state = AgentState(pose=pose, dimensions=dims, outline=None, type=AgentEnum.PEDESTRIAN, activity=AgentActivityEnum.MOVING, velocity=(0, 0, 0), yaw_rate=0)
                    agent_dict[ped_id].append(agent_state)
                    iterations += 1
                # print("single pedestrian time", time.time()-starttime)
            agent_list.append(agent_dict)
            # print(starttime-time.time(), "iteration", traj)

        # print(time.time() - starttime, "line 245")
        # print(iterations, "iterations of innermost loop")
        return agent_list
            

    # takes in the agent states of the past 8 frames and returns the predicted trajectories of the agents in the next 12 frames
    # outputs dictionary where key is the sampler id and value is the list of agent states for the next 12 frames
    # def update(self, past_agent_states : List[str]) -> List[Dict[List[AgentState]]]:
    # Assuming that past_agent_states is actually a numpy array instead of just a list of strings
    # past_agent_states.shape: [num_frames_in_model * (peds_in_frame for frame in frames), 17]
    def update(self, past_agent_states) -> Dict[AgentEnum, Dict[int, Dict[int, AgentState]]]:
        # print("input to trajpredict, ", len(past_agent_states.items()))

        data = copy.deepcopy(past_agent_states)
        self.cur_time = time.time()
        # flip the x- and y-coordinates for each pedestrian in each frame

        if data is None or data == []: 
            print("NO INPUT TO trajpredict")
            return []

        model_input = self.convert_data_to_model_input(data)

        # print(time.time() - self.cur_time, "COMPUTED MODEL INPUT")

        if len(model_input) == 0 or model_input.shape == (0, ):
            print("no pedestrians found, no need to run model. ")
            return []

        # print("input to model:")
        # for m in model_input:
        #     print(m)
        # save model_input to file 
        np.savetxt(INPUT_FILE_PATH, model_input, delimiter=" ", fmt="%s")
        # run the traj prediction model on data
        sample_model_3D, valid_ids, frame = self.run_model()
        model_finish = time.time()
        print(model_finish - self.cur_time, "RAN MODEL")

        # output frame 7/2.5 -> time  + cur_time = detection_time

        # convert data to AgentState objects make sure to convert the frames to time(which will add to the AgentPose object)
        agent_list = self.convert_data_from_model_output(sample_model_3D, valid_ids, frame)
        print(time.time() - model_finish, "CONVERT DATA")
        
        # print("agent list", len(agent_list))
        # return data
        return agent_list
        
    def cleanup(self):
        # clean up subprocess which runs the model.
        self.model_process.stdin.close()
        self.model_process.wait()
        pass

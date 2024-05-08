# model1.py
import sys
import logging
import numpy as np
import argparse
import os
import sys
import subprocess
import shutil
import sys, copy
import time
from typing import Dict
import torch
import pandas as pd
import numpy as np
from collections import defaultdict

# path = "GEMstack/onboard/prediction/agentformer"
sys.path.append('perception/agentformer')
from utils.torch import *
from utils.config import Config
from model.model_lib import model_dict
from utils.utils import prepare_seed, print_log, mkdir_if_missing
from dataloader_improved import data_generator

# How many of the top trajectories that we want to keep
NUM_TOP_TRAJECTORIES = 5
# 8 frames used to base future trajectories off of (current frame plus previous 7)
NUM_PREV_FRAMES = 7
NUM_FUTURE_FRAMES = 12
INPUT_FILE_PATH = 'GEMstack/onboard/perception/agentformer/model_input.txt'

def get_model_prediction(data, sample_k, model):
    model.set_data(data)
    sample_motion_3D, data = model.inference(mode='infer', sample_num=sample_k, need_weights=False)
    sample_motion_3D = sample_motion_3D.transpose(0, 1).contiguous()
    return sample_motion_3D

def run_model(model, cfg, gt_data):
    best_samples = [0,1,17,18,19]
    # Perform model1 operations using the input data
    epoch = 30
    split = 'test'
    log = open(os.path.join(cfg.log_dir, 'log_test.txt'), 'w')
    generator = data_generator(cfg, gt_data, log, split=split, phase='testing')
    save_dir = f'{cfg.result_dir}/epoch_{epoch:04d}/{split}'; mkdir_if_missing(save_dir)
    sample_motion_3D, valid_id, frame = run_model_on_data(generator, save_dir, cfg, model, device, log) # [samples, num_agents, future_frames, 2]
    sample_motion_3D = sample_motion_3D[best_samples]
    # write to file or return  
    # select the
    # flush the output to stdout
    sys.stdout.flush()
    # save results to file
    torch.save(sample_motion_3D, 'sample_motion.pt')
    torch.save(valid_id, 'valid_id.pt')
    torch.save(frame, 'frame.pt')
    return sample_motion_3D, valid_id, frame

def run_model_on_data(generator, save_dir, cfg, model, device, log):
    total_num_pred = 0
    while not generator.is_epoch_end():
        data = generator()
        if data is None:
            continue
        seq_name, frame = data['seq'], data['frame']

        frame = int(frame)
        
        with torch.no_grad():
            sample_motion_3D = get_model_prediction(data, cfg.sample_k, model)
        sample_motion_3D = sample_motion_3D * cfg.traj_scale
    
        return sample_motion_3D, data['valid_id'], frame
    
# {framenum: {ped_id: {x or y: float}}}
def convert_data_to_model_input(past_agent_states: Dict[int, Dict[int, Dict[str, float]]]) -> np.ndarray:
    # get tracked frames for pedestrian agents
    pedestrian_agent_states = past_agent_states
    if len(pedestrian_agent_states) == 0:
        return []


    # get the 8 most recent frames(highest frame number) from the past_agent_states
    past_agent_states = []

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
            x, y = agent_state["x"], agent_state["y"]
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

# output:
# [{ped_id: {x or y: float}}]
def convert_data_from_model_output(self, sample_model_3D, valid_id, frame):
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
                y, x = sample_model_3D[traj][ped_idx][future_frame_id]
                agent_state = {"x": x, "y": y}
                # convert the frame_id to time
                # frame_id = frame + future_frame_id + 1
                # frame_time = frame_id / self.frame_rate + self.cur_time


                # # create an AgentState object
                # pose = ObjectPose(t=frame_time, x=x, y=y, z=0, yaw=0, pitch=0, roll=0, frame=ObjectFrameEnum.START)

                # # dimensions of a pedestrian (not accurate)
                # dims = PEDESTRIAN_DIMS
                # # velocity = esimate velocity from past few frames
                # # velocity = self.estimate_velocity(past few frames)
                # agent_state = AgentState(pose=pose, dimensions=dims, outline=None, type=AgentEnum.PEDESTRIAN, activity=AgentActivityEnum.MOVING, velocity=(0, 0, 0), yaw_rate=0)
                agent_dict[ped_id].append(agent_state)
                iterations += 1
            # print("single pedestrian time", time.time()-starttime)
        agent_list.append(agent_dict)
        # print(starttime-time.time(), "iteration", traj)

    # print(time.time() - starttime, "line 245")
    # print(iterations, "iterations of innermost loop")
    return agent_list


if __name__ == "__main__":
    config_file = 'cfg/eth_ucy/inference_data/inference.yml'
    cfg = Config(config_file)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    model = model_dict['dlow'](cfg)
    model.set_device(device)
    model.eval()

    cp_path = cfg.model_path
    print(f'loading model from checkpoint: {cp_path}')
    model_cp = torch.load(cp_path, map_location=device)
    model.load_state_dict(model_cp['model_dict'], strict=False)

    cp_path = cfg.model_path % 5
    print(f'loading model from checkpoint: {cp_path}')
    model_cp = torch.load(cp_path, map_location='cpu')
    model.load_state_dict(model_cp['model_dict'], strict=False)
    # print("we here")
    # create logger for file logging.txt
    # Log some messages


    while True:
        # for line in sys.stdin:
        #     input_data = line.strip()
        #     if not input_data:
        #         break
        _ = sys.stdin.readline().strip()
        # read in txt file into numpy array of strings with delimiter ' '
        input_data = np.genfromtxt(INPUT_FILE_PATH, delimiter=' ')
        run_model(model, cfg)
        print("READY")
        sys.stdout.flush()
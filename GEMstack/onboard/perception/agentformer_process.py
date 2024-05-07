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

if __name__ == "__main__":
    cfg = Config('inference')
    # device = 'cuda'
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    model = model_dict['dlow'](cfg)
    model.set_device(device)
    model.eval()

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
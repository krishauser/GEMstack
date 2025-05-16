import yaml
import numpy as np
from calib_util import load_ex, load_in

#  
# THIS FILE SHOULD BE RUN FROM ITS LOCAL DIRECTORY FOR THE PATHS TO WORK
#

# Destination files name
output_file = 'gem_e4_perception_cameras.yaml'

# Collect names of all sensors and associated extrinsic/intrinsic files
camera_files = {'front': ['gem_e4_oak.yaml', 'gem_e4_oak_in.yaml'],
                'front_right': ['gem_e4_fr.yaml', 'gem_e4_fr_in.yaml'],
                'front_left': ['gem_e4_fl.yaml', 'gem_e4_fl_in.yaml'],
                'back_right': ['gem_e4_rr.yaml', 'gem_e4_rr_in.yaml'],
                'back_left': ['gem_e4_rl.yaml', 'gem_e4_rl_in.yaml']}
lidar_file = 'gem_e4_ouster.yaml'

# Initialize variables
output_dict = {'cameras': {}}
T_lidar_to_vehicle = load_ex(lidar_file, 'matrix')

# Collect data for all cameras
for camera in camera_files:
    # Load from files
    ex_file = camera_files[camera][0]
    in_file = camera_files[camera][1]
    T_camera_to_vehicle = load_ex(ex_file, 'matrix')
    K, D = load_in(in_file, 'matrix', return_distort=True)

    # Calculate necessary values
    T_lidar_to_camera = np.linalg.inv(T_camera_to_vehicle) @ T_lidar_to_vehicle

    # Store in the proper format
    camera_dict = {}
    camera_dict['K'] = K
    camera_dict['D'] = D
    camera_dict['T_l2c'] = T_lidar_to_camera
    camera_dict['T_l2v'] = T_lidar_to_vehicle
    for key in camera_dict:
        if type(camera_dict[key]) == np.ndarray:
            camera_dict[key] = camera_dict[key].tolist()
    output_dict['cameras'][camera] = camera_dict

# Write to file
with open(output_file,'w') as stream:
    yaml.safe_dump(output_dict, stream)
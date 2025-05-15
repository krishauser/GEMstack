# BEVFusion Set Up Instructions
These instructions were tested on T4 g4dn.xlarge AWS instances with Arara Ubuntu 20.04 DCV images.

## Set Up Instructions for Cuda 11.3
### Set Up your Nvida Driver
```
sudo apt-get update
sudo apt-get install -y ubuntu-drivers-common
ubuntu-drivers devices
sudo apt-get install -y nvidia-driver-565
sudo reboot
```

### Check to make sure that your nvidia driver was set up correctly:
```
nvidia-smi
```

### Install Cuda 11.3
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.3.0/local_installers/cuda-repo-ubuntu2004-11-3-local_11.3.0-465.19.01-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-11-3-local_11.3.0-465.19.01-1_amd64.deb # This never copy pastes right. Just manually type
sudo apt-key add /var/cuda-repo-ubuntu2004-11-3-local/7fa2af80.pub
sudo apt-get update
sudo apt-get -y install cuda-11-3
```

### Manually modify your bashrc file to include Cuda 11.3
```
sudo nano ~/.bashrc 
```

Add the next 2 lines to the bottom of the file:
```
export PATH=/usr/local/cuda-11.3/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.3/lib64:$LD_LIBRARY_PATH
```

Ensure you source your bashrc file:
```
source ~/.bashrc
nvidia-smi
```

### Set Up Miniconda
```
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash ~/Miniconda3-latest-Linux-x86_64.sh
source ~/.bashrc
```

### Set Up mmdetection3d:
```
conda create --name openmmlab python=3.8 -y
conda activate openmmlab
conda install pytorch=1.10.2 torchvision=0.11.3 cudatoolkit=11.3 -c pytorch
pip install -U openmim
mim install mmengine
mim install 'mmcv>=2.0.0rc4, <2.2.0'
mim install 'mmdet>=3.0.0,<3.3.0'
pip install cumm-cu113
pip install spconv-cu113
git clone https://github.com/open-mmlab/mmdetection3d.git -b dev-1.x
cd mmdetection3d
pip install -v -e .
python projects/BEVFusion/setup.py develop
```

### Run this afterwards to verify BEVFusion has been set up correctly:
```
python projects/BEVFusion/demo/multi_modality_demo.py demo/data/nuscenes/n015-2018-07-24-11-22-45+0800__LIDAR_TOP__1532402927647951.pcd.bin demo/data/nuscenes/ demo/data/nuscenes/n015-2018-07-24-11-22-45+0800.pkl projects/BEVFusion/configs/bevfusion_lidar-cam_voxel0075_second_secfpn_8xb4-cyclic-20e_nus-3d.py ~/Downloads/bevfusion_lidar-cam_voxel0075_second_secfpn_8xb4-cyclic-20e_nus-3d-5239b1af.pth --cam-type all --score-thr 0.2 --show
```

## Set Up Instructions for Cuda 11.1
### Set Up your Nvida Driver
```
sudo apt-get update
sudo apt-get install -y ubuntu-drivers-common
ubuntu-drivers devices
sudo apt-get install -y nvidia-driver-565
sudo reboot
```

### Check to make sure that your nvidia driver was set up correctly:
```
nvidia-smi
```

### Install Cuda 11.3
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.1.0/local_installers/cuda-repo-ubuntu2004-11-1-local_11.1.0-455.23.05-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-11-1-local_11.1.0-455.23.05-1_amd64.deb
sudo apt-key add /var/cuda-repo-ubuntu2004-11-1-local/7fa2af80.pub
sudo apt-get update
sudo apt-get -y install cuda-11-1
```

### Manually modify your bashrc file to include Cuda 11.3
```
sudo nano ~/.bashrc 
```

Add the next 2 lines to the bottom of the file:
```
export PATH=/usr/local/cuda-11.1/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.1/lib64:$LD_LIBRARY_PATH
```

Ensure you source your bashrc file:
```
source ~/.bashrc
nvidia-smi
```

### Set Up Miniconda
```
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash ~/Miniconda3-latest-Linux-x86_64.sh
source ~/.bashrc
```

### Set Up mmdetection3d:
```
conda create --name openmmlab python=3.8 -y
conda activate openmmlab
pip install torch==1.10.0+cu111 torchvision==0.11.1+cu111 -f https://download.pytorch.org/whl/torch_stable.html
pip install -U openmim
mim install mmengine
mim install 'mmcv>=2.0.0rc4, <2.2.0'
mim install 'mmdet>=3.0.0,<3.3.0'
pip install cumm-cu111
pip install spconv-cu111
git clone https://github.com/open-mmlab/mmdetection3d.git -b dev-1.x
cd mmdetection3d
pip install -v -e .
python projects/BEVFusion/setup.py develop
```

### Run this afterwards to verify BEVFusion has been set up correctly:
```
python projects/BEVFusion/demo/multi_modality_demo.py demo/data/nuscenes/n015-2018-07-24-11-22-45+0800__LIDAR_TOP__1532402927647951.pcd.bin demo/data/nuscenes/ demo/data/nuscenes/n015-2018-07-24-11-22-45+0800.pkl projects/BEVFusion/configs/bevfusion_lidar-cam_voxel0075_second_secfpn_8xb4-cyclic-20e_nus-3d.py ~/Downloads/bevfusion_lidar-cam_voxel0075_second_secfpn_8xb4-cyclic-20e_nus-3d-5239b1af.pth --cam-type all --score-thr 0.2 --show
```
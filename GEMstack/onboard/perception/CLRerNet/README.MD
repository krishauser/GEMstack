### To Use Lane Detection, You Need to first install the CLRerNet here
- First, clone the repository at 
- Second, install the virtual env using these commands
    ```python
    conda create -n mmlab python=3.10 -y
    conda activate mmlab
    pip install torch==2.0.1+cu117 torchvision==0.15.2+cu117 torchaudio==2.0.2 --index-url https://download.pytorch.org/whl/cu117
    pip install -U openmim
    mim install "mmcv>=2.0.0rc4,<2.2.0"
    pip install mmdet
    pip install -r requirements.txt
    pip install "numpy<2"
    conda install -c conda-forge cudatoolkit-dev
    cd libs/models/layers/nms
    pip install -e .
    ```
- Third, download the checkpoints and put them under a folder called 'checkpoints'


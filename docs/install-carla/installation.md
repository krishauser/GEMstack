# CARLA Installation 
This project aims to use [CARLA](https://carla.org//) __version 0.9.15__ for simulation testing, here's the way to set-up and test CARLA on your local environment.
Requirements:
- __OS__ : Linux - Ubuntu 20.04
- __Hardware__ : GPU with 8 GB VGA Mem
- __Storage__ :  ~ 20GB for Carla binaries + town-maps
- __Python__ : Python 3.7.X

## Python 3.7.X 
There are multiple ways to set-up python 3.7 on your machine. If you are using any other version currently, it is suggested to use __conda__ or __pyenv(with brew)__ to support multiple python versions. 

### Install conda and setup python 3.7 virtual env ([source](https://medium.com/@anarmammadli/how-to-install-conda-on-ubuntu-b6e67f15a4dd))
Run the following commands on your machine to install conda
```
wget https://repo.anaconda.com/miniconda/Miniconda3-py38_23.3.1-0-Linux-x86_64.sh --no-check-certificate
bash Miniconda3-py38_23.3.1-0-Linux-x86_64.sh 
```

Create a python virtual env and run it

```
conda create -n carla-test python=3.7 anaconda
conda activate carla-test
```

To exit this env later, use _(to exit from __current__ or __base__ environment)_

```
conda deactivate
```

## Carla installation
You can checkout carla home-page (link mentioned above) and try the installation steps from [Get started](https://carla.readthedocs.io/en/latest/start_quickstart/) but there are a few problems with accessing carla server.
To avoid such issues, you can use the following installation commands [source](https://github.com/carla-simulator/carla/issues/7017#issuecomment-1908462106)
__Note__ : _This download and install can take several minutes_
```
sudo apt-get -y install libomp5
```
```
wget https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.15.tar.gz
```
```
mkdir ~/carla-simulator
tar -xzvf CARLA_0.9.15.tar.gz -C ~/carla-simulator/
```
```
python -m pip install carla==0.9.15
```
```
python -m pip install -r ~/carla-simulator/PythonAPI/examples/requirements.txt
```
```
pip install --user pygame numpy
```

Post installation steps to install carla-client can be followed from the user-guide, for simplicity run these commands
```
cd ~/carla-simulator/PythonAPI/carla/dist
ls -lart
```
You should __see__ something like the following:
```
total 127020
-rw-rw-r-- 1 sonar sonar 33164371 Nov 10 05:48 carla-0.9.15-py2.7-linux-x86_64.egg
-rw-rw-r-- 1 sonar sonar 31243089 Nov 10 05:48 carla-0.9.15-py3.7-linux-x86_64.egg
-rw-rw-r-- 1 sonar sonar 31847920 Nov 10 05:48 carla-0.9.15-cp37-cp37m-manylinux_2_27_x86_64.whl
-rw-rw-r-- 1 sonar sonar 33798949 Nov 10 05:48 carla-0.9.15-cp27-cp27mu-manylinux_2_27_x86_64.whl
drwxrwxr-x 2 sonar sonar     4096 Nov 10 05:48 .
drwxrwxr-x 4 sonar sonar     4096 Nov 10 05:48 ..
```
The third file in the list is the __whl__ file you can use for use for python-carla setup.
Run the following commands to install python-carla with the wheel
```
pip install carla-0.9.15-cp37-cp37m-manylinux_2_27_x86_64.whl
```

Given that all the above commands run successfully, try to execute the following command
```
bash ~/carla-simulator/CarlaUE4.sh 
```
__Developer's suggestion__
The above script can give output that is limited by the compute's capacity. To get the best output when you perform tests, use the following args 

```
bash ~/carla-simulator/CarlaUE4.sh -prefernvidia -RenderOffScreen -quality-level=Low
```
(On a 6 year old _Nvidia GTX GeForce 1050 Ti_, you can observe 4-6x FPS improvement)

__You must see a visualization output like [this](https://uillinoisedu-my.sharepoint.com/:v:/g/personal/qilong3_illinois_edu/EWqpObuKsexFtbSm0gy3H-cBcHbpr3fKoO1ZCe-pCYoXaw?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=2NhkWr)__

## Inside CARLA
If you want to start using CARLA to obtain LiDAR/Camera data for testing purposes, you can get most of the code/results by runnning __~/carla-simulator/PythonAPI/examples/manual_control.py__
Use the following code to run this file
```
conda activate carla-test
python ~/carla-simulator/PythonAPI/examples/manual_control.py
```

You'll see an output window where you'll drive a car manually.

As a default, you'll see the __Town10_Opt map__. For the requirements of lot-parking, curb-side driving and road driving, __Town05_Opt__ map is well suited. In order to change the map, and get better results in CARLA simulation for this map, you can run the following python file with args:
```
python ~/carla-simulator/PythonAPI/util/config.py --map Town05_Opt --no-rendering --no-sync
```
<table>
<tr>
<td>
--map map-name
</td>
<td> 
changes your current map
</td>
</tr>
<tr>
<td>
--no-rendering
</td> 
<td>
pre-renders the entire map to improve output
</td>
</tr>
<tr>
<td>
--no-sync
</td> 
<td>
runs with unlimited sync latency to avoid segmentation fault errors
</td>
</tr>
</table>

__TBC__
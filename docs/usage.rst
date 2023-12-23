Usage
=====

.. _installation:

Installation
------------

To use GEMstack, you will need Python 3.7+ and ROS Noetic.  To run it live on the GEM vehicle, you will also need `PACMOD <http://wiki.ros.org/pacmod>`_, which is Autonomoustuff's low level  interface to the vehicle


Next, clone it from the source Github repository:

.. code-block:: console

	git clone https://github.com/krishauser/GEMstack

Next, install the Python dependencies, running the following command from this folder 


.. code-block:: console

   (.venv) $ pip install -r requirements.txt

This command will install the following packages onto your system:
- opencv-python
- numpy
- scipy
- torch
- shapely
- dacite
- pyyaml

Now, you can run the GEMstack entrypoints from the current folder (recommended). 

Alternatively, you can install GEMstack using pip (not recomended):

.. code-block:: console

   (.venv) $ pip install .

This is not recommended because has a couple of drawbacks:
- You will need to reinstall every time you change code in this folder.
- You might make changes in this directory, e.g., via `git pull`, and then forget to reinstall, so the changes won't be reflected when you run your code.
- If you added model or roadgraph files, e.g., to the `knowledge` directory, they may not be installed.  You will need to edit `pyproject.toml` to include those files.

If you are finding versioning errors due to an accidentally installed version of GEMstack conflicting with your work in this folder, make sure to run ``pip uninstall GEMstack``.





Launching the stack
-------------------

To launch a simulation using:

.. code-block:: console

	python main.py GEMstack/launch/LAUNCH_FILE.yaml
	
where ``LAUNCH_FILE.yaml`` is your preferred simulation launch file.  Inspect the simulator classes in ``GEMstack/onboard/interface/gem_simulator.py`` for more information about configuring a simulator.

To launch onboard behavior you will open four terminal windows, and in each of them run:
- ``roscore``
- ``roslaunch basic_launch sensor_init.launch``
- ``roslaunch basic_launch visualization.launch``
- ``python main.py GEMstack/launch/LAUNCH_FILE.yaml`` where ``LAUNCH_FILE.yaml`` is your preferred launch file. 


Note that if you try to use ``import GEMstack`` in a script or Jupyter notebook anywhere outside of this directory, Python will not know where the ``GEMstack`` module is.  If you wish to import ``GEMstack`` from a script located in a separate directory, you can put


.. code-block:: python

	import sys
	import os
	sys.path.append(os.getcwd())   #or enter the absolute path of this directory

	import GEMstack


at the top of your script.  Then, you can run the script from this directory via ``python PATH/TO/SCRIPT/myscript.py``.  See the scripts in ``testing`` for an example of how this is done.


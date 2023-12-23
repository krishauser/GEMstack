"""Top level package for GEMstack.  Submodules include:

- onboard: All algorithms governing onboard behavior are located here.  These algorithms may make use of items in the `knowledge/` stack.
- offboard: Creation and management of data and knowledge, running off of the vehicle.
- knowledge: Models and parameters common to onboard / offboard use.  The file "current.py" in each directory will store the current model being used.
- mathutils: Math utilities common to onboard / offboard use.
- utils: Other utilities common to onboard / offboard use.
- `GEMstack/launch/`: Launch scripts are listed here.  Specify which configuration you want to use as an argument to `main.py`.

"""

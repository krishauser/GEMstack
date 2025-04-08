from ...utils.config import load_config_recursive
import os,sys


# Identify the vehicle used and load settings else default to vehicle config at current.yaml'
# default currently uses GEM e4
vehicle = "default"

for arg in sys.argv:
    if arg.startswith('--'):
        k,v = arg.split('=',1)
        k = k[2:]
        v = v.strip('"')
        if k == "vehicle":
            vehicle = v
            sys.argv.remove(arg)
            break

if(vehicle == 'e2'):
    vehicle_config = 'e2.yaml'
elif(vehicle == 'e4'):
    vehicle_config = 'current.yaml'
elif(vehicle == 'e2_gazebo'):
    vehicle_config = 'e2_gazebo.yaml'
elif(vehicle == 'e4_gazebo'):
    vehicle_config = 'e4_gazebo.yaml'
else:
    print("Unknown vehicle argument passed")
    vehicle = "default"
    vehicle_config = 'current.yaml'

SETTINGS_FILE = os.path.join(os.path.split(__file__)[0],vehicle_config)
print("**************************************************************")
print(f"Loading {vehicle} settings from",SETTINGS_FILE)
print("**************************************************************")
SETTINGS = load_config_recursive(SETTINGS_FILE)

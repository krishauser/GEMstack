from ...utils.config import load_config_recursive
import os
SETTINGS_FILE = os.path.join(os.path.split(__file__)[0],'current.yaml')
print("**************************************************************")
print("Loading default settings from",SETTINGS_FILE)
print("**************************************************************")
SETTINGS = load_config_recursive(SETTINGS_FILE)

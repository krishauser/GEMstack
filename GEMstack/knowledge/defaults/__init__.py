from ...utils.config import load_config_recursive
import os
SETTINGS = load_config_recursive(os.path.join(os.path.split(__file__)[0],'current.yaml'))

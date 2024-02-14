import json
from ..knowledge import defaults
import copy
from typing import List,Union,Any

SETTINGS = None

def load_settings():
    """Loads the settings object for the first time.
    
    Order of operations is to look into defaults.SETTINGS, and then
    look through the command line arguments to determine whether the user has
    overridden any settings using --KEY=VALUE.
    """
    global SETTINGS
    if SETTINGS is not None:
        return
    import os
    import sys
    SETTINGS = copy.deepcopy(defaults.SETTINGS)
    for arg in sys.argv:
        if arg.startswith('--'):
            k,v = arg.split('=',1)
            k = k[2:]
            v = v.strip('"')
            try:
                v = json.loads(v)
            except json.decoder.JSONDecodeError:
                pass
            if v.startswith('{'):
                set(k,v,leaf_only=False)
            else:
                set(k,v)
    
    return


def settings():
    """Returns all global settings, loading them if necessary."""
    global SETTINGS
    load_settings()
    return SETTINGS


def get(path : Union[str,List[str]], defaultValue=KeyError) -> Any:
    """Retrieves a setting by a list of keys or a '.'-separated string."""
    global SETTINGS
    load_settings()
    if isinstance(path,str):
        path = path.split('.')
    try:
        val = SETTINGS
        for key in path:
            val = val[key]
        return val
    except KeyError:
        if defaultValue is KeyError:
            print("settings.py: Unable to get",path,"available keys:",val.keys())
            raise
        return defaultValue

def set(path : Union[str,List[str]], value : Any, leaf_only=True) -> None:
    """Sets a setting by a list of keys or a '.'-separated string.
    
    If leaf_only=True (default), we prevent inadvertently deleting parts of the
    settings dictionary.
    """
    global SETTINGS
    load_settings()
    if isinstance(path,str):
        path = path.split('.')
    val = SETTINGS
    if len(path) == 0:
        raise KeyError("Cannot set top-level settings")
    for key in path[:-1]:
        val = val[key]
    if leaf_only:
        if path[-1] in val and isinstance(val[path[-1]],dict):
            raise ValueError("Can only set leaves of the settings dictionary when leaf_only=True is given")
    val[path[-1]] = value

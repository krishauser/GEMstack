import json
import yaml
import os
from typing import Any, IO
import collections
import warnings

def save_config(fn : str, config : dict) -> None:
    """Saves a configuration file."""
    if fn.endswith('yaml') or fn.endswith('yml'):
        with open(fn,'w') as f:
            yaml.dump(config, f, yaml.Dumper)
    elif fn.endswith('json'):
        with open(fn,'w') as f:
             json.dump(config,f)
    else:
        raise IOError("Config file not specified as .yaml, .yml, or .json extension")


def load_config_recursive(fn : str) -> dict:
    """Loads a configuration file with !include and !relative_path directives."""
    if fn.endswith('yaml') or fn.endswith('yml'):
        with open(fn,'r') as f:
            res = yaml.load(f,_Loader)
        return res
    elif fn.endswith('json'):
        with open(fn,'r') as f:
            res = json.load(f)
        base,_ = os.path.split(fn)
        return _load_recursive(res,base)
    else:
        raise IOError("Config file not specified as .yaml, .yml, or .json extension")




class _Loader(yaml.SafeLoader):
    """YAML Loader with `!include` and `!relative_path` directives."""

    def __init__(self, stream: IO) -> None:
        """Initialise Loader."""

        try:
            self._root = os.path.split(stream.name)[0]
        except AttributeError:
            self._root = os.path.curdir

        super().__init__(stream)


def _construct_include(loader: _Loader, node: yaml.Node) -> Any:
    """Include file referenced at node."""
    return _load_config_or_text_recursive(os.path.join(loader._root, loader.construct_scalar(node)))

def _construct_relative_path(loader: _Loader, node: yaml.Node) -> Any:
    return os.path.normpath(os.path.join(loader._root, loader.construct_scalar(node)))

yaml.add_constructor('!include', _construct_include, _Loader)

yaml.add_constructor('!relative_path', _construct_relative_path, _Loader)

def _load_config_or_text_recursive(fn : str) -> dict:
    """Loads a configuration file with !include and !relative_path directives."""
    if fn.endswith('yaml') or fn.endswith('yml'):
        with open(fn,'r') as f:
            res = yaml.load(f,_Loader)
        return res
    elif fn.endswith('json'):
        with open(fn,'r') as f:
            res = json.load(f)
        base,_ = os.path.split(fn)
        return _load_recursive(res,base)
    else:
        with open(fn,'r') as f:
            return ''.join(f.readlines())

def _load_recursive(obj, folder : str):
    if isinstance(obj,dict):
        for k,v in obj.copy().items():
            obj[k] = _load_recursive(v,folder)
    elif isinstance(obj,list):
        for i in range(len(obj)):
            obj[i] = _load_recursive(obj[i],folder)
    elif isinstance(obj,str):
        if obj.startswith('!include '):
            fn = obj.split(' ',1)[1]
            return _load_config_or_text_recursive(os.path.normpath(os.path.join(folder,fn)))
        elif obj.startswith('!!include'):
            return obj[1:]
    return obj



def update_recursive(d : dict, u : dict, caution_callback=None) -> dict:
    """Updates a dictionary d with another dictionary u recursively.

    The update happens in-place and d is returned.
    """
    if not isinstance(d, collections.abc.Mapping):
        raise ValueError("Trying to update a non-dict with a dict")
    for k, v in u.items():
        sub_callback = None
        if caution_callback:
            if k not in d:
                caution_callback(k)
            else:
                sub_callback = lambda k2:caution_callback(k + '.' + k2)
        if isinstance(v, collections.abc.Mapping):
            if not isinstance(d.get(k, {}), collections.abc.Mapping):
                d[k] = {}
            d[k] = update_recursive(d.get(k, {}), v, sub_callback)
        else:
            d[k] = v
    return d
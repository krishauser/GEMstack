import dacite
from dataclasses import dataclass, is_dataclass, asdict
import json
from enum import Enum
import typing

REGISTRY = dict()

def register(klass, name=None, version=None) -> None:
    """Decorator that declares a class to be serializable/deserializable.
    
    If you wish to have multiple versions of a class, you can provide the
    `version` keyword.
    """
    if name is None:
        name = klass.__name__
    if version is None:
        version = ''
    else:
        version = str(version)
    if name not in REGISTRY:
        REGISTRY[name] = dict()
    if version in REGISTRY[name]:
        raise ValueError("A data class of name {} and version '{}' already exists".format(name,version))
    REGISTRY[name][version] = klass
    setattr(klass,'__SERIALIZATION_NAME__',name)
    setattr(klass,'__SERIALIZATION_VERSION__',version)
    return klass



def _custom_asdict_factory(data):
    """Converts enums to their string values"""
    def convert_value(obj):
        if isinstance(obj, Enum):
            return obj.value
        return obj

    return dict((k, convert_value(v)) for k, v in data)


def is_registered(obj) -> bool:
    """Returns True if this is a previously registered class"""
    return '__SERIALIZATION_NAME__' in obj.__class__.__dict__

def json_encode(obj, format=str):
    if format is str:
        return json.dumps(obj)
    elif format is bytes:
        return json.dumps(obj).encode('utf-8')
    elif format is dict:
        return obj
    elif format == 'ros':
        from std_msgs import String
        res = String()
        res.data = json.dumps(obj)
        return res
    else:
        raise ValueError("Format needs to be str, bytes, dict, or 'ros'")

def json_decode(data) -> dict:
    if isinstance(data,str):
        return json.loads(data)
    elif isinstance(data,bytes):
        return  json.loads(data.decode('utf-8'))
    elif isinstance(data,dict):
        return  data
    else:
        from std_msgs import String
        if isinstance(data,String):
            return json.loads(data.data)
        else:
            raise ValueError("Format needs to be str, bytes, dict, or ROS String")


def serialize(obj, format=str):
    """Serializes an instanceof a class previously declared using
    `register()`.
    
    `format` can be `str`, `bytes`, `dict`, or `"ros"`.
    """
    if not hasattr(obj.__class__,'__SERIALIZATION_NAME__'):
        raise ValueError("Provided object of class {} which was not previously registered".format(obj.__class__.__name__))
    if not is_dataclass(obj):
        raise ValueError("Provided object of class {} which is not a dataclass".format(obj.__class__.__name__))
    
    values = asdict(obj, dict_factory=_custom_asdict_factory)
    frame = {'type':obj.__class__.__SERIALIZATION_NAME__,'data':values}
    if obj.__class__.__SERIALIZATION_VERSION__:
        frame['version'] = obj.__class__.__SERIALIZATION_VERSION__
    return json_encode(frame,format)


def deserialize(data):
    """Deserializes data into an instance of a class previously declared using
    `register()`.
    
    `data` can be a `str`, `bytes`, `dict`, or ROS std_msgs/String object.
    """
    global REGISTRY
    name,version,data = deserialize_raw(data)
    if name not in REGISTRY:
        raise IOError("Class of type {} not found in registry".format(name))
    if version not in REGISTRY[name]:
        raise IOError("Version {} of type {} not found in registry".format(version,name))
    return dacite.from_dict(REGISTRY[name][version],data, config=dacite.Config(cast=[Enum,tuple,typing.Tuple]))
 
 
def deserialize_raw(data) -> tuple:
    """Deserializes data into a tuple (name, version, dict).  More error-
    tolerant than deserialize.
    """
    frame = json_decode(data)
    if not isinstance(frame,dict):
        raise IOError("Did not get a dict from JSON-encoded data")
    if 'type' not in frame:
        raise IOError("Data did not contain 'type' key")
    if 'data' not in frame:
        raise IOError("Data did not contain 'data' key")
    name = frame['type']
    version = frame.get('version','')
    return name,version,frame['data']


def serialize_collection(objs, format=str):
    """Serializes a collection of registered objects into a serialized
    format.
    
    `format` can be `str`, `bytes`, `dict`, or `"ros"`.
    """
    if format is dict:
        if isinstance(objs,dict):
            objs = dict((k,serialize_collection(v,dict)) for k,v in objs.items())
        elif isinstance(objs,list):
            objs = [serialize_collection(v,dict) for v in objs]
        elif is_registered(objs):
            objs = serialize(objs,dict)
        return objs
    return json_encode(serialize_collection(objs,dict),format)


def deserialize_collection(data):
    """Deserializes a message into a collection of registered object instances.

    We detect registered types as dicts with "type" and "data" keys, and
    optional "version" key.
    """
    frame = json_decode(data)
    def _recurse(obj):
        if isinstance(obj,dict) and 'type' in obj and 'data' in obj:
            if len(obj) == 2 or (len(obj) == 3 and 'version' in obj):
                return deserialize(obj)
            return obj
        elif isinstance(obj,dict):
            for k,v in obj.items():
                obj[k] = _recurse(v)
            return obj
        elif isinstance(obj,list):
            for i in range(len(obj)):
                obj[i] = _recurse(obj[i])
            return obj
        else:
            return obj
    return _recurse(frame)


def load(file):
    """Loads a JSON file containing serialized data into a registered class."""
    return deserialize(file.read())

def save(obj,file):
    """Saves a JSON file containing serialized data into a registered class."""
    file.write(serialize(obj))
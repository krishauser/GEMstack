from ...utils import settings,config
from ..component import Component
from .execution import ExecutorBase
from .log_replay import LogReplay
import importlib
from typing import Dict,List,Optional
import os

def import_module_dynamic(module_name, parent_module=None):
    if parent_module is not None:
        full_path = parent_module + '.' + module_name
    else:
        full_path = module_name
    return importlib.import_module(full_path)


def make_class(config_info, component_module, parent_module=None, extra_args = None):
    if extra_args is None:
        extra_args = {}
    args = ()
    if isinstance(config_info,str):
        if '.' in config_info:
            component_module,class_name = config_info.rsplit('.',1)
        else:
            class_name = config_info
    else:
        class_name = config_info['type']
        if '.' in class_name:
            component_module,class_name = class_name.rsplit('.',1)
        if 'module' in config_info:
            component_module = config_info['module']
        if 'args' in config_info:
            args = config_info['args']
    if parent_module is not None:
        print("Importing",component_module,"from",parent_module,"to get",class_name)
    else:
        print("Importing",component_module,"to get",class_name)
    module = import_module_dynamic(component_module,parent_module)
    klass = getattr(module,class_name)
    try:
        return klass(*args,**extra_args)
    except TypeError:
        return klass(*args)


def main():
    runconfig = settings.get('run')
    mode = settings.get('run.mode')
    vehicle_interface_settings = settings.get('run.vehicle_interface')
    mission_executor_settings = settings.get('run.mission_execution')
    log_settings = settings.get('run.log',{})
    replay_settings = settings.get('run.replay',{})

    #create top-level components
    vehicle_interface = make_class(vehicle_interface_settings,'default','GEMstack.onboard.interface')
    mission_executor = make_class(mission_executor_settings,'execution','GEMstack.onboard.execution')
    if not isinstance(mission_executor,ExecutorBase):
        raise ValueError("Mission executor must be an Executor")

    drive_pipeline_settings = settings.get('run.drive')
    recovery_pipeline_settings = settings.get('run.recovery')

    #create pipelines and add to executor
    pipeline_settings = {'drive':drive_pipeline_settings,
                        'recovery':recovery_pipeline_settings}
    for k,v in runconfig.items():
        if isinstance(v,dict) and ('perception' in v or 'planning' in v or 'other' in v):
            #possible pipeline
            if k not in pipeline_settings:
                print("Adding non-standard pipeline",k)
                pipeline_settings[k] = v

    existing_components = {}
    for (name,s) in pipeline_settings.items():
        perception_settings = s.get('perception',{})
        planning_settings = s.get('planning',{})
        other_settings = s.get('other',{})

        perception_components = {}   #type: Dict[str,Component]
        for (k,v) in perception_settings.items():
            if str((k,v)) in existing_components:
                perception_components[k] = existing_components[(k,v)]
            else:
                perception_components[k] = make_class(v,k,'GEMstack.onboard.perception', {'vehicle_interface':vehicle_interface})
                existing_components[str((k,v))] = perception_components[k]
        planning_components = {}   #type: Dict[str,Component]
        for (k,v) in planning_settings.items():
            if str((k,v)) in existing_components:
                planning_components[k] = existing_components[(k,v)]
            else:
                if k == 'trajectory_tracking':
                    planning_components[k] = make_class(v,k,'GEMstack.onboard.planning', {'vehicle_interface':vehicle_interface})
                else:
                    planning_components[k] = make_class(v,k,'GEMstack.onboard.planning')
                existing_components[str((k,v))] = planning_components[k]
        other_components = {}   #type: Dict[str,Component]
        for (k,v) in other_settings.items():
            if str((k,v)) in existing_components:
                other_components[k] = existing_components[(k,v)]
            else:
                other_components[k] = make_class(v,k,'GEMstack.onboard.execution', {'vehicle_interface':vehicle_interface})
                existing_components[str((k,v))] = other_components[k]

        #TODO: add logging and ROS topic replay
        if 'replay' in runconfig:
            logfolder = runconfig['replay'].get('log',None)
            if logfolder is not None:
                logmeta = config.load_config_recursive(os.path.join(logfolder,'meta.yaml'))
                replay_topics = runconfig['replay'].get('ros_topics',[])
                
                #TODO: launch a roslog replay of the topics in replay_topics, disable in the vehicle interface
                replay_components = runconfig['replay'].get('components',[])
                for c in replay_components:
                    found = False
                    for component in (perception_components,planning_components,other_components):
                        if c in component:
                            outputs = component[c].state_outputs()
                            for o in outputs:
                                if o not in logmeta['state_log_items']:
                                    raise ValueError("Replay component",c,"has output",o,"which is not in log")
                            print("Replaying component",c,"from log",logfolder,"with outputs",outputs)
                            component[c] = LogReplay(outputs,
                                                    os.path.join(logfolder,'state_log.json'),
                                                    rate=component[c].rate())
                            found = True
                    if not found:
                        raise ValueError("Replay component",c,"not found in pipeline",name)

        mission_executor.add_pipeline(name,perception_components,planning_components,other_components)


    mission_executor.run()


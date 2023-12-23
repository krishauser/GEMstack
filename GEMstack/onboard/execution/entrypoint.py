from ...utils import settings,config
from ..component import Component
from .execution import EXECUTION_PREFIX,ExecutorBase,load_computation_graph
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
    kwargs = {}
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
            if isinstance(args,dict):
                kwargs = args
                args = ()
    if parent_module is not None:
        print(EXECUTION_PREFIX,"Importing",component_module,"from",parent_module,"to get",class_name)
    else:
        print(EXECUTION_PREFIX,"Importing",component_module,"to get",class_name)
    module = import_module_dynamic(component_module,parent_module)
    klass = getattr(module,class_name)
    try:
        return klass(*args,**kwargs,**extra_args)
    except TypeError:
        try:
            return klass(*args,**kwargs)
        except TypeError:
            print(EXECUTION_PREFIX,"Unable to launch module",component_module,"with class",class_name,"and args",args,"kwargs",kwargs)
            raise
        


def main():
    """The main entrypoint for the execution stack."""
    runconfig = settings.get('run')
    mode = settings.get('run.mode')
    vehicle_interface_settings = settings.get('run.vehicle_interface')
    mission_executor_settings = settings.get('run.mission_execution')
    log_settings = settings.get('run.log',{})
    replay_settings = settings.get('run.replay',{})
    load_computation_graph()

    #create top-level components
    vehicle_interface = make_class(vehicle_interface_settings,'default','GEMstack.onboard.interface')
    mission_executor = make_class(mission_executor_settings,'execution','GEMstack.onboard.execution',{'vehicle_interface':vehicle_interface})
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
                print(EXECUTION_PREFIX,"Adding non-standard pipeline",k)
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

        #configure components that would be replayed from log rather than run
        if replay_settings:
            logfolder = replay_settings.get('log',None)
            if logfolder is not None:
                logmeta = config.load_config_recursive(os.path.join(logfolder,'meta.yaml'))

                replay_components = replay_settings.get('components',[])
                for c in replay_components:
                    found = False
                    for component in (perception_components,planning_components,other_components):
                        if c in component:
                            outputs = component[c].state_outputs()
                            for o in outputs:
                                if o not in logmeta['state_log_items']:
                                    raise ValueError("Replay component",c,"has output",o,"which is not in log")
                            print(EXECUTION_PREFIX,"Replaying component",c,"from log",logfolder,"with outputs",outputs)
                            component[c] = LogReplay(outputs,
                                                    os.path.join(logfolder,'state_log.json'),
                                                    rate=component[c].rate())
                            found = True
                    if not found:
                        raise ValueError("Replay component",c,"not found in pipeline",name)

        mission_executor.add_pipeline(name,perception_components,planning_components,other_components)


    #TODO: ROS topic replay
    if replay_settings:
        logfolder = replay_settings.get('log',None)
        if logfolder is not None:
            logmeta = config.load_config_recursive(os.path.join(logfolder,'meta.yaml'))
            replay_topics = replay_settings.get('ros_topics',[])
            
            #TODO: launch a roslog replay of the topics in replay_topics, disable in the vehicle interface

    #configure logging
    if log_settings:
        topfolder = log_settings.get('log','logs')
        prefix = log_settings.get('prefix','')
        from datetime import datetime
        default_suffix = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        suffix = log_settings.get('suffix',default_suffix)
        logfolder = os.path.join(topfolder,prefix+suffix)
        print(EXECUTION_PREFIX,"Logging to",logfolder)
        os.makedirs(logfolder,exist_ok=True)

        #save settings.yaml
        config.save_config(os.path.join(logfolder,'settings.yaml'),settings.settings())
        #configure logging for components
        mission_executor.set_log_folder(logfolder)
        #configure ROS logging
        log_topics = replay_settings.get('ros_topics',[])
        rosbag_options = log_settings.get('rosbag_options','')
        if log_topics:
            command = 'rosbag record --output-name={} {} {}'.format(os.path.join(logfolder,'vehicle.bag'),rosbag_options,' '.join(log_topics))
            print(EXECUTION_PREFIX,"Recording ROS topics with command",command)
            os.system(command)
        #determine whether to log vehicle interface
        log_vehicle_interface = log_settings.get('vehicle_interface',False)
        mission_executor.log_vehicle_interface(log_vehicle_interface)
        #determine whether to log components
        log_components = log_settings.get('components',[])
        mission_executor.log_components(log_components)
        #determine whether to log state
        log_state_attributes = log_settings.get('state',[])
        log_state_rate = log_settings.get('state_rate',None)
        if log_state_attributes:
            mission_executor.log_state(log_state_attributes,log_state_rate)

    vehicle_interface.start()
    try:
        mission_executor.run()
    except Exception as e:
        raise
    finally:
        vehicle_interface.stop()


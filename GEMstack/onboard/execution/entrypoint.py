from ...utils import settings,config
from ..component import Component
from .execution import EXECUTION_PREFIX,ExecutorBase,ComponentExecutor,load_computation_graph,make_class
import multiprocessing
from typing import Dict,List,Optional
import os


def main():
    """The main entrypoint for the execution stack."""

    multiprocessing.set_start_method('spawn')    
    runconfig = settings.get('run')
    mode = settings.get('run.mode')
    vehicle_interface_settings = settings.get('run.vehicle_interface')
    mission_executor_settings = settings.get('run.mission_execution')
    log_settings = settings.get('run.log',{})
    replay_settings = settings.get('run.replay',{})
    load_computation_graph()

    #initialize ros node
    has_ros = False
    try:
        import rospy
        rospy.init_node('GEM_executor',disable_signals=True)
        has_ros = True
    except (ImportError,ModuleNotFoundError):
        if mode == 'simulation':
            print(EXECUTION_PREFIX,"Warning, ROS not found, but simulation mode requested")
        else:
            print(EXECUTION_PREFIX,"Error, ROS not found on system.")
            raise

    #create top-level components
    vehicle_interface = make_class(vehicle_interface_settings,'default','GEMstack.onboard.interface')
    mission_executor = make_class(mission_executor_settings,'execution','GEMstack.onboard.execution',{'vehicle_interface':vehicle_interface})
    if not isinstance(mission_executor,ExecutorBase):
        raise ValueError("Mission executor must be an Executor")

    drive_pipeline_settings = settings.get('run.drive')
    recovery_pipeline_settings = settings.get('run.recovery')
    visualization_settings = settings.get('run.visualization',None)

    #create pipelines and add to executor
    pipeline_settings = {'drive':drive_pipeline_settings,
                        'recovery':recovery_pipeline_settings}
    for k,v in runconfig.items():
        if isinstance(v,dict) and ('perception' in v or 'planning' in v or 'other' in v):
            #possible pipeline
            if k not in pipeline_settings:
                print(EXECUTION_PREFIX,"Adding non-standard pipeline",k)
                pipeline_settings[k] = v

    visualizers = []
    if isinstance(visualization_settings,dict):
        #one visualizer
        visualizers.append(mission_executor.make_component(visualization_settings,'visualization','GEMstack.onboard.visualization',{'vehicle_interface':vehicle_interface}))
    elif isinstance(visualization_settings,list):
        #multiple visualizers
        for v in visualization_settings:
            visualizers.append(mission_executor.make_component(v,'visualization','GEMstack.onboard.visualization',{'vehicle_interface':vehicle_interface}))
    for v in visualizers:
        mission_executor.always_run(v.c.__class__.__name__,v)

    #mark the components that will be replayed rather than run
    if replay_settings:
        logfolder = replay_settings.get('log',None)
        if logfolder is not None:
            replay_components = replay_settings.get('components',[])
            mission_executor.replay_components(replay_components,logfolder)

            #TODO: ROS topic replay
            logmeta = config.load_config_recursive(os.path.join(logfolder,'meta.yaml'))
            replay_topics = replay_settings.get('ros_topics',[])
            
            #TODO: launch a roslog replay of the topics in ros_topics, disable in the vehicle interface

    for (name,s) in pipeline_settings.items():
        perception_settings = s.get('perception',{})
        planning_settings = s.get('planning',{})
        other_settings = s.get('other',{})

        perception_components = {}   #type: Dict[str,ComponentExecutor]
        for (k,v) in perception_settings.items():
            perception_components[k] = mission_executor.make_component(v,k,'GEMstack.onboard.perception', {'vehicle_interface':vehicle_interface})
        planning_components = {}   #type: Dict[str,ComponentExecutor]
        for (k,v) in planning_settings.items():
            planning_components[k] = mission_executor.make_component(v,k,'GEMstack.onboard.planning', {'vehicle_interface':vehicle_interface})
        other_components = {}   #type: Dict[str,ComponentExecutor]
        for (k,v) in other_settings.items():
            other_components[k] = mission_executor.make_component(v,k,'GEMstack.onboard.other', {'vehicle_interface':vehicle_interface})
        
        mission_executor.add_pipeline(name,perception_components,planning_components,other_components)

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

        #configure logging for components
        mission_executor.set_log_folder(logfolder)
        #configure ROS logging
        log_topics = log_settings.get('ros_topics',[])
        rosbag_options = log_settings.get('rosbag_options','')
        mission_executor.log_ros_topics(log_topics, rosbag_options)
        #determine whether to log vehicle interface
        log_vehicle_interface = log_settings.get('vehicle_interface',False)
        if log_vehicle_interface:
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

    if has_ros:
        #need manual ros node shutdown due to disable_signals=True
        rospy.signal_shutdown('GEM_executor finished')


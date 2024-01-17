from __future__ import annotations
from ..component import Component
from ...utils import serialization,logging,config,settings
from typing import List,Optional,Dict,Set,Any
import time
import datetime
import os
import subprocess
import io

class LoggingManager:
    """A top level manager of the logging process.  This is responsible for
    creating log folders, log metadata files, and for replaying components from log
    files."""
    def __init__(self):
        self.log_folder = None        # type: Optional[str]
        self.replayed_components = dict()  # type Dict[str,str]
        self.logged_components = set() # type: Set[str]
        self.component_output_loggers = dict() # type: Dict[str,list]
        self.behavior_log = None
        self.rosbag_process = None
        self.run_metadata = dict()    # type: dict
        self.run_metadata['pipelines'] = []
        self.run_metadata['events'] = []
        self.run_metadata['exit_reason'] = 'unknown'

    def logging(self) -> bool:
        return self.log_folder is not None

    def set_log_folder(self, folder : str) -> None:
        self.log_folder = folder

        #save settings.yaml
        config.save_config(os.path.join(folder,'settings.yaml'),settings.settings())

        #save meta.yaml
        self.run_metadata['start_time'] = time.time()
        self.run_metadata['start_time_human_readable'] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        import subprocess
        git_commit_id = subprocess.check_output(['git','rev-parse','HEAD'])
        self.run_metadata['git_commit_id'] = git_commit_id.decode('utf-8').strip()
        git_branch = subprocess.check_output(['git','rev-parse','--abbrev-ref','HEAD'])
        self.run_metadata['git_branch'] = git_branch.decode('utf-8').strip()
        self.dump_log_metadata()
    
    def close(self):
        if self.behavior_log is not None:
            self.behavior_log.close()
            self.behavior_log = None
        for k,(stdout,stderr) in self.component_output_loggers.items():
            if stdout is not None:
                stdout.close()
            if stderr is not None:
                stderr.close()
        self.component_output_loggers = dict()
    
    def replay_components(self, replayed_components : list, replay_folder : str):
        """Declare that the given components should be replayed from a log folder.

        Further make_component calls to this component will be replaced with
        LogReplay objects.
        """
        #sanity check: was this item logged?
        settings = config.load_config_recursive(os.path.join(replay_folder,'settings.yaml'))
        try:
            logged_components = settings['run']['log']['components']
        except KeyError:
            logged_components = []
        for c in replayed_components:
            if c not in logged_components:
                raise ValueError("Replay component",c,"was not logged in",replay_folder,"(see settings.yaml)")
            self.replayed_components[c] = replay_folder
    
    def component_replayer(self, component_name : str, component : Component) -> Optional[LogReplay]:
        if component_name in self.replayed_components:
            #replace behavior of class with the LogReplay class
            replay_folder = self.replayed_components[component_name]
            outputs = component.state_outputs()
            rate = component.rate()
            assert rate is not None and rate > 0, "Replayed component {} must have a positive rate".format(component_name)
            return LogReplay(outputs,
                                os.path.join(replay_folder,'behavior_log.json'),
                                rate=rate)
        return None

    def dump_log_metadata(self):
        if not self.log_folder:
            return
        from ...utils import config
        config.save_config(os.path.join(self.log_folder,'meta.yaml'),self.run_metadata)

    def load_log_metadata(self):
        if not self.log_folder:
            return
        from ...utils import config
        self.run_metadata = config.load_config_recursive(os.path.join(self.log_folder,'meta.yaml'))

    def component_stdout_file(self,component_name : str) -> str:
        return os.path.join(self.log_folder,component_name+'.stdout.log')

    def component_stderr_file(self,component_name : str) -> str:
        return os.path.join(self.log_folder,component_name+'.stderr.log')

    def log_vehicle_behavior(self,vehicle_interface) -> VehicleBehaviorLogger:
        if not self.log_folder:
            return
        if self.behavior_log is None:
            self.behavior_log = logging.Logfile(os.path.join(self.log_folder,'behavior.json'),delta_format=True,mode='w')
        return VehicleBehaviorLogger(self.behavior_log,vehicle_interface)
    
    def log_state(self,state_attributes : List[str], rate : Optional[float]=None) -> AllStateLogger:
        if not self.log_folder:
            return
        log_fn = os.path.join(self.log_folder,'state.json')
        return AllStateLogger(state_attributes,rate,log_fn)

    def log_components(self,components : List[str]) -> None:
        """Indicate that the state output of these components should be logged"""
        if not self.log_folder:
            return
        if components:
            if self.behavior_log is None:
                self.behavior_log = logging.Logfile(os.path.join(self.log_folder,'behavior.json'),delta_format=True,mode='w')
        self.logged_components = set(components)

    def log_ros_topics(self, topics : List[str], rosbag_options : str = '') -> Optional[str]:
        if topics:
            command = ['rosbag','record','--output-name={}'.format(os.path.join(self.log_folder,'vehicle.bag'))]
            command += rosbag_options.split()
            command += topics
            self.rosbag_process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            return ' '.join(command)
        return None

    def event(self, vehicle_time : float, event_description : str):
        """Logs an event to the metadata."""
        self.run_metadata['events'].append({'time':time.time(),'vehicle_time':vehicle_time,'description':event_description})
        self.dump_log_metadata()

    def pipeline_start_event(self, vehicle_time : float, pipeline_name : str) -> None:
        """Logs a pipeline start event to the metadata."""
        self.run_metadata['pipelines'].append({'time':time.time(),'vehicle_time':vehicle_time,'name':pipeline_name})
        self.dump_log_metadata()

    def exit_event(self, description, force = False):
        """Exit main loop event.  If a prior reason was given, this does nothing
        unless force = True."""
        if self.run_metadata['exit_reason'] == 'unknown' or force:
            self.run_metadata['exit_reason'] = description
            self.dump_log_metadata()

    def log_component_update(self, component : str, vehicle_time : float, state : Any, outputs : List[str]) -> None:
         """Component update"""
         if component in self.logged_components and len(outputs)!=0:
            self.behavior_log.log(state, outputs, vehicle_time)
    
    def log_component_stdout(self, component : str, vehicle_time: float, msg : List[str]) -> None:
        if not self.log_folder:
            return
        if component not in self.component_output_loggers:
            self.component_output_loggers[component] = [None,None]
        if self.component_output_loggers[component][0] is None:
            self.component_output_loggers[component][0] = open(self.component_stdout_file(component),'w')
        timestr = datetime.datetime.fromtimestamp(vehicle_time).strftime("%H:%M:%S.%f")[:-3]
        for l in msg:
            self.component_output_loggers[component][0].write(timestr + ': ' + l + '\n')

    def log_component_stderr(self, component : str, vehicle_time: float, msg : List[str]) -> None:
        if not self.log_folder:
            return
        if component not in self.component_output_loggers:
            self.component_output_loggers[component] = [None,None]
        if self.component_output_loggers[component][1] is None:
            self.component_output_loggers[component][1] = open(self.component_stderr_file(component),'w')
        timestr = datetime.datetime.fromtimestamp(vehicle_time).strftime("%H:%M:%S.%f")[:-3]
        for l in msg:
            self.component_output_loggers[component][1].write(timestr + ': ' + l + '\n')

    def close(self):
        if self.rosbag_process is not None:
            out,err = self.rosbag_process.communicate()  # Will block 
            print('-------------------------------------------')
            print("rosbag output:")
            print(out)
            print()
            loginfo = os.stat(os.path.join(self.log_folder,'vehicle.bag'))
            print("Logged to",os.path.join(self.log_folder,'vehicle.bag'))
            print('Log file size in MegaBytes is {}'.format(loginfo.st_size / (1024 * 1024)))
            print('-------------------------------------------')
            self.rosbag_process = None
    
    def __del__(self):
        self.close()
            

class LogReplay(Component):
    """Substitutes the output of a component with replayed data from a log file.

    There are two forms of log files supported.  The first is a delta format, where
    each line is a dictionary of the form ``{'time':t,ITEM1:{...},ITEM2:{...}}``. 
    
    The second is a state format, where each line is a dictionary of the form
    ``{ITEM1:VAL, ITEM2:VAL, ITEM1_update_time:t, ITEM2_update_time:t}``.
    
    If the `delta_format` attribute is True, then the delta format is assumed.
    """
    def __init__(self, vehicle_interface, outputs : List[str],
                 log_file : str,
                 delta_format=True,
                 rate : float = 10.0,
                 speed_multiplier : float = 1.0):
        self.vehicle_interface = vehicle_interface
        self.outputs = outputs
        self.logfn = log_file
        self._rate = rate
        self.speed_multiplier = speed_multiplier
        self.logfile = logging.Logfile(log_file,delta_format,'r')
        self.start_time = None
    
    def rate(self):
        return self._rate
   
    def state_outputs(self):
        return self.outputs

    def update(self):
        t = self.vehicle_interface.time()
        if self.start_time == None:
            self.start_time = t
        if not self.logfile:
            return

        res,msgs = self.logfile.read(duration_from_start = (t - self.start_time)*self.speed_multiplier, cumulative = True)
        #if nothing new was read, just return None
        if len(msgs)==0:
            return None
        #convert the dict to a list of values in the same order as self.outputs
        res = [res.get(o,None) for o in self.outputs]
        if len(self.outputs)==1:
            return res[0]
        if all(v is None for v in res):
            return None
        return res

    def cleanup(self):
        self.logfile.close()



class VehicleBehaviorLogger(Component):
    def __init__(self,behavior_log, vehicle_interface):
        if isinstance(behavior_log,str):
            behavior_log = logging.Logfile(behavior_log,delta_format=True,mode='w')
        self.behavior_log = behavior_log
        self.vehicle_interface = vehicle_interface
        self.vehicle_log_t_last = None

    def rate(self):
        return None

    def state_inputs(self):
        return ['all']
    
    def state_outputs(self):
        return []

    def update(self,state):
        if state.t != self.vehicle_log_t_last:
            collection = {'vehicle_interface_command':self.vehicle_interface.last_command,
                        'vehicle_interface_reading':self.vehicle_interface.last_reading}
            self.behavior_log.log(collection,t=state.t)
            self.vehicle_log_t_last = state.t


class AllStateLogger(Component):
    def __init__(self,attributes,rate,log_fn):   
        self._rate = rate     
        self.attributes = attributes
        self.state_log = logging.Logfile(log_fn,delta_format=False,mode='w')

    def rate(self):
        return self._rate

    def state_inputs(self):
        return ['all']
    
    def state_outputs(self):
        return []
    
    def cleanup(self):
        if self.state_log:
            self.state_log.close()
            self.state_log = None

    def update(self,state):
        if self.attributes:
            if self.attributes[0] == 'all':
                self.state_log.log(state)
            else:
                self.state_log.log(state,self.attributes)

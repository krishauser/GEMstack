from __future__ import annotations
from dataclasses import asdict
from ...state import AllState, MissionEnum
from ..component import Component
from ...utils.loops import TimedLooper
from ...utils import settings,config
from ...utils.serialization import serialize
from ...utils.logging import Logfile
from .logging import VehicleBehaviorLogger,AllStateLogger,LogReplay
import json
import time
import datetime
import os
import importlib
import io
import contextlib
from typing import Dict,Tuple,Set,List,Optional

EXECUTION_PREFIX = "Execution:"
EXECUTION_VERBOSITY = 1

# Define the computation graph
COMPONENTS = None 
COMPONENT_ORDER = None
COMPONENT_SETTINGS = None

def normalize_computation_graph(components : list) -> List[Dict]:
    normalized_components = []
    for c in components:
        if isinstance(c,str):
            normalized_components.append({c:{'inputs':[],'outputs':[]}})
        else:
            assert isinstance(c,dict), "Component {} is not a string or dict".format(c)
            assert len(c) == 1, "Component {} dict has more than one key".format(c)
            k = list(c.keys())[0]
            v = c[k]
            assert isinstance(v,dict), "Component {} value is not a string or dict".format(c)
            if 'inputs' not in v:
                v['inputs'] = []
            elif isinstance(v['inputs'],str):
                v['inputs'] = [v['inputs']]
            elif v['inputs'] is None:
                v['inputs'] = []
            if 'outputs' not in v:
                v['outputs'] = []
            elif isinstance(v['outputs'],str):
                v['outputs'] = [v['outputs']]
            elif v['outputs'] is None:
                v['outputs'] = []
            normalized_components.append({k:v})
    return normalized_components

def load_computation_graph():
    """Loads the computation graph from settings[run.computation_graph.components]
    and sets global variables COMPONENTS, COMPONENT_ORDER, and COMPONENT_SETTINGS."""
    global COMPONENTS, COMPONENT_ORDER, COMPONENT_SETTINGS
    COMPONENTS = normalize_computation_graph(settings.get('run.computation_graph.components'))
    COMPONENT_ORDER = [list(c.keys())[0] for c in COMPONENTS]
    COMPONENT_SETTINGS = dict(list(c.items())[0] for c in COMPONENTS)

def import_module_dynamic(module_name, parent_module=None):
    if parent_module is not None:
        full_path = parent_module + '.' + module_name
    else:
        full_path = module_name
    return importlib.import_module(full_path)


def make_class(config_info, component_module, parent_module=None, extra_args = None):
    """Creates an object from a config_info dictionary or string.

    Args:
        config_info: either a str with format module.classname or a dict
            with keys 'type', 'args' (optional), and 'module' (optional).
        component_module: name of the the module to import classes
            of this type from.
        parent_module: if not None, the parent module to import from.
        extra_args: if provided, a dict of arguments to send to the component's
            constructor.
    
    Returns:
        Component: instance of named class
    """
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


def validate_components(components : Dict[str,ComponentExecutor], provided : List = None):
    """Checks whether the defined components match the known computation graph"""
    state = asdict(AllState.zero())
    if provided is None:
        provided = set()
    else:
        provided = set(provided)
    provided_all = False
    for k in COMPONENT_ORDER:
        if k not in components:
            continue
        possible_inputs = COMPONENT_SETTINGS[k]['inputs']
        required_outputs = COMPONENT_SETTINGS[k]['outputs']

        c = components[k]
        inputs = c.c.state_inputs()
        for i in inputs:
            if i == 'all':
                assert possible_inputs == ['all'], "Component {} inputs are not provided by previous components".format(k)
            else:
                assert provided_all or i in provided, "Component {} input {} is not provided by previous components".format(k,i)
                if i not in state:
                    print(EXECUTION_PREFIX,"Component {} input {} does not exist in AllState object".format(k,i))
                if possible_inputs != ['all']:
                    assert i in possible_inputs, "Component {} is not supposed to receive input {}".format(k,i)
        outputs = c.c.state_outputs()
        for o in required_outputs:
            if o == 'all':
                assert outputs == ['all'], "Component {} outputs are not provided by previous components".format(k)
            else:
                assert o in outputs, "Component {} doesn't output required output {}".format(k,o)
        for o in outputs:
            if 'all' != o:
                provided.add(o)
                if o not in state:
                    print(EXECUTION_PREFIX,"Component {} output {} does not exist in AllState object".format(k,o))
            else:
                provided_all = True
    for k,c in components.items():
        print(k,c.c.__class__.__name__)
        assert k in COMPONENT_SETTINGS, "Component {} is not known".format(k)
    return list(provided)


class ComponentExecutor:
    """Polls for whether a component should be updated, and reads/writes
    inputs / outputs to the AllState object."""
    def __init__(self,c : Component, essential : bool = True):
        self.c = c
        self.essential = essential
        self.print_stdout = True
        self.print_stderr = False
        self.stdout_log_file = None
        self.stderr_log_file = None
        self.inputs = c.state_inputs()
        self.output = c.state_outputs()
        self.last_update_time = None
        self.next_update_time = None
        rate = c.rate()
        self.dt = 1.0/rate if rate is not None else 0.0
        self.num_overruns = 0
        self.overrun_amount = 0.0
        self.do_update = None
    
    def healthy(self):
        return self.c.healthy()

    def start(self):
        self.c.initialize()

    def stop(self):
        self.c.cleanup()
        if isinstance(self.stdout_log_file,io.TextIOWrapper):
            self.stdout_log_file.close()
            self.stdout_log_file = None
        if isinstance(self.stderr_log_file,io.TextIOWrapper):
            self.stderr_log_file.close()
            self.stderr_log_file = None

    def update(self, t : float, state : AllState):
        if self.next_update_time is None or t >= self.next_update_time:
            self.update_now(t,state)
            self.last_update_time = t
            if self.next_update_time is None:
                self.next_update_time = t + self.dt
            else:
                self.next_update_time += self.dt
            if self.next_update_time < t and self.dt > 0:
                if EXECUTION_VERBOSITY >= 1:
                    print(EXECUTION_PREFIX,"Component {} is running behind, overran dt by {} seconds".format(self.c,self.dt,t-self.next_update_time))
                self.num_overruns += 1
                self.overrun_amount += t - self.next_update_time
                self.next_update_time = t + self.dt
            return True
        if EXECUTION_VERBOSITY >= 3:
            print(EXECUTION_PREFIX,"Component",self.c,"not updating at time",t,", next update time is",self.next_update_time)
        return False

    def _do_update(self, t:float, *args):
        f = io.StringIO()
        g = io.StringIO()
        with contextlib.redirect_stdout(f):
            with contextlib.redirect_stderr(g):
                if self.do_update is not None:
                    res = self.do_update(*args)
                else:
                    res = self.c.update(*args)
        self.log_output(t, f.getvalue(),g.getvalue())
        return res

    def update_now(self, t:float, state : AllState):
        """Performs the updates for this component, without fussing with the polling scheduling"""
        if self.inputs == ['all']:
            args = (state,)
        else:
            args = tuple([getattr(state,i) for i in self.inputs])
        if EXECUTION_VERBOSITY >= 2:
            print(EXECUTION_PREFIX,"Updating",self.c.__class__.__name__)
        #capture stdout/stderr

        res = self._do_update(t, *args)
        #write result to state
        if res is not None:
            if len(self.output) > 1:
                assert len(res) == len(self.output), "Component {} output {} does not match expected length {}".format(self.c,self.output,len(self.output))
                for (k,v) in zip(self.output,res):
                    setattr(state,k, v)
                    setattr(state,k+'_update_time', t)
            else:
                setattr(state,self.output[0],  res)
                setattr(state,self.output[0]+'_update_time', t)

    def log_output(self,t,stdout,stderr):
        timestr = datetime.datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3]
        if stdout:
            lines = stdout.split('\n')
            if len(lines) > 0 and len(lines[-1])==0:
                lines = lines[:-1]
            if self.print_stdout:
                print("------ Component",self.c.__class__.__name__,"stdout ---------")
                for l in lines:
                    print("   ",l)
                print("-------------------------------------------")
            if isinstance(self.stdout_log_file,str):
                self.stdout_log_file = open(self.stdout_log_file,'w')
            if isinstance(self.stdout_log_file,io.TextIOWrapper):
                for l in lines:
                    self.stdout_log_file.write(timestr + ': ' + l + '\n')
        if stderr:
            lines = stderr.split('\n')
            if len(lines) > 0 and len(lines[-1])==0:
                lines = lines[:-1]
            if self.print_stderr:
                print("------ Component",self.c.__class__.__name__,"stderr ---------")
                for l in lines:
                    print("   ",l)
                print("-------------------------------------------")
            if isinstance(self.stderr_log_file,str):
                self.stderr_log_file = open(self.stderr_log_file,'w')
            if isinstance(self.stderr_log_file,io.TextIOWrapper):
                for l in lines:
                    self.stderr_log_file.write(timestr + ': ' + l + '\n')




class ExecutorBase:
    """Base class for a mission executor.  Handles the computation graph setup.
    Subclasses should implement begin(), update(), done(), and end() methods."""
    def __init__(self, vehicle_interface):
        self.vehicle_interface = vehicle_interface
        self.all_components = dict()  # type: Dict[str,ComponentExecutor]
        self.always_run_components = dict()      # type: Dict[str,ComponentExecutor]
        self.pipelines = dict()       # type: Dict[str,Tuple[Dict[str,ComponentExecutor],Dict[str,ComponentExecutor],Dict[str,ComponentExecutor]]]
        self.current_pipeline = 'drive'  # type: str
        self.state = None             # type: Optional[AllState]
        self.replayed_components = dict()  # type Dict[str,str]
        self.log_folder = None        # type: Optional[str]
        self.logged_components = set() # type: Set[str]
        self.behavior_log = None
        self.run_metadata = dict()    # type: dict
        self.run_metadata['pipelines'] = []
        self.run_metadata['events'] = []
        self.run_metadata['exit_reason'] = 'unknown'
        self.last_loop_time = time.time()

    def begin(self):
        """Override me to do any initialization.  The vehicle will have
        already been started and sensors will have been validated."""
        pass

    def update(self, state : AllState) -> Optional[str]:
        """Override me to implement mission and pipeline switching logic.
        
        Returns the name of the next pipeline to run, or None to continue the current pipeline"""
        return None

    def done(self):
        """Override me to implement mission completion logic."""
        return False

    def end(self):
        """Override me to do any mission cleanup.  This will be called before
        the vehicle is stopped."""
        pass

    def make_component(self, config_info, component_name, parent_module=None, extra_args = None) -> ComponentExecutor:
        """Creates a component, caching the result.  See arguments of :func:`make_class`.

        If the component was marked as being a replayed component, will return an executor of a
        LogReplay object.
        """
        identifier = str((component_name,config_info))
        if identifier in self.all_components:
            return self.all_components[identifier]
        else:
            component = make_class(config_info,component_name,parent_module,extra_args)
            if not isinstance(component,Component):
                raise RuntimeError("Component {} is not a subclass of Component".format(component_name))
            if component_name in self.replayed_components:
                #replace behavior of class with the LogReplay class
                replay_folder = self.replayed_components[component_name]
                outputs = component.state_outputs()
                rate = component.rate()
                assert rate is not None and rate > 0, "Replayed component {} must have a positive rate".format(component_name)
                if EXECUTION_VERBOSITY >= 1:
                    print(EXECUTION_PREFIX,"Replaying component",component_name,"from log",replay_folder,"with outputs",outputs)
                component = LogReplay(outputs,
                                    os.path.join(replay_folder,'behavior_log.json'),
                                    rate=rate)
            if isinstance(config_info,dict) and config_info.get('multiprocess',False):
                #wrap component in a multiprocess executor.  TODO: not tested yet
                from .multiprocess_execution import MPComponentExecutor
                executor = MPComponentExecutor(component)
            else:
                executor = ComponentExecutor(component)
            if isinstance(config_info,dict):
                executor.essential = config_info.get('essential',True)
                if 'rate' in config_info:
                    executor.dt = 1.0/config_info['rate']
                executor.print_stderr = executor.print_stdout = config_info.get('print',True)
            if self.log_folder:
                executor.stdout_log_file = os.path.join(self.log_folder,component.__class__.__name__+'.stdout.log')
                executor.stderr_log_file = os.path.join(self.log_folder,component.__class__.__name__+'.stderr.log')
            self.all_components[identifier] = executor
            return executor
    
    def always_run(self, component_name, component: ComponentExecutor):
        """Adds a component the always-run set."""
        self.always_run_components[component_name] = component

    def add_pipeline(self,name : str, perception : Dict[str,ComponentExecutor], planning : Dict[str,ComponentExecutor], other : Dict[str,ComponentExecutor]):
        output = validate_components(perception)
        output = validate_components(planning, output)
        validate_components(other, output)
        self.pipelines[name] = (perception,planning,other)
        #TODO: set any custom do_update functions here

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

    def set_log_folder(self,folder : str):
        self.log_folder = folder
        #save meta.yaml
        self.run_metadata['start_time'] = time.time()
        self.run_metadata['start_time_human_readable'] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        import subprocess
        git_commit_id = subprocess.check_output(['git','rev-parse','HEAD'])
        self.run_metadata['git_commit_id'] = git_commit_id.decode('utf-8').strip()
        git_branch = subprocess.check_output(['git','rev-parse','--abbrev-ref','HEAD'])
        self.run_metadata['git_branch'] = git_branch.decode('utf-8').strip()
        for k,c in self.all_components.items():
            c.stdout_log_file = os.path.join(self.log_folder,c.c.__class__.__name__+'.stdout.log')
            c.stderr_log_file = os.path.join(self.log_folder,c.c.__class__.__name__+'.stderr.log')
        self.dump_log_metadata()
    
    def log_vehicle_interface(self,enabled=True):
        if enabled:
            if self.behavior_log is None:
                self.behavior_log = Logfile(os.path.join(self.log_folder,'behavior.json'),delta_format=True,mode='w')
            self.always_run('vehicle_behavior_logger',ComponentExecutor(VehicleBehaviorLogger(self.behavior_log,self.vehicle_interface)))
        else:
            raise NotImplementedError("Disabling vehicle interface logging not supported yet")
    
    def log_components(self,components : List[str]):
        if components:
            if self.behavior_log is None:
                self.behavior_log = Logfile(os.path.join(self.log_folder,'behavior.json'),delta_format=True,mode='w')
        self.logged_components = set(components)
    
    def log_state(self,state_attributes : List[str], rate : Optional[float]=None):
        log_fn = os.path.join(self.log_folder,'state.json')
        self.always_run('state_logger',ComponentExecutor(AllStateLogger(state_attributes,rate,log_fn,self.vehicle_interface)))

    def dump_log_metadata(self):
        if not self.log_folder:
            return
        import os
        from ...utils import config
        config.save_config(os.path.join(self.log_folder,'meta.yaml'),self.run_metadata)
    
    def event(self,event_description : str, event_print_string : str = None):
        """Logs an event to the metadata and prints a message to the console."""
        if event_print_string is None:
            event_print_string = event_description if event_description.endswith('.') else event_description + '.'
        self.run_metadata['events'].append({'time':time.time(),'vehicle_time':self.state.t,'description':event_description})
        if EXECUTION_VERBOSITY >= 1:
            print(EXECUTION_PREFIX,event_print_string)
        self.dump_log_metadata()

    def set_exit_reason(self, description):
        """Sets a main loop exit reason"""
        self.run_metadata['exit_reason'] = description
        self.dump_log_metadata()

    def run(self):
        """Main entry point.  Runs the mission execution loop."""
        #sanity checking
        if self.current_pipeline not in self.pipelines:
            print(EXECUTION_PREFIX,"Initial pipeline {} not found".format(self.current_pipeline))
            return
        #must have recovery pipeline
        if 'recovery' not in self.pipelines:
            print(EXECUTION_PREFIX,"'recovery' pipeline not found")
            return
        #did we ask to replay any components that don't exist in any pipelines?
        for c in self.replayed_components.keys():
            found = False
            for (name,(perception_components,planning_components,other_components)) in self.pipelines.items():
                if c in perception_components or c in planning_components or c in other_components:
                    found = True
                    break
            if not found:
                raise ValueError("Replay component",c,"not found in any pipeline")

        #start running components
        for k,c in self.all_components.items():
            c.start()

        #start running mission
        self.state = AllState.zero()
        self.state.mission.type = MissionEnum.IDLE
        
        validated = False
        try:
            validated = self.validate_sensors()
            if not validated:
                self.event("Sensor validation failed","Could not validate sensors, stopping components and exiting")
                self.set_exit_reason("Sensor validation failed")
        except KeyboardInterrupt:
            self.event("Ctrl+C interrupt during sensor validation","Could not validate sensors, stopping components and exiting")
            self.set_exit_reason("Sensor validation failed")
            if time.time() - self.last_loop_time > 0.5:
                print(EXECUTION_PREFIX,"A component may have hung. Traceback:")
                import traceback
                traceback.print_exc()

        if validated:
            self.begin()
        while validated:
            self.run_metadata['pipelines'].append({'time':time.time(),'vehicle_time':self.state.t,'name':self.current_pipeline})
            self.dump_log_metadata()
            try:
                if EXECUTION_VERBOSITY >= 1:
                    print(EXECUTION_PREFIX,"Executing pipeline {}".format(self.current_pipeline))
                next = self.run_until_switch()
                if next is None:
                    #done
                    if self.run_metadata['exit_reason'] == 'unknown':
                        self.set_exit_reason("normal exit")
                    break
                if next not in self.pipelines:
                    if EXECUTION_VERBOSITY >= 1:
                        print(EXECUTION_PREFIX,"Pipeline {} not found, switching to recovery".format(next))
                    next = 'recovery'
                if self.current_pipeline == 'recovery' and next == 'recovery':
                    if EXECUTION_VERBOSITY >= 1:
                        print(EXECUTION_PREFIX)
                        print("************************************************")
                        print("   Recovery pipeline is not working, exiting!   ")
                        print("************************************************")
                    self.set_exit_reason("recovery pipeline not working")
                    break
                self.current_pipeline = next
                if not self.validate_sensors(1):
                    self.event("Sensors in desired pipeline {} are not working, switching to recovery".format(self.current_pipeline))
                    self.current_pipeline = 'recovery'
            except KeyboardInterrupt:
                if self.current_pipeline == 'recovery':
                    if EXECUTION_VERBOSITY >= 1:
                        print(EXECUTION_PREFIX)
                        print("************************************************")
                        print("    Ctrl+C interrupt during recovery, exiting!  ")
                        print("************************************************")
                    self.set_exit_reason("Ctrl+C interrupt during recovery")
                    break
                self.current_pipeline = 'recovery'
                self.event("Ctrl+C pressed, switching to recovery mode")
                if time.time() - self.last_loop_time > 0.5:
                    if EXECUTION_VERBOSITY >= 1:
                        print(EXECUTION_PREFIX,"A component may have hung. Traceback:")
                    import traceback
                    traceback.print_exc()
        if validated:
            self.end()
            #done with mission
            self.event("Mission execution ended","Done with mission execution, stopping components and exiting")

        for k,c in self.all_components.items():
            if EXECUTION_VERBOSITY >= 2:
                print("Stopping",k)
            c.stop()
        
        if self.behavior_log:
            self.behavior_log.close()
            self.behavior_log = None
        print("Done with execution loop")


    def validate_sensors(self,numsteps=None):
        """Verifies sensors are working"""
        (perception_components,planning_components,other_components) = self.pipelines[self.current_pipeline]
        components = list(perception_components.values()) + list(self.always_run_components.values())
        dt_min = min([c.dt for c in components if c.dt != 0.0])
        looper = TimedLooper(dt_min,name="main executor")
        sensors_working = False
        num_attempts = 0
        t0 = time.time()
        next_print_time = t0 + 1.0
        while looper and not sensors_working:
            self.last_loop_time = time.time()
            self.update_components(perception_components,self.state)
            sensors_working = all([c.healthy() for c in perception_components.values()])

            self.update_components(self.always_run_components,self.state,force=True)
            always_run_working = all([c.healthy() for c in self.always_run_components.values()])
            if not always_run_working:
                if EXECUTION_VERBOSITY >= 1:
                    print(EXECUTION_PREFIX,"Always-run components not working, ignoring")

            num_attempts += 1
            if numsteps is not None and num_attempts >= numsteps:
                return False
            if time.time() > next_print_time:
                if EXECUTION_VERBOSITY >= 1:
                    print(EXECUTION_PREFIX,"Waiting for sensors to be healthy...")
                next_print_time += 1.0
        return True

    def run_until_switch(self):
        """Runs a pipeline until a switch is requested."""
        if self.current_pipeline == 'recovery':        
            self.state.mission.type = MissionEnum.RECOVERY_STOP
        
        (perception_components,planning_components,other_components) = self.pipelines[self.current_pipeline]
        components = list(perception_components.values()) + list(planning_components.values()) + list(other_components.values()) + list(self.always_run_components.values())
        dt_min = min([c.dt for c in components if c.dt != 0.0])
        looper = TimedLooper(dt_min,name="main executor")
        while looper and not self.done():
            self.state.t = self.vehicle_interface.time()
            self.last_loop_time = time.time()
            self.update_components(perception_components,self.state)
            #check for faults
            for name,c in perception_components.items():
                if not c.healthy():
                    if c.essential and self.current_pipeline != 'recovery':
                        if EXECUTION_VERBOSITY >= 1:
                            print(EXECUTION_PREFIX,"Sensor %s not working, entering recovery mode"%(name,))
                        return 'recovery'
                    else:
                        if EXECUTION_VERBOSITY >= 1:
                            print(EXECUTION_PREFIX,"Warning, sensor %s not working, ignoring"%(name,))
            
            next_pipeline = self.update(self.state)
            if next_pipeline is not None and next_pipeline != self.current_pipeline:
                print(EXECUTION_PREFIX,"update() requests to switch to pipeline {}".format(next_pipeline))
                return next_pipeline

            self.update_components(planning_components,self.state)
            #check for faults
            for name,c in planning_components.items():
                if not c.healthy():
                    if c.essential and self.current_pipeline != 'recovery':
                        if EXECUTION_VERBOSITY >= 1:
                            print(EXECUTION_PREFIX,"Planner %s not working, entering recovery mode"%(name,))
                        return 'recovery'
                    else:
                        if EXECUTION_VERBOSITY >= 1:
                            print(EXECUTION_PREFIX,"Warning, planner %s not working, ignoring"%(name,))

            self.update_components(other_components,self.state)
            for name,c in other_components.items():
                if not c.healthy():
                    if c.essential and self.current_pipeline != 'recovery':
                        if EXECUTION_VERBOSITY >= 1:
                            print(EXECUTION_PREFIX,"Other component %s not working, entering recovery mode"%(name,))
                        return 'recovery'
                    else:
                        if EXECUTION_VERBOSITY >= 1:
                            print(EXECUTION_PREFIX,"Warning, other component %s not working"%(name,))

            self.update_components(self.always_run_components,self.state,force=True)
            for name,c in self.always_run_components.items():
                if not c.healthy():
                    if c.essential and self.current_pipeline != 'recovery':
                        if EXECUTION_VERBOSITY >= 1:
                            print(EXECUTION_PREFIX,"Always-run component %s not working, entering recovery mode"%(name,))
                        return 'recovery'
                    else:
                        if EXECUTION_VERBOSITY >= 1:
                            print(EXECUTION_PREFIX,"Warning, always-run component %s not working"%(name,))


        #self.done() returned True
        return None


    def update_components(self, components : Dict[str,ComponentExecutor], state : AllState, now = False, force = False):
        """Updates the components and performs necessary logging.
        
        If now = True, all components are run regardless of polling state.

        If force = False, only components listed in COMPONENT_ORDER are run. 
        Otherwise, all components in `components` are run in arbitrary order.
        """
        t = state.t
        if force:
            order = list(components.keys())
        else:
            order = []
            for k in COMPONENT_ORDER:
                if k in components:
                    order.append(k)
        for k in order:
            updated = False
            if now:
                components[k].update_now(t,state)
                updated = True
            else:
                updated = components[k].update(t,state)
            #log component output if necessary
            if updated and k in self.logged_components:
                if len(components[k].output)!=0:
                    self.behavior_log.log(state, components[k].output, t)


class StandardExecutor(ExecutorBase):
    def begin(self):
        try:
            import rospy
            rospy.init_node('GEM executor')
        except (ImportError,ModuleNotFoundError):
            if settings.get('run.mode','hardware') == 'simulation':
                print(EXECUTION_PREFIX,"Warning, ROS not found, but simulation mode requested")
            else:
                print(EXECUTION_PREFIX,"Error, ROS not found on system.")
                raise
    
    def done(self):
        if self.current_pipeline == 'recovery':
            if self.vehicle_interface.last_reading is not None and \
                abs(self.vehicle_interface.last_reading.speed) < 1e-3:
                if EXECUTION_VERBOSITY >= 1:
                    print(EXECUTION_PREFIX,"Vehicle has stopped, exiting execution loop.")
                return True
        return False

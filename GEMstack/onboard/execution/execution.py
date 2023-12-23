from dataclasses import asdict
from ...state import AllState, MissionEnum
from ..component import Component
from ...utils.loops import TimedLooper
from ...utils import settings
from ...utils.serialization import serialize
from ...utils.logging import Logfile
import json
import time
import os
from typing import Dict,Tuple,Set,List,Optional

EXECUTION_PREFIX = "Execution:"

# Define the computation graph

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

COMPONENTS = normalize_computation_graph(settings.get('run.computation_graph.components'))
COMPONENT_ORDER = [list(c.keys())[0] for c in COMPONENTS]
COMPONENT_SETTINGS = dict(list(c.items())[0] for c in COMPONENTS)


def validate_components(components : Dict[str,Component], provided : List = None):
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
        inputs = c.state_inputs()
        for i in inputs:
            if i == 'all':
                assert possible_inputs == ['all'], "Component {} inputs are not provided by previous components".format(k)
            else:
                assert provided_all or i in provided, "Component {} input {} is not provided by previous components".format(k,i)
                if i not in state:
                    print(EXECUTION_PREFIX,"Component {} input {} does not exist in AllState object".format(k,i))
                if possible_inputs != ['all']:
                    assert i in possible_inputs, "Component {} is not supposed to receive input {}".format(k,i)
        outputs = c.state_outputs()
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
        assert k in COMPONENT_SETTINGS, "Component {} is not known".format(k)
    return list(provided)


class ComponentExecutor:
    """Polls for whether a component should be updated, and reads/writes
    inputs / outputs to the AllState object."""
    def __init__(self,c : Component):
        self.c = c
        self.inputs = c.state_inputs()
        self.output = c.state_outputs()
        self.last_update_time = None
        self.next_update_time = None
        self.dt = 1.0/c.rate()
        self.num_overruns = 0
        self.overrun_amount = 0.0
        self.do_update = None
    
    def start(self):
        pass

    def stop(self):
        pass

    def update(self, t : float, state : AllState):
        if self.next_update_time is None or t >= self.next_update_time:
            self.update_now(t,state)
            self.last_update_time = t
            if self.next_update_time is None:
                self.next_update_time = t + self.dt
            else:
                self.next_update_time += self.dt
            if self.next_update_time < t:
                print(EXECUTION_PREFIX,"Component {} is running behind, overran dt by {} seconds".format(self.c,self.dt,t-self.next_update_time))
                self.num_overruns += 1
                self.overrun_amount += t - self.next_update_time
                self.next_update_time = t + self.dt
            return True
        #print("Component",self.c,"not updating at time",t,", next update time is",self.next_update_time)
        return False

    def _do_update(self, *args):
        if self.do_update is not None:
            res = self.do_update(*args)
        else:
            res = self.c.update(*args)
        return res

    def update_now(self, t:float, state : AllState):
        """Performs the updates for this component, without fussing with the polling scheduling"""
        if self.inputs == ['all']:
            args = (state,)
        else:
            args = tuple([getattr(state,i) for i in self.inputs])
        print(EXECUTION_PREFIX,"Updating",self.c.__class__.__name__)
        res = self._do_update(*args)
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




class ExecutorBase:
    """Base class for a mission executor.  Handles the computation graph setup.
    Subclasses should implement begin(), update(), done(), and end() methods."""
    def __init__(self, vehicle_interface):
        self.vehicle_interface = vehicle_interface
        self.pipelines = dict()       # type: Dict[str,Tuple[Dict[str,ComponentExecutor],Dict[str,ComponentExecutor],Dict[str,ComponentExecutor]]]
        self.current_pipeline = 'drive'  # type: str
        self.state = None             # type: Optional[AllState]
        self.log_folder = None        # type: Optional[str]
        self.logged_components = set() # type: Set[str]
        self.logged_state_attributes = []  # type: List[str]
        self.log_state_rate = None    # type: Optional[float]
        self.state_log = None
        self.behavior_log = None
        self.vehicle_log_t_last = None
        self.run_metadata = dict()    # type: dict
        self.last_loop_time = time.time()

    def begin(self):
        """Override me to do any initialization"""
        pass

    def update(self, state : AllState) -> Optional[str]:
        """Override me to implement mission and pipeline switching logic.
        
        Returns the name of the next pipeline to run, or None to continue the current pipeline"""
        return None

    def done(self):
        return False

    def end(self):
        pass

    def add_pipeline(self,name : str, perception : Dict[str,Component], planning : Dict[str,Component], other : Dict[str,Component]):
        output = validate_components(perception)
        output = validate_components(planning, output)
        validate_components(other, output)
        perception_component_state = dict((k,ComponentExecutor(c)) for (k,c) in perception.items())
        planning_component_state = dict((k,ComponentExecutor(c)) for (k,c) in planning.items())
        other_component_state = dict((k,ComponentExecutor(c)) for (k,c) in other.items())
        self.pipelines[name] = (perception_component_state,planning_component_state,other_component_state)
        #TODO: set any custom do_update functions here

    def set_log_folder(self,folder):
        self.log_folder = folder
        #save meta.yaml
        import subprocess
        self.run_metadata['start_time'] = time.time()
        git_commit_id = subprocess.check_output(['git','rev-parse','HEAD'])
        self.run_metadata['git_commit_id'] = git_commit_id.decode('utf-8').strip()
        git_branch = subprocess.check_output(['git','rev-parse','--abbrev-ref','HEAD'])
        self.run_metadata['git_branch'] = git_branch.decode('utf-8').strip()
        self.dump_log_metadata()
    
    def log_vehicle_interface(self,enabled=True):
        if enabled:
            if 'vehicle_interface' not in self.logged_components:
                self.logged_components.add('vehicle_interface')
        else:
            if 'vehicle_interface' in self.logged_components:
                self.logged_components.remove('vehicle_interface')\
    
    def log_components(self,components : List[str]):
        if 'vehicle_interface' in self.logged_components:
            self.logged_components = set(components) | set(['vehicle_interface'])
        else:
            self.logged_components = set(components)
        if components:
            if self.behavior_log is None:
                self.behavior_log = Logfile(os.path.join(self.log_folder,'behavior.json'),delta_format=True,mode='w')
    
    def log_state(self,state_attributes : List[str], rate : Optional[float]=None):
        self.logged_state_attributes = state_attributes[:]
        self.log_state_rate = rate
        if state_attributes:
            if self.state_log is None:
                self.state_log = Logfile(os.path.join(self.log_folder,'state.json'),delta_format=False,mode='w')

    def dump_log_metadata(self):
        import os
        from ...utils import config
        config.save_config(os.path.join(self.log_folder,'meta.yaml'),self.run_metadata)

    def run(self):
        if self.current_pipeline not in self.pipelines:
            print(EXECUTION_PREFIX,"Pipeline {} not found".format(self.current_pipeline))
            return
        if 'recovery' not in self.pipelines:
            print(EXECUTION_PREFIX,"'recovery' pipeline not found")
            return

        for (name,(perception_components,planning_components,other_components)) in self.pipelines.items():
            for (k,c) in perception_components.items():
                c.start()
            for (k,c) in planning_components.items():
                c.start()
            for (k,c) in other_components.items():
                c.start()

        #start running mission
        self.state = AllState.zero()
        self.state.mission.type = MissionEnum.IDLE
        
        validated = False
        try:
            self.validate_sensors()
            validated = True
        except KeyboardInterrupt:
            print(EXECUTION_PREFIX,"Could not validate sensors, stopping components and exiting")
            if time.time() - self.last_loop_time > 0.5:
                print(EXECUTION_PREFIX,"A component may have hung. Traceback:")
                import traceback
                traceback.print_exc()

        if validated:
            self.begin()
        while validated:
            try:
                print(EXECUTION_PREFIX,"Executing pipeline {}".format(self.current_pipeline))
                next = self.run_until_switch()
                if next is None:
                    #done
                    break
                if next not in self.pipelines:
                    print(EXECUTION_PREFIX,"Pipeline {} not found, switching to recovery".format(next))
                    next = 'recovery'
                if self.current_pipeline == 'recovery' and next == 'recovery':
                    print(EXECUTION_PREFIX)
                    print("************************************************")
                    print("   Recovery pipeline is not working, exiting!   ")
                    print("************************************************")
                    break
                self.current_pipeline = next
                if not self.validate_sensors(1):
                    print(EXECUTION_PREFIX,"Sensors in desired pipeline {} are not working, switching to recovery".format(self.current_pipeline))
                    self.current_pipeline = 'recovery'
            except KeyboardInterrupt:
                if self.current_pipeline == 'recovery':
                    print(EXECUTION_PREFIX)
                    print("************************************************")
                    print("    Ctrl+C interrupt during recovery, exiting!  ")
                    print("************************************************")
                    break
                self.current_pipeline = 'recovery'
                print(EXECUTION_PREFIX,"Ctrl+C pressed, switching to recovery mode.")
                if time.time() - self.last_loop_time > 0.5:
                    print(EXECUTION_PREFIX,"A component may have hung. Traceback:")
                    import traceback
                    traceback.print_exc()
        if validated:
            self.end()
            #done with mission
            print(EXECUTION_PREFIX,"Done with mission execution, stopping components and exiting")

        for (name,(perception_components,planning_components,other_components)) in self.pipelines.items():
            for (k,c) in perception_components.items():
                c.stop()
            for (k,c) in planning_components.items():
                c.stop()
            for (k,c) in other_components.items():
                c.stop()
        
        if self.state_log:
            self.state_log.close()
            self.state_log = None
        if self.behavior_log:
            self.behavior_log.close()
            self.behavior_log = None

    def validate_sensors(self,numsteps=None):
        #verify sensors are working
        (perception_components,planning_components,other_components) = self.pipelines[self.current_pipeline]
        dt_min = min([c.dt for c in perception_components.values()])
        if self.log_state_rate is not None:
            dt_min = min(dt_min,1.0/self.log_state_rate)
        looper = TimedLooper(dt_min,name="main executor")
        sensors_working = False
        num_attempts = 0
        t0 = time.time()
        next_print_time = t0 + 1.0
        while looper and not sensors_working:
            self.last_loop_time = time.time()
            self.update_components(perception_components,self.state)
            sensors_working = all([c.c.healthy() for c in perception_components.values()])
            num_attempts += 1
            if numsteps is not None and num_attempts >= numsteps:
                return False
            if time.time() > next_print_time:
                print(EXECUTION_PREFIX,"Waiting for sensors to be healthy...")
                next_print_time += 1.0
        return True

    def run_until_switch(self):
        if self.current_pipeline == 'recovery':        
            self.state.mission.type = MissionEnum.RECOVERY_STOP
        
        (perception_components,planning_components,other_components) = self.pipelines[self.current_pipeline]
        dt_min = min([c.dt for c in perception_components.values()] + [c.dt for c in planning_components.values()] + [c.dt for c in other_components.values()])
        if self.log_state_rate is not None:
            dt_min = min(dt_min,1.0/self.log_state_rate)
        looper = TimedLooper(dt_min,name="main executor")
        while looper and not self.done():
            self.state.t = self.vehicle_interface.time()
            self.last_loop_time = time.time()
            self.update_components(perception_components,self.state)
            #check for faults
            sensors_working = all([c.c.healthy() for c in perception_components.values()])
            if not sensors_working:
                print(EXECUTION_PREFIX,"Sensors not working, entering recovery mode")
                return 'recovery'
            
            next_pipeline = self.update(self.state)
            if next_pipeline is not None and next_pipeline != self.current_pipeline:
                print(EXECUTION_PREFIX,"update() requests to switch to pipeline {}".format(next_pipeline))
                return next_pipeline

            self.update_components(planning_components,self.state)
            #check for faults
            planners_working = all([c.c.healthy() for c in planning_components.values()])
            if not planners_working:
                print(EXECUTION_PREFIX,"Planners not working, entering recovery mode")
                return 'recovery'

            self.update_components(other_components,self.state)
            others_working = all([c.c.healthy() for c in other_components.values()])
            if not others_working:
                print(EXECUTION_PREFIX,"Other items not working, ignoring")

        #self.done() returned True
        return None


    def update_components(self, components : Dict[str,ComponentExecutor], state : AllState, now = False):
        """Updates the components and performs necessary logging.
        
        If now = True, all components are run regardless of polling state.
        """
        t = state.t
        for k in COMPONENT_ORDER:
            if k in components:
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
        if 'vehicle_interface' in self.logged_components:
            if state.t != self.vehicle_log_t_last:
                collection = {'vehicle_interface_command':self.vehicle_interface.last_command,
                            'vehicle_interface_reading':self.vehicle_interface.last_reading}
                self.behavior_log.log(collection,t=t)
                self.vehicle_log_t_last = state.t
        #log state if necessary
        if self.logged_state_attributes:
            if self.logged_state_attributes[0] == 'all':
                self.state_log.log(state)
            else:
                self.state_log.log(state,self.logged_state_attributes)


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
                print(EXECUTION_PREFIX,"Vehicle has stopped, exiting execution loop.")
                return True
        return False

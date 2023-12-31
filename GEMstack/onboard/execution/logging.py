from ..component import Component
from ...utils import serialization,logging
from typing import List
import time

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
    def __init__(self,attributes,rate,log_fn,vehicle_interface):   
        self._rate = rate     
        self.attributes = attributes
        self.vehicle_interface = vehicle_interface
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

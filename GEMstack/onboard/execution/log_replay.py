from ...utils import serialization
from ..component import Component
from typing import List
import time

class LogReplay(Component):
    """Substitutes the output of a component with replayed data from a log file.
    """
    def __init__(self, vehicle_interface, outputs : List[str], log_file : str, rate : float = 10.0):
        self.vehicle_interface = vehicle_interface
        self.outputs = outputs
        self.logfn = log_file
        self._rate = rate
        self.logfile = open(log_file,'r') 
        self.logfile_delta_format = True
        self.log_start_time = None
        self.next_item = None
        self.next_item_time = None
        self.start_time = None
    
    def rate(self):
        return self._rate
   
    def state_outputs(self):
        return self.outputs

    def update(self):
        t = self.vehicle_interface.time()
        if self.start_time == None:
            self.start_time = t
        if self.logfile == None:
            return
        if self.log_start_time is None:
            try:
                while self.next_item is None:
                    self.read_next()
            except IOError:
                self.logfile.close()
                self.logfile = None
            self.log_start_time = self.next_item_time

        if self.next_item_time - self.log_start_time > t - self.start_time:
            #not yet the time to read the next item
            return
        #return the next item
        res = [self.next_item.get(o,None) for o in self.outputs]
        #advance to the next item in the log
        try:
            while self.next_item_time - self.log_start_time < t - self.start_time:
                self.read_next()
        except IOError:
            self.logfile.close()
            self.logfile = None
        if len(self.outputs)==1:
            return res[0]
        if all(v is None for v in res):
            return None
        return res

    def read_next(self):
        line = self.logfile.readline()
        if line == '':
            raise IOError("End of log file")
        msg = serialization.deserialize_collection(line[:-1])
        if self.logfile_delta_format:
            #assumed to be of the form {'time':t,ITEM1:{...},ITEM2:{...}}
            assert 'time' in msg
            if self.next_item is None:
                self.next_item = {}
            for o in self.outputs:
                if o in msg:
                    self.next_item[o] = msg[o]
                    self.next_item_time = msg['time']
            if self.next_item_time is None:
                self.next_item = None
        else:
            #assumed to be state dictionaries of the form {ITEM1:VAL, ITEM2:VAL, ITEM1_update_time:t, ITEM2_update_time:t}
            self.next_item = msg
            self.next_item_time = None
            for o in self.outputs:
                if o+'_update_time' in msg:
                    v = msg[o+'_update_time']
                    if self.next_item_time is None:
                        self.next_item_time = v
                    else:
                        self.next_item_time = max(self.next_item_time,v)

from .serialization import deserialize,serialize,deserialize_collection,serialize_collection
from typing import Union,Tuple

class Logfile:
    """A log file of serializable collections that can be read or written.

    There are two forms of log files supported.  The first is a delta format, where
    each line is a dictionary of the form ``{'time':t,ITEM1:{...},ITEM2:{...}}``. 
    
    The second is a state format, where each line is a dictionary of the form
    ``{ITEM1:VAL, ITEM2:VAL, ITEM1_update_time:t, ITEM2_update_time:t}``.
    """
    def __init__(self, filename : str, delta_format: bool, mode='w'):
        self.filename = filename
        try:
            self.file = open(filename,mode)
        except IOError:
            print("Error opening log file",filename)
            self.file = None
        self.delta_format = delta_format
        self.mode = mode
        
        self.eof = False
        self.next_item = None
        self.cumulative_item = None
        self.last_read_time = None
        self.next_item_time = None
        self.start_time = None
        self.time_index = None
    
    def __nonzero__(self):
        return self.file is not None and not self.eof 

    def log(self, message, fields=None, t : float = None) -> None:
        """Logs a message to the log file.
        
        Arguments:
            message: a dict or instance of a serializable class registered in
                the utils.serialization module.
            fields (list, optional): if provided, then it specifies a list of
                keys to extract from the message dict / object.  If not
                provided, then the entire message is logged.
            t (float, optional): if provided, then it specifies the time to
                log the message at.  If not provided, then the message is
                logged at the current time.
        """
        if self.mode != 'w':
            raise RuntimeError("Logfile is not open for writing")
        if fields is not None:
            if isinstance(message,dict):
                new_message = {k:message[k] for k in fields}
                if self.delta_format:
                    new_message['time'] = message.get('time',t)
                else:
                    for k in fields:
                        new_message[k+'_update_time'] = message.get(k+'_update_time',t)
                message = new_message
            else:
                new_message = {k:getattr(message,k) for k in fields}
                if self.delta_format:
                    if t is None:
                        raise ValueError("Need to provide time for delta format")
                    new_message['time'] = t
                else:
                    for k in fields:
                        new_message[k+'_update_time'] = getattr(message,k+'_update_time')
                message = new_message
        else:
            if self.delta_format:
                if t is None:
                    raise ValueError("Need to provide time for delta format")
                if not isinstance(message,dict):
                    message = serialize(message,dict)['data']
                message['time'] = t
                
        if isinstance(message,dict):
            self.file.write(serialize_collection(message))
        else:
            self.file.write(serialize(message)['data'])
        self.file.write('\n')
    
    def read(self,
             duration_to_advance : float = None,
             duration_from_start : float = None,
             absolute_time : float = None,
             cumulative = False) -> Union[list,Tuple[dict,list]]:
        """Reads the next item(s) from the log file. 
        
        - If `duration_to_advance` is given, then it will read this amount of
            seconds from the current read time or from the start if no item has
            been read yet.
        - If `duration_from_start` is given, then it will read until this 
            amount of seconds from the start of the log file.  This time cannot
            be before the time of the last-read item.
        - If `absolute_time` is given, then it will read the items until this
            absolute time.  This time cannot be before the time of the last-
            read item.
        
        Arguments:
            duration_to_advance (float, optional): the amount of time to
                advance from the current read time.
            duration_from_start (float, optional): the amount of time to
                advance from the start of the log file.
            absolute_time (float, optional): the absolute time to advance to.
            cumulative (bool, optional): if True, then all of the updated
                items up to the  given time will be returned as a dictionary
                in the first return argument.
        
        Returns:
            list or tuple: if `cumulative` is False, then just returns a list
                of the messages read from the log. If `cumulative` is True,
                then returns a tuple (cumulative_item, messages) where 
                `cumulative_item` is a dictionary of the items that have been
                updated up to the given time.
        """
        if self.mode != 'r':
            raise RuntimeError("Logfile is not open for reading")
        if self.time_index is None:
            #initialize the time index
            try:
                self._read_next()
            except IOError:
                if not self.eof:
                    print("WARNING: empty log file")
                self.eof = True
                if cumulative:
                    return {},[]
                return []
            self.start_time = self.next_item_time
            self.last_read_time = self.start_time
            self.time_index = self.start_time
        next_t = 0
        if duration_to_advance is not None:
            if duration_to_advance < 0:
                raise ValueError("Can't advance backwards in time")
            next_t = self.time_index + duration_to_advance
        elif duration_from_start is not None:
            next_t = self.start_time + duration_from_start
            if next_t < self.time_index:
                raise ValueError("Can't advance backwards in time")
        elif absolute_time is not None:
            next_t = absolute_time
            if next_t < self.time_index:
                raise ValueError("Can't advance backwards in time")
        else:
            raise ValueError("Need to provide a time to advance to")
        msgs = []
        while self.next_item_time <= next_t:
            self.last_read_time = self.next_item_time
            msgs.append(self.next_item)
            try:
                self._read_next()
            except IOError:
                #end of file
                self.eof = True
                break
        self.time_index = next_t
        if cumulative:
            return self.cumulative_item.copy(),msgs
        else:
            return msgs

    def _read_next(self):
        line = self.file.readline()
        if line == '':
            raise IOError("End of log file")
        msg = deserialize_collection(line[:-1])
        if self.cumulative_item is None:
            self.cumulative_item = {}
        if self.delta_format:
            #assumed to be of the form {'time':t,ITEM1:{...},ITEM2:{...}}
            assert 'time' in msg, "Invalid message read from log file"
            self.next_item = msg
            self.next_item_time = msg['time']
            for o in msg:
                if o != 'time':
                    self.cumulative_item[o] = msg[o]
        else:
            #assumed to be state dictionaries of the form {ITEM1:VAL, ITEM2:VAL, ITEM1_update_time:t, ITEM2_update_time:t}
            self.next_item = msg
            self.next_item_time = None
            for o,v in msg.items():
                if o.endswith('_update_time'):
                    if self.next_item_time is None:
                        self.next_item_time = v
                    else:
                        self.next_item_time = max(self.next_item_time,v)
                self.cumulative_item[o] = v

    def close(self):
        """Cleanly closes the log file."""
        self.file.close()
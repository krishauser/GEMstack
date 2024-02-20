from multiprocessing import Process, Queue
import queue
from ..component import Component
from .execution import ComponentExecutor
import time
import sys
import traceback

MAX_QUEUE_SIZE = 10
#wait this many seconds before timing out on a queue get
UPDATE_TIMEOUT = 5.0   

class MPComponentExecutor(ComponentExecutor):
    """A component executor that uses a subprocess.

    TODO: print stdout / stderr according to the print config and
    logging settings.
    """
    def __init__(self, component : Component, *args, **kwargs):
        super(MPComponentExecutor, self).__init__(component, *args, **kwargs)
        self._in_queue = Queue()
        self._out_queue = Queue()
        self._process = None
        self.do_update = self._do_update
        self._times_put = []
        self._delay_count = 0
        self._num_delayed = 0
        self._amount_delayed = 0.0
    
    def __str__(self):
        return "MPComponentExecutor(%s)"%self.c.__class__.__name__

    def start(self):
        print("STARTING",self)
        config = {
            'print_stdout': self.print_stdout,
            'print_stderr': self.print_stderr,
        }
        if hasattr(self.c,'debugger'):
            delattr(self.c,'debugger') #can't be serialized via Pickle 
        self._process = Process(target=self._run, args=(self.c, self._in_queue, self._out_queue, config))
        try:
            self._process.start()
        except Exception as e:
            print("Unable to start process",self.c.__class__.__name__)
            print("Exception:",e)
            
            self._process = None
            raise RuntimeError("Error starting "+self.c.__class__.__name__+" process, usually a pickling error")
        res = self._out_queue.get()
        if isinstance(res,tuple) and isinstance(res[0],Exception):
            print("Traceback:")
            for line in res[1]:
                print(line)
            print("Exception:",res[0])
            
            self._process.join()
            self._process.close()
            self._process = None
            raise RuntimeError("Error initializing "+self.c.__class__.__name__)
        if res !='initialized':
            raise RuntimeError("Uh... didn't hear back from subprocess? "+self.c.__class__.__name__)

    def stop(self):
        if self._process and self._process.is_alive():
            print("STOPPING",self)
            self._in_queue.put('stop')
            self._process.join()
            self._process.close()
            self._process = None

    def _run(self, component : Component, inqueue : Queue, outqueue : Queue, config):
        #let parent process handle SIGINT
        import signal    
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        #initialize
        try:
            component.initialize()
        except Exception as e:
            print("Error initializing",component.__class__.__name__)
            outqueue.put((e,traceback.format_tb(e.__traceback__)))
            return
        outqueue.put('initialized')
        while True:
            #update
            try:
                data = inqueue.get(UPDATE_TIMEOUT)
            except queue.Empty:
                print("Timeout waiting for data from parent process",component.__class__.__name__)
                break

            if data == 'stop':
                print("Parent process requested stop",component.__class__.__name__)
                break
            
            try:
                res = component.update(*data)
                outqueue.put(res)
            except KeyboardInterrupt:
                return
            except Exception as e:
                print("Error updating",component.__class__.__name__)
                outqueue.put((e,traceback.format_tb(e.__traceback__)))
                break

        try:
            component.cleanup()
        except Exception as e:
            print("Error cleaning up",component.__class__.__name__)
            outqueue.put((e,traceback.format_tb(e.__traceback__)))
        print("Exiting process",component.__class__.__name__)
        
    def _do_update(self, t, *args):
        if not self.healthy():
            return None
        if self._in_queue.qsize() < MAX_QUEUE_SIZE:
            self._in_queue.put(args)
            self._times_put.append(time.time())
        else:
            print(self,"Warning: queue is full, dropping data")
        if not self._out_queue.empty():
            t_put = self._times_put.pop(0)
            if self._delay_count > 1:
                self._num_delayed += 1
                self._amount_delayed += time.time() - t_put
            self._delay_count = 0
            res = self._out_queue.get()
            if isinstance(res,tuple) and isinstance(res[0],Exception):
                print("Error in",self.c.__class__.__name__)
                print("Traceback:")
                for line in res[1]:
                    print(line)
                print("Exception:",res[0])
                
                self._process.join()
                self._process.close()
                self._process = None
                return None
            return res
        else:
            self._delay_count += 1
            if self._delay_count*max(self.dt,0.1) > UPDATE_TIMEOUT:
                print("ERROR:",self,"seems to be frozen")
                self._process.terminate()
                self._process.close()
                self._process = None
        return None

    def healthy(self):
        return self._process is not None and self._process.is_alive()

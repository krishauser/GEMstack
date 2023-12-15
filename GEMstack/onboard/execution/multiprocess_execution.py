from multiprocessing import Process, Queue
from .execution import ComponentExecutor
import time

MAX_QUEUE_SIZE = 10

class MPComponentExecutor(ComponentExecutor):
    def __init__(self, component, *args, **kwargs):
        super(MPComponentExecutor, self).__init__(component, *args, **kwargs)
        self._in_queue = Queue()
        self._out_queue = Queue()
        self._process = Process(target=self._run, args=(self.c, self._in_queue, self._out_queue))
        self.do_update = self._do_update
        self._times_put = []
        self._delay_count = 0
        self._num_delayed = 0
        self._amount_delayed = 0.0

    def start(self):
        self._process.start()

    def stop(self):
        self._process.terminate()
        self._in_queue.put('stop')
        self._process.join()
        self._process = None

    def _run(self, component, inqueue, outqueue):
        while True:
            if not inqueue.empty():
                data = inqueue.get()
                if data == 'stop':
                    break
                res = component.update(data)
                outqueue.put(res)

    def _do_update(self, *args):
        if len(self._in_queue) < MAX_QUEUE_SIZE:
            self._in_queue.put(args)
            self._times_put.append(time.time())
        else:
            print("Warning: queue is full, dropping data")
        if not self._out_queue.empty():
            t_put = self._times_put.pop(0)
            if self._delay_count > 1:
                self._num_delayed += 1
                self._amount_delayed += time.time() - t_put
            self._delay_count = 0
            return self._out_queue.get()
        else:
            self._delay_count += 1
        return None

    def __del__(self):
        if self._process is not None:
            self.stop()
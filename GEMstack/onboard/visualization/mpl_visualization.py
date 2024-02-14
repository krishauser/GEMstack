from ...utils import mpl_visualization
from ..component import Component
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
from collections import deque

class MPLVisualization(Component):
    """Runs a matplotlib visualization at 10Hz. 
    
    If save_as is not None, saves the visualization to a file.
    """
    def __init__(self, rate : float = 10.0, save_as : str = None):
        self._rate = rate
        self.save_as = save_as
        self.anim = None
        self.writer = None
        self.num_updates = 0
        self.fig = None
        self.axs = None
        self.tstart = 0
        self.plot_t_range = 10
        self.plot_values = {}
        self.plot_events = {}

    def rate(self) -> float:
        return self._rate

    def state_inputs(self):
        return ['all']

    def initialize(self):
        matplotlib.use('TkAgg')
        if self.save_as is not None:
            print("Saving Matplotlib visualization to",self.save_as)
            if self.save_as.endswith('.gif'):
                self.writer = animation.PillowWriter(fps=int(self._rate))
            else:
                self.writer = animation.FFMpegWriter(fps=int(self._rate))
            self.writer.setup(plt.gcf(), self.save_as, dpi=100)
        plt.ion()
        # to run GUI event loop
        self.fig,self.axs = plt.subplots(1,2,figsize=(12,6))
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        plt.show(block=False)
        self.tstart = time.time()

    def on_close(self,event):
        print("PLOT CLOSED")
        plt.ioff()

    def debug(self, source, item, value):
        t = time.time() - self.tstart
        item = source+'.'+item
        if item not in self.plot_values:
            self.plot_values[item] = deque()
        plot = self.plot_values[item]
        self.plot_values[item].append((t,value))
        while t - plot[0][0] > self.plot_t_range:
            plot.popleft()

    def debug_event(self, source, event):
        t = time.time() - self.tstart
        event = source+'.'+event
        if event not in self.plot_events:
            self.plot_events[event] = deque()
        plot = self.plot_events[event]
        plot.append(t)
        while t - plot[0] > self.plot_t_range:
            plot.popleft()

    def update(self, state):
        if not plt.fignum_exists(self.fig.number):
            #plot closed
            return
        self.num_updates += 1
        self.debug("vehicle","velocity",state.vehicle.v)
        self.debug("vehicle","front wheel angle",state.vehicle.front_wheel_angle)
        time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(state.t))
        #frame=ObjectFrameEnum.CURRENT
        #state = state.to_frame(frame)
        xrange = [state.vehicle.pose.x - 10, state.vehicle.pose.x + 10]
        yrange = [state.vehicle.pose.y - 10, state.vehicle.pose.y + 10]
        #plot main visualization
        mpl_visualization.plot(state,title="Scene %d at %s"%(self.num_updates,time_str),xrange=xrange,yrange=yrange,show=False,ax=self.axs[0])
        #plot figure on axs[1]
        self.axs[1].clear()
        for k,v in self.plot_values.items():
            t = [x[0] for x in v]
            y = [x[1] for x in v]
            self.axs[1].plot(t,y,label=k)
        for i,(k,v) in enumerate(self.plot_events.items()):
            for t in v:
                self.axs[1].axvline(x=t,linestyle='--',color='C'+str(i),label=k)
        self.axs[1].set_xlabel('Time (s)')
        self.axs[1].legend()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

        if self.save_as is not None and self.writer is not None:
            try:
                self.writer.grab_frame()
            except IOError as e:
                if e.errno == 32: #broken pipe
                    self.writer = None
                    print("Movie write process terminated, saved Matplotlib visualization to",self.save_as)
                    return
                print(e)
                pass

    def cleanup(self):
        if self.writer:
            print("Saved Matplotlib visualization to",self.save_as)
            self.writer.finish()
            self.writer = None


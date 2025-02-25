from ...utils import mpl_visualization
from ..component import Component
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
from collections import deque
from ...state.agent import AgentEnum
import numpy as np

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

        # Separate vehicle and pedestrian tracking
        self.vehicle_plot_values = {}
        self.pedestrian_plot_values = {}
        self.vehicle_plot_events = {}
        self.pedestrian_plot_events = {}

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
        self.fig,self.axs = plt.subplots(1,3,figsize=(18,6))
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        plt.show(block=False)
        self.tstart = time.time()

    def on_close(self,event):
        print("PLOT CLOSED")
        plt.ioff()

    def debug(self, source, item, value):
        t = time.time() - self.tstart
        item = source+'.'+item
        # Determine which plot dict to use based on source
        if source.startswith('ped_'):
            target_dict = self.pedestrian_plot_values
        else:
            target_dict = self.vehicle_plot_values
            
        if item not in target_dict:
            target_dict[item] = deque()
        plot = target_dict[item]
        plot.append((t,value))
        while t - plot[0][0] > self.plot_t_range:
            plot.popleft()

    def debug_event(self, source, event):
        t = time.time() - self.tstart
        event = source+'.'+event
        target_dict = self.pedestrian_plot_events if source.startswith('ped_') else self.vehicle_plot_events
        
        if event not in target_dict:
            target_dict[event] = deque()
        plot = target_dict[event]
        plot.append(t)
        while t - plot[0] > self.plot_t_range:
            plot.popleft()

    def update(self, state):
        if not plt.fignum_exists(self.fig.number):
            return
        self.num_updates += 1
        
        # Vehicle metrics
        self.debug("vehicle", "velocity", state.vehicle.v)
        self.debug("vehicle", "front_wheel_angle", state.vehicle.front_wheel_angle)
        
        # Pedestrian metrics
        for agent_id, agent in state.agents.items():
            if agent.type == AgentEnum.PEDESTRIAN:
                # self.debug(f"ped_{agent_id}", "x", agent.pose.x)
                # self.debug(f"ped_{agent_id}", "y", agent.pose.y)
                self.debug(f"ped_{agent_id}", "velocity", np.linalg.norm(agent.velocity))  # Magnitude of resultant velocity
                self.debug(f"ped_{agent_id}", "yaw_rate", agent.yaw_rate)
        
        time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(state.t))
        #frame=ObjectFrameEnum.CURRENT
        #state = state.to_frame(frame)
        xrange = [state.vehicle.pose.x - 10, state.vehicle.pose.x + 10]
        yrange = [state.vehicle.pose.y - 10, state.vehicle.pose.y + 10]
        #plot main visualization
        mpl_visualization.plot(state,title="Scene %d at %s"%(self.num_updates,time_str),xrange=xrange,yrange=yrange,show=False,ax=self.axs[0])
        
        # Vehicle plot (axs[1])
        self.axs[1].clear()
        for k,v in self.vehicle_plot_values.items():
            t = [x[0] for x in v]
            y = [x[1] for x in v]
            self.axs[1].plot(t,y,label=k)
        self.axs[1].set_title('Vehicle Metrics')
        self.axs[1].set_xlabel('Time (s)')
        self.axs[1].legend()

        # Pedestrian plot (axs[2])
        self.axs[2].clear()
        for k,v in self.pedestrian_plot_values.items():
            t = [x[0] for x in v]
            y = [x[1] for x in v]
            self.axs[2].plot(t,y,label=k)
        self.axs[2].set_title('Pedestrian Metrics')
        self.axs[2].set_xlabel('Time (s)')
        self.axs[2].legend()

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


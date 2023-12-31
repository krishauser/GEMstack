from ...utils import mpl_visualization
from ..component import Component
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

class MPLVisualization(Component):
    """Runs a matplotlib visualization at 10Hz. 
    
    If save_as is not None, saves the visualization to a file.
    """
    def __init__(self, rate : float = 10.0, save_as : str = None):
        self._rate = rate
        self.save_as = save_as
        self.anim = None
        self.writer = None
        self.fig = None
        self.num_updates = 0

    def rate(self) -> float:
        return self._rate

    def state_inputs(self):
        return ['all']

    def initialize(self):
        if self.save_as is not None:
            print("Saving Matplotlib visualization to",self.save_as)
            if self.save_as.endswith('.gif'):
                self.writer = animation.PillowWriter(fps=int(self._rate))
            else:
                self.writer = animation.FFMpegWriter(fps=int(self._rate))
            self.writer.setup(plt.gcf(), self.save_as, dpi=100)
        plt.ion()
        # to run GUI event loop
        self.fig = plt.gcf()
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        plt.show(block=False)

    def on_close(self,event):
        print("PLOT CLOSED")
        plt.ioff()

    def update(self, state):
        if not plt.fignum_exists(self.fig.number):
            #plot closed
            return
        self.num_updates += 1
        time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(state.t))
        #frame=ObjectFrameEnum.CURRENT
        #state = state.to_frame(frame)
        xrange = [state.vehicle.pose.x - 10, state.vehicle.pose.x + 10]
        yrange = [state.vehicle.pose.y - 10, state.vehicle.pose.y + 10]
        mpl_visualization.plot(state,title="Scene %d at %s"%(self.num_updates,time_str),xrange=xrange,yrange=yrange,show=False)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

        if self.save_as is not None:
            self.writer.grab_frame()

    def cleanup(self):
        if self.writer:
            print("Saved Matplotlib visualization to",self.save_as)
            self.writer.finish()
            self.writer = None


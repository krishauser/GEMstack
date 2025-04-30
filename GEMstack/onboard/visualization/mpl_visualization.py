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
        
        # Pedestrian metrics and position debugging
        ped_positions = []
        for agent_id, agent in state.agents.items():
            if agent.type == AgentEnum.PEDESTRIAN:
                # Position logging
                ped_x, ped_y = agent.pose.x, agent.pose.y
                ped_positions.append((ped_x, ped_y))
                # Debug output every 10 updates
                if self.num_updates % 10 == 0:
                    print(f"Pedestrian {agent_id} position: ({ped_x:.2f}, {ped_y:.2f}), frame: {agent.pose.frame}")
                    # Calculate distance from vehicle
                    dist = np.sqrt((ped_x - state.vehicle.pose.x)**2 + (ped_y - state.vehicle.pose.y)**2)
                    print(f"Distance to vehicle: {dist:.2f} meters")
                
                # Track positions for plotting
                self.debug(f"ped_{agent_id}", "x", ped_x)
                self.debug(f"ped_{agent_id}", "y", ped_y)
                self.debug(f"ped_{agent_id}", "velocity", np.linalg.norm(agent.velocity))
                self.debug(f"ped_{agent_id}", "yaw_rate", agent.yaw_rate)
        
        time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(state.t))
        
        # Print coordinate frame information for debugging
        if self.num_updates % 10 == 0:
            print(f"Vehicle: pos=({state.vehicle.pose.x:.2f}, {state.vehicle.pose.y:.2f}), frame={state.vehicle.pose.frame}")
            if state.start_vehicle_pose:
                print(f"Start pose: pos=({state.start_vehicle_pose.x:.2f}, {state.start_vehicle_pose.y:.2f}), frame={state.start_vehicle_pose.frame}")
            
            if len(state.agents) > 0:
                print(f"Number of agents: {len(state.agents)}")
                
        # Determine a good plot range that includes both vehicle and pedestrians
        if len(ped_positions) > 0:
            # Start with vehicle position
            min_x, max_x = state.vehicle.pose.x, state.vehicle.pose.x
            min_y, max_y = state.vehicle.pose.y, state.vehicle.pose.y
            
            # Expand to include pedestrians
            for x, y in ped_positions:
                min_x = min(min_x, x)
                max_x = max(max_x, x)
                min_y = min(min_y, y)
                max_y = max(max_y, y)
            
            # Add margin - based on the content size
            size_x = max(50, max_x - min_x)
            size_y = max(50, max_y - min_y)
            margin_x = max(20, size_x * 0.2)  # at least 20m or 20% of content
            margin_y = max(20, size_y * 0.2)
            
            xrange = [min_x - margin_x, max_x + margin_x]
            yrange = [min_y - margin_y, max_y + margin_y]
        else:
            # Default range around vehicle if no pedestrians
            xrange = [state.vehicle.pose.x - 10, state.vehicle.pose.x + 10]
            yrange = [state.vehicle.pose.y - 10, state.vehicle.pose.y + 10]
        
        # Print xrange and yrange for debugging
        if self.num_updates % 10 == 0:
            print(f"Plot range: X {xrange}, Y {yrange}")
        
        # Try converting state to START frame for visualization
        try:
            if state.start_vehicle_pose is not None:
                state_start = state.to_frame(ObjectFrameEnum.START)
                print(f"Successfully converted state to START frame")
                
                # Log the conversion for comparison
                if len(state.agents) > 0 and self.num_updates % 10 == 0:
                    for agent_id, agent in state.agents.items():
                        # Get the agent in both frames
                        agent_start = state_start.agents.get(agent_id)
                        if agent_start:
                            print(f"Agent {agent_id} in ABSOLUTE: ({agent.pose.x:.2f}, {agent.pose.y:.2f})")
                            print(f"Agent {agent_id} in START: ({agent_start.pose.x:.2f}, {agent_start.pose.y:.2f})")
                
                # Try using the START frame for visualization
                mpl_visualization.plot(state_start, title=f"START Frame: Scene {self.num_updates}", 
                                      xrange=xrange, yrange=yrange, show=False, ax=self.axs[0])
            else:
                # Use original state if no start pose
                mpl_visualization.plot(state, title=f"Scene {self.num_updates} at {time_str}", 
                                      xrange=xrange, yrange=yrange, show=False, ax=self.axs[0])
        except Exception as e:
            print(f"Error converting to START frame: {str(e)}")
            # Fallback to the original state
            mpl_visualization.plot(state, title=f"Scene {self.num_updates} at {time_str}",
                                  xrange=xrange, yrange=yrange, show=False, ax=self.axs[0])
        
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


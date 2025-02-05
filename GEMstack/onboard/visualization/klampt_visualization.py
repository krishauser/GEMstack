from ..component import Component
from klampt import __version__ as klampt_version
from klampt import vis
from klampt.math import se3
from klampt import *
from ...state import AllState,ObjectFrameEnum,VehicleState
from ...mathutils.signal import OnlineLowPassFilter
from ...utils import klampt_visualization, settings
import time
import os
import math
import numpy as np

class KlamptVisualization(Component):
    """Runs a Klampt visualization.
     
    Runs at 20Hz by default. 
    """
    def __init__(self, vehicle_interface, rate : float = 20.0, save_as : str = None):
        self.vehicle_interface = vehicle_interface
        self._rate = rate
        self.save_as = save_as
        if save_as is not None:
            print("WARNING: automatic saving of KlamptVisualization to movie is not supported yet. You can use Ctrl+M to start / stop saving the movie")
        self.num_updates = 0
        self.last_yaw = None
        self.plot_values = {}
        self.plot_events = {}
        self.vfilter = OnlineLowPassFilter(1.2, 30, 4)
        self.last_v = 0.0

        self.world = WorldModel()
        fn = os.path.abspath(os.path.join(__file__,settings.get('vehicle.geometry.urdf_model')))
        if not self.world.loadFile(fn):
            print("Warning, could not load vehicle model from",fn)
            input("Press enter to continue.")
            self.vehicle = None
        else:
            self.vehicle = self.world.robot(0)
        fn = os.path.abspath(os.path.join(__file__,"../../../knowledge/vehicle/model/plane.off"))
        if not self.world.loadTerrain(fn):
            print("Warning, could not load plane model from",fn)
            input("Press enter to continue.")
        plane = self.world.terrain(0)
        tm =  plane.geometry().getTriangleMesh()
        v = tm.getVertices()
        v[:] *= 20.0  #+/- 100m
        #plane.geometry().getTriangleMesh().setVertices(v)
        v = plane.geometry().setTriangleMesh(tm)
        plane.appearance().setColor(0.7,0.7,0.7)
        plane.appearance().setTexgen(np.array([[0.5,0,0,0],[0,0.5,0,0]]))
        plane.appearance().setTexcoords2D([[-50,-50],[50,-50],[50,50],[-50,50]])
        
    def rate(self) -> float:
        return self._rate

    def state_inputs(self):
        return ['all']

    def initialize(self):
        vis.setWindowTitle("GEMstack visualization")
        vp = vis.getViewport()
        if klampt_version == '0.10.0':
            vp.controller.rot[1] = -0.15
            vp.controller.rot[2] = -math.pi/2
            vp.controller.dist = 30.0
            vp.resize(1280,720)
            vp.n = 0.1
            vp.f = 1000
        else:
            vp.camera.rot[1] = -0.15
            vp.camera.rot[2] = -math.pi/2
            vp.camera.dist = 30.0
            vp.w = 1280
            vp.h = 720
            vp.clippingplanes = (0.1,1000)
        vis.setViewport(vp)
        vis.add("vehicle_plane",self.world.terrain(0),hide_label=True)
        #note: show() takes over the interrupt handler and sets it to default, so we restore it
        import signal
        oldsig = signal.getsignal(signal.SIGINT)
        vis.show()
        signal.signal(signal.SIGINT,oldsig)
    
    def cleanup(self):
        print("klampt_visualization Cleanup")
        vis.show(False)
        vis.clear()
        vis.kill()
        self.vehicle = None
        self.world = None
    
    def debug(self, source, item, value):
        if source not in self.plot_values:
            #draw overlay plots
            vis.addPlot(source+"_plot")
            self.plot_values[source] = {}
        vis.logPlot(source+"_plot",item, value)

    def debug_event(self, source, event):
        if source not in self.plot_events:
            #draw overlay plots
            vis.addPlot(source+"_plot")
            self.plot_events[source] = {}
        vis.logPlotEvent(source+"_plot",event)

    def update(self, state):
        ground_truth_state = None
        if hasattr(self.vehicle_interface,'simulator'):
            #simulation mode, show the actual simulator state and show
            with self.vehicle_interface.thread_lock: 
                ground_truth_state = self.vehicle_interface.simulator.state().to_frame(ObjectFrameEnum.START,start_pose_abs = state.start_vehicle_pose)
        if not vis.shown():
            #plot closed
            return
        vis.lock()
        try:
            self.num_updates += 1
            self.debug("vehicle","velocity",state.vehicle.v)
            self.debug("vehicle","front wheel angle",state.vehicle.front_wheel_angle)
            self.debug("vehicle","accelerator",state.vehicle.accelerator_pedal_position)
            self.debug("vehicle","brake",state.vehicle.brake_pedal_position)
            time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(state.t))
            state_start = state.to_frame(ObjectFrameEnum.START) if state.start_vehicle_pose is not None else state.to_frame(state.vehicle.pose.frame)
            klampt_visualization.plot(state_start,title="Scene %d at %s"%(self.num_updates,time_str),ground_truth_vehicle=ground_truth_state,vehicle_model=self.vehicle,show=False)

            #update pose of the tracked vehicle for camera 
            tracked_vehicle = ground_truth_state if ground_truth_state is not None else state.vehicle
            if self.last_yaw is not None:
                vp = vis.getViewport()
                v = self.vfilter(tracked_vehicle.v)
                center_offset = 1.0
                lookahead = 4.0*v
                dx,dy = math.cos(tracked_vehicle.pose.yaw)*(lookahead+center_offset),math.sin(tracked_vehicle.pose.yaw)*(lookahead+center_offset)
                if klampt_version == '0.10.0':
                    vp.controller.tgt = [tracked_vehicle.pose.x+dx,tracked_vehicle.pose.y+dy,1.5]
                    vp.controller.rot[2] += tracked_vehicle.pose.yaw - self.last_yaw
                    vp.controller.dist += 5.0*(v - self.last_v)
                else:
                    vp.camera.tgt = [tracked_vehicle.pose.x+dx,tracked_vehicle.pose.y+dy,1.5]
                    vp.camera.rot[2] += tracked_vehicle.pose.yaw - self.last_yaw
                    vp.camera.dist += 5.0*(v - self.last_v)
                self.last_v = v
                vis.setViewport(vp)
            
            self.last_yaw = tracked_vehicle.pose.yaw
            vis.add("vehicle_plane",self.world.terrain(0),hide_label=True)
        finally:
            vis.unlock()

    def cleanup(self):
        vis.show(False)
        vis.kill()

from klampt import vis
from klampt.math import vectorops,so3,se3
from klampt.model.trajectory import Trajectory
from klampt import Geometry3D, GeometricPrimitive, TriangleMesh
import numpy as np
from . import settings
from ..state import ObjectFrameEnum,ObjectPose,PhysicalObject,VehicleState,VehicleGearEnum,Path,Obstacle,AgentState,AgentEnum,Roadgraph,RoadgraphLane,RoadgraphLaneEnum,RoadgraphCurve,RoadgraphCurveEnum,RoadgraphRegion,RoadgraphRegionEnum,RoadgraphSurfaceEnum,Trajectory,Route,SceneState,AllState

#KH: there is a bug on some system where the visualization crashes with an OpenGL error when drawing curves
#this is a workaround.  We really should find the source of the bug! Change to 30 if still not working
MAX_POINTS_IN_CURVE = 50

OBJECT_COLORS = {
    AgentEnum.CAR : (1,1,0,1),
    AgentEnum.PEDESTRIAN : (0,1,0,1),
    AgentEnum.BICYCLIST : (0,0,1,1),
    AgentEnum.MEDIUM_TRUCK : (1,0,1,1),
    AgentEnum.LARGE_TRUCK : (0,1,1,1),
    None: (0.7,0.7,0.7,1),
}

AUX_BBOX_COLOR = (0,0,1,1)

CURVE_TO_STYLE = {
    RoadgraphCurveEnum.LANE_BOUNDARY : {'color':(1,1,1,1),'width':2,'pointSize':0},
    RoadgraphCurveEnum.CURB : {'color':(0.5,0.5,0.5,1),'width':2,'pointSize':0},
    RoadgraphCurveEnum.CLIFF : {'color':(1,0,0,1),'width':2,'pointSize':0},
    RoadgraphCurveEnum.CROSSING_BOUNDARY : {'color':(1,1,1,1),'width':1,'pointSize':0},
    RoadgraphCurveEnum.PARKING_SPOT_BOUNDARY : {'color':(1,1,1,1),'width':1,'pointSize':0},
    RoadgraphCurveEnum.STOP_LINE : {'color':(1,1,1,1),'width':2,'pointSize':0},
    RoadgraphCurveEnum.WALL : {'color':(0,0,1,1),'width':2,'pointSize':0},
    None : {'color':(1,1,1,1),'width':1,'pointSize':0},
}

SURFACE_TO_STYLE = {
    RoadgraphSurfaceEnum.PAVEMENT : {'color':(0.5,0.5,0.5,0.2),'pointSize':0},
    RoadgraphSurfaceEnum.DIRT : {'color':(160/255.0,82/255.0,45/255.0,0.2),'pointSize':0},
    RoadgraphSurfaceEnum.GRASS : {'color':(50/255.0,255/255.0,50/255.0,0.2),'pointSize':0},
    RoadgraphSurfaceEnum.GRAVEL : {'color':(0.7,0.7,0.7,0.2),'pointSize':0},
    None: {'color':(1,0,0,0.2),'pointSize':0},
}

REGION_TO_STYLE = {
    RoadgraphRegionEnum.INTERSECTION : {'color':(0,1,0,0.5),'width':1,'pointSize':0},
    RoadgraphRegionEnum.PARKING_LOT : {'color':(0,0,1,0.5),'width':1,'pointSize':0},
    RoadgraphRegionEnum.CLOSED_COURSE : {'color':(1,0,0,0.5),'width':1,'pointSize':0},
    RoadgraphRegionEnum.VIRTUAL : {'color':(0,0,0,0.5),'width':1,'pointSize':0},
}

def plot_pose(name : str, pose : ObjectPose, axis_len=0.1, label=True):
    """Plots the pose in the given axes.  The coordinates
    of the pose are plotted in the pose's indicated frame."""
    R = pose.rotation()
    t = pose.translation()
    T = (so3.from_matrix(R),t)
    vis.add(name, T, length=axis_len, hide_label=(not label))
    if pose.frame == ObjectFrameEnum.START:
        vis.add(name+"_origin",t,color=(0,0,1,1),size=5,hide_label=True)
    elif pose.frame == ObjectFrameEnum.CURRENT:
        vis.add(name+"_origin",t,color=(1,0,0,1),size=5,hide_label=True)
    elif pose.frame in [ObjectFrameEnum.GLOBAL,ObjectFrameEnum.ABSOLUTE_CARTESIAN]:
        vis.add(name+"_origin",t,color=(0,1,0,1),size=5,hide_label=True)
    else:
        raise ValueError("Unknown frame %s" % pose.frame)

def plot_object(name : str, obj : PhysicalObject, type=None, axis_len=None, outline=True, solid=True, bbox=True, label=True):
    """Shows an object in the given axes.

    If axis_len is given, shows the object's pose with
    a coordinate frame of the given length.

    If outline is True, shows the object's outline.

    If bbox is True, shows the object's bounding box.
    """
    height = obj.dimensions[2]
    core_color = OBJECT_COLORS[type]
    bbox_color = AUX_BBOX_COLOR
    if label:
        #add a point at the object's origin
        vis.add(name,obj.pose.translation(),size=5,color=(0,0,0,1))
    if axis_len: 
        plot_pose(name+"_pose", obj.pose, axis_len=0)
    #plot bounding box
    R = obj.pose.rotation()
    t = obj.pose.translation()
    if bbox or (outline and obj.outline is None): 
        bounds = obj.bounds()
        (xmin,xmax),(ymin,ymax),(zmin,zmax) = bounds
        if not solid:
            corners = [[xmin,ymin,0.01],[xmin,ymax,0.01],[xmax,ymax,0.01],[xmax,ymin,0.01]]
            corners = [list(R.dot(c)+t) for c in corners]
            corners.append(corners[0])
            if not bbox:
                vis.add(name+"_bbox1",corners,width=1,color=core_color,pointSize=0,hide_label=True)
            else:
                vis.add(name+"_bbox1",corners,width=1,color=AUX_BBOX_COLOR,pointSize=0,hide_label=True)
            if height > 0:
                corners = [[xmin,ymin,height],[xmin,ymax,height],[xmax,ymax,height],[xmax,ymin,height]]
                corners = [list(R.dot(c)+t) for c in corners]
                corners.append(corners[0])
                if not bbox:
                    vis.add(name+"_bbox2",corners,width=1,color=core_color,pointSize=0,hide_label=True)
                else:
                    vis.add(name+"_bbox2",corners,width=1,color=AUX_BBOX_COLOR,pointSize=0,hide_label=True)
        else:
            prim = GeometricPrimitive()
            prim.setAABB([xmin,ymin,0],[xmax,ymax,height])
            g = Geometry3D(prim)
            g.setCurrentTransform(so3.from_matrix(R),t)
            vis.add(name+"_bbox",g,color=core_color[:3]+(0.5,),hide_label=True)

    #plot outline
    if outline and obj.outline:
        outline = [R.dot((p[0],p[1],0))+t for p in obj.outline]
        outline.append(outline[0])
        vis.add(name+"_outline1",outline,width=1,color=core_color,hide_label=True)
        if height > 0:
            outline = [R.dot((p[0],p[1],height))+t for p in obj.outline]
            outline.append(outline[0])
            vis.add(name+"_outline2",outline,width=1,color=core_color,hide_label=True)


def plot_vehicle(vehicle : VehicleState, vehicle_model=None, axis_len=1.0):
    """Plots the vehicle in the given axes.  The coordinates
    of the vehicle are plotted in the vehicle's indicated frame."""
    plot_object("vehicle",vehicle.to_object(), type=None, axis_len=0, solid=False, label=False)
    R = vehicle.pose.rotation()
    t = vehicle.pose.translation()
    T = (so3.from_matrix(R),t)
    vis.add("vehicle", T, length=axis_len, hide_label=True)

    xbounds,ybounds,zbounds = settings.get('vehicle.geometry.bounds')
    #plot velocity arrow
    R = vehicle.pose.rotation2d()
    t = np.array([vehicle.pose.x,vehicle.pose.y])
    v = R.dot([vehicle.v,0])
    front_pt = vehicle.pose.apply((xbounds[1],0.0))
    h = (vehicle.pose.z if vehicle.pose.z is not None else 0) + zbounds[1]
    vis.add("vehicle_velocity",[[front_pt[0],front_pt[1],h],[front_pt[0]+v[0],front_pt[1]+v[1],h]],width=2,color=(0,1,0,1),pointSize=0)

    #plot front wheel angles
    wheelbase = settings.get('vehicle.geometry.wheelbase')
    wheel_spacing = 0.8*settings.get('vehicle.geometry.width') / 2
    phi = vehicle.front_wheel_angle
    if vehicle_model:
        q = vehicle_model.getConfig()
        lwhindex = vehicle_model.link("left_steering_hinge_link").index
        rwhindex = vehicle_model.link("right_steering_hinge_link").index
        lwindex = vehicle_model.link("front_left_wheel_link").index
        rwindex = vehicle_model.link("front_right_wheel_link").index
        rlwindex = vehicle_model.link("rear_left_wheel_link").index
        rrwindex = vehicle_model.link("rear_right_wheel_link").index
        q[lwhindex] = phi
        q[rwhindex] = phi
        q[lwindex] += vehicle.v * 0.2
        q[rwindex] += vehicle.v * 0.2
        q[rlwindex] += vehicle.v * 0.2
        q[rrwindex] += vehicle.v * 0.2
        vehicle_model.setConfig(q)
    else:
        left_wheel_origin = t + R.dot([wheelbase,wheel_spacing])
        right_wheel_origin = t + R.dot([wheelbase,-wheel_spacing])
        wheel_width = 0.5  #meters
        wheel_offset = R.dot(np.array([np.cos(phi),np.sin(phi)]))*wheel_width*0.5
        vis.add("left_wheel",[vectorops.sub(left_wheel_origin,wheel_offset),vectorops.add(left_wheel_origin,wheel_offset)],color=(0,0,0,1),width=2)
        vis.add("right_wheel",[vectorops.sub(right_wheel_origin,wheel_offset),vectorops.add(right_wheel_origin,wheel_offset)],color=(0,0,0,1),width=2)

    #plot gear
    if vehicle.gear in [VehicleGearEnum.NEUTRAL,VehicleGearEnum.REVERSE,VehicleGearEnum.PARK]:
        if vehicle.gear == VehicleGearEnum.NEUTRAL:
            gear = 'N'
        elif vehicle.gear == VehicleGearEnum.REVERSE:
            gear = 'R'
        else:
            gear = 'P'
        vis.addText("gear",gear,position=(t[0],t[1],1.5),color=(0,0,0,1))
    
    #plot lights
    light_point_size = 4
    light_size = 0.15
    light_color = (1,1,0,1)
    turn_indicator_height = 0.7
    headlight_height = 0.6
    if vehicle.left_turn_indicator:
        lp = vehicle.pose.apply([xbounds[0],ybounds[0]+light_size,turn_indicator_height])
        vis.add("left_turn_indicator",list(lp),size=light_point_size,color=light_color)
    if vehicle.right_turn_indicator:
        lp = vehicle.pose.apply([xbounds[0],ybounds[1]-light_size,turn_indicator_height])
        vis.add("right_turn_indicator",list(lp),size=light_point_size,color=light_color)
    if vehicle.headlights_on:
        lp = vehicle.pose.apply([xbounds[1],ybounds[0]+light_size*2,headlight_height])
        vis.add("left_headlight",list(lp),size=light_point_size,color=light_color)
        lp = vehicle.pose.apply([xbounds[1],ybounds[1]-light_size*2,headlight_height])
        vis.add("right_headlight",list(lp),size=light_point_size,color=light_color)
    if vehicle_model is not None:
        if vehicle.brake_pedal_position > 0.1:
            vehicle_model.link('rear_right_stop_light_link').appearance().setColor(1,0,0,1)
            vehicle_model.link('rear_left_stop_light_link').appearance().setColor(1,0,0,1)
        else:
            vehicle_model.link('rear_right_stop_light_link').appearance().setColor(0.3,0,0,1)
            vehicle_model.link('rear_left_stop_light_link').appearance().setColor(0.3,0,0,1)

def plot_path(name : str, path : Path, color=(0,0,0), width=1):
    if len(path.points) > MAX_POINTS_IN_CURVE:  # downsample due to OpenGL error?
        vis.add(name,[list(p) for p in path.points[::len(path.points)//MAX_POINTS_IN_CURVE]],color=color,width=width)
    else:
        vis.add(name,[list(p) for p in path.points],color=color,width=width)

def plot_curve(name : str, curve : RoadgraphCurve, color=None, width=None):
    style = CURVE_TO_STYLE.get(curve.type,CURVE_TO_STYLE[None])
    if curve.crossable and curve.type == RoadgraphCurveEnum.LANE_BOUNDARY:
        #style['linestyle'] = '--'
        #TODO: how to indicate crossable lines?
        pass
    if color is not None:
        style['color'] = color
    if width is not None:
        style['width'] = width
    for i,seg in enumerate(curve.segments):
        if len(seg) > MAX_POINTS_IN_CURVE:  # downsample due to OpenGL error?
            vis.add(name+"_%d" % i,seg[::len(seg)//MAX_POINTS_IN_CURVE],**style)
        else:
            vis.add(name+"_%d" % i,seg,**style)

def plot_lane(name : str, lane : RoadgraphLane, on_route=False):
    if lane.surface != RoadgraphSurfaceEnum.PAVEMENT:
        style = SURFACE_TO_STYLE.get(lane.surface,SURFACE_TO_STYLE[None])
        outline = lane.outline()
        vis.add(name, outline, **style)
    if lane.left is not None:
        plot_curve(name+"_left", lane.left)
    if lane.right is not None:
        plot_curve(name+"_right", lane.right)

def plot_region(name : str, region : RoadgraphRegion, color=None, width=None):
    style = REGION_TO_STYLE.get(region.type,REGION_TO_STYLE[None])
    points = region.outline()
    pts = points + points[0]
    if color is not None:
        style['color'] = color
    if width is not None:
        style['width'] = width
    vis.add(name, [list(p) for p in pts], **style)

def plot_roadgraph(roadgraph : Roadgraph, route : Route = None):
    #plot lanes
    for k,l in roadgraph.lanes.items():
        if route is not None and k in route.lanes:
            plot_lane(k,l,on_route=True)
        else:
            plot_lane(k,l)
    for k,c in roadgraph.curves.items():
        plot_curve(k,c,color=(0,0,0,1))
    #plot intersections
    for k,r in roadgraph.regions.items():
        plot_region(k,r)
    #plot 
    for k,o in roadgraph.static_obstacles.items():
        plot_object(k,o)

def plot_scene(scene : SceneState, ground_truth_vehicle=None, vehicle_model = None, title = None, show=True):
    for i in list(vis.scene().items.keys()):
        if not i.startswith("vehicle"):
            if not isinstance(vis.scene().items[i],vis.VisPlot):
                vis.remove(i)
    #set plot range
    #TODO
    if vehicle_model is not None:
        vis.add("vehicle_model",vehicle_model)
        if ground_truth_vehicle is not None:
            xform = ground_truth_vehicle.to_object().pose.transform()
        else:
            xform = scene.vehicle.to_object().pose.transform()
        vehicle_model.link(0).setParentTransform(*se3.from_ndarray(xform))
        vehicle_model.setConfig(vehicle_model.getConfig())
    
    #plot roadgraph
    plot_roadgraph(scene.roadgraph,scene.route)
    #plot vehicle and objects
    plot_vehicle(scene.vehicle, vehicle_model)
    for k,a in scene.agents.items():
        plot_object(k,a,type=a.type)
    for k,o in scene.obstacles.items():
        plot_object(k,o)
    if title is None:
        if show:
            vis.add("title","Scene at t = %.2f" % scene.t, position=(10,10))
    else:
        vis.add("title",title)
    if show:
        vis.show()

def plot(state : AllState, ground_truth_vehicle = None, vehicle_model=None, title=None, show=True):
    plot_scene(state, ground_truth_vehicle=ground_truth_vehicle, vehicle_model=vehicle_model, title=title, show=show)
    if state.route is not None:
        plot_path("route",state.route,color=(1,0.5,0,1),width=2)
    if state.trajectory is not None:
        plot_path("trajectory",state.trajectory,color=(1,0,0,1),width=3)

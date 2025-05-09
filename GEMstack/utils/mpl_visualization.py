import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from . import settings
from ..state import ObjectFrameEnum,ObjectPose,PhysicalObject,VehicleState,VehicleGearEnum,Path,Obstacle,AgentState,Roadgraph,RoadgraphLane,RoadgraphLaneEnum,RoadgraphCurve,RoadgraphCurveEnum,RoadgraphRegion,RoadgraphRegionEnum,RoadgraphSurfaceEnum,Trajectory,Route,SceneState,AllState

CURVE_TO_STYLE = {
    RoadgraphCurveEnum.LANE_BOUNDARY : {'color':'k','linewidth':1,'linestyle':'-'},
    RoadgraphCurveEnum.CURB : {'color':'k','linewidth':2,'linestyle':'-'},
    RoadgraphCurveEnum.CLIFF : {'color':'r','linewidth':2,'linestyle':'-'},
    RoadgraphCurveEnum.CROSSING_BOUNDARY : {'color':'k','linewidth':1,'linestyle':'--'},
    RoadgraphCurveEnum.PARKING_SPOT_BOUNDARY : {'color':'k','linewidth':1,'linestyle':'-'},
    RoadgraphCurveEnum.STOP_LINE : {'color':'k','linewidth':1,'linestyle':':'},
    RoadgraphCurveEnum.WALL : {'color':'b','linewidth':2,'linestyle':'-'},
    None : {'color':'k','linewidth':1,'linestyle':'-'},
}

SURFACE_TO_STYLE = {
    RoadgraphSurfaceEnum.PAVEMENT : {'color':(0.5,0.5,0.5,0.2)},
    RoadgraphSurfaceEnum.DIRT : {'color':(160/255.0,82/255.0,45/255.0,0.2)},
    RoadgraphSurfaceEnum.GRASS : {'color':(50/255.0,255/255.0,50/255.0,0.2)},
    RoadgraphSurfaceEnum.GRAVEL : {'color':(0.7,0.7,0.7,0.2),'hatch':'oo'},
    None: {'color':(1,0,0,0.2)},
}

REGION_TO_STYLE = {
    RoadgraphRegionEnum.INTERSECTION : {'color':'g','linewidth':1,'linestyle':':'},
    RoadgraphRegionEnum.PARKING_LOT : {'color':'b','linewidth':1,'linestyle':':'},
    RoadgraphRegionEnum.CLOSED_COURSE : {'color':'r','linewidth':1,'linestyle':':'},
    RoadgraphRegionEnum.VIRTUAL : {'color':'k','linewidth':1,'linestyle':':'},
}

def plot_pose(pose : ObjectPose, axis_len=0.1, ax=None):
    """Plots the pose in the given axes.  The coordinates
    of the pose are plotted in the pose's indicated frame."""
    if ax is None:
        ax = plt.gca()
    R = pose.rotation2d()
    t = [pose.x,pose.y]
    x_ax = R.dot([axis_len,0])+t
    y_ax = R.dot([0,axis_len])+t
    ax.plot([pose.x,x_ax[0]],[pose.y,x_ax[1]],'r-')
    ax.plot([pose.x,y_ax[0]],[pose.y,y_ax[1]],'g-')
    if pose.frame == ObjectFrameEnum.START:
        ax.plot(pose.x,pose.y,'bo')
    elif pose.frame == ObjectFrameEnum.CURRENT:
        ax.plot(pose.x,pose.y,'ro')
    elif pose.frame in [ObjectFrameEnum.GLOBAL,ObjectFrameEnum.ABSOLUTE_CARTESIAN]:
        ax.plot(pose.x,pose.y,'go')
    else:
        raise ValueError("Unknown frame %s" % pose.frame)

def plot_object(obj : PhysicalObject, axis_len=None, outline=True, bbox=True, ax=None):
    """Shows an object in a 2D plot in the given axes.

    If axis_len is given, shows the object's pose with
    a coordinate frame of the given length.

    If outline is True, shows the object's outline.

    If bbox is True, shows the object's bounding box.
    """
    if ax is None:
        ax = plt.gca()
    if axis_len: 
        plot_pose(obj.pose, axis_len, ax)
    #plot bounding box
    R = obj.pose.rotation2d()
    t = [obj.pose.x,obj.pose.y]
    if bbox or (outline and obj.outline is None): 
        bounds = obj.bounds()
        (xmin,xmax),(ymin,ymax),(zmin,zmax) = bounds
        corners = [[xmin,ymin],[xmin,ymax],[xmax,ymax],[xmax,ymin]]
        corners = [R.dot(c)+t for c in corners]
        corners.append(corners[0])
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        if not bbox:
            ax.plot(xs,ys,'r-')
        else:
            ax.plot(xs,ys,'b-')
    #plot outline
    if outline and obj.outline:
        outline = [R.dot(p)+t for p in obj.outline]
        outline.append(outline[0])
        xs = [c[0] for c in outline]
        ys = [c[1] for c in outline]
        ax.plot(xs,ys,'r-')

def plot_vehicle(vehicle : VehicleState, axis_len=0.1, ax=None):
    """Plots the vehicle in the given axes.  The coordinates
    of the vehicle are plotted in the vehicle's indicated frame."""
    if ax is None:
        ax = plt.gca()
    plot_object(vehicle.to_object(), axis_len, ax=ax)

    #plot velocity arrow
    R = vehicle.pose.rotation2d()
    t = np.array([vehicle.pose.x,vehicle.pose.y])
    v = R.dot([vehicle.v,0])
    ax.arrow(t[0],t[1],v[0],v[1],head_width=0.05,head_length=0.1,color='g')

    #plot front wheel angles
    wheelbase = settings.get('vehicle.geometry.wheelbase')
    wheel_spacing = 0.8*settings.get('vehicle.geometry.width') / 2
    phi = vehicle.front_wheel_angle
    left_wheel_origin = t + R.dot([wheelbase,wheel_spacing])
    right_wheel_origin = t + R.dot([wheelbase,-wheel_spacing])
    wheel_width = 0.5  #meters
    wheel_offset = R.dot(np.array([np.cos(phi),np.sin(phi)]))*wheel_width*0.5
    ax.plot([left_wheel_origin[0]-wheel_offset[0],left_wheel_origin[0]+wheel_offset[0]],
            [left_wheel_origin[1]-wheel_offset[1],left_wheel_origin[1]+wheel_offset[1]],'k-',linewidth=2)
    ax.plot([right_wheel_origin[0]-wheel_offset[0],right_wheel_origin[0]+wheel_offset[0]],
            [right_wheel_origin[1]-wheel_offset[1],right_wheel_origin[1]+wheel_offset[1]],'k-',linewidth=2)

    #plot gear
    if vehicle.gear in [VehicleGearEnum.NEUTRAL,VehicleGearEnum.REVERSE,VehicleGearEnum.PARK]:
        if vehicle.gear == VehicleGearEnum.NEUTRAL:
            gear = 'N'
        elif vehicle.gear == VehicleGearEnum.REVERSE:
            gear = 'R'
        else:
            gear = 'P'
        ax.text(t[0],t[1],gear,ha='center',va='center',color='k')
    
    #plot lights
    light_size = 0.15
    light_color = 'y'
    xbounds,ybounds,zbounds = settings.get('vehicle.geometry.bounds')
    if vehicle.left_turn_indicator:
        lp = vehicle.pose.apply([xbounds[0]+light_size,ybounds[0]+light_size])
        ax.add_patch(patches.Circle(lp,radius=light_size,fc=light_color,ec=light_color,fill=True,zorder=10))
    if vehicle.right_turn_indicator:
        lp = vehicle.pose.apply([xbounds[0]+light_size,ybounds[1]-light_size])
        ax.add_patch(patches.Circle(lp,radius=light_size,fc=light_color,ec=light_color,fill=True,zorder=10))
    if vehicle.headlights_on:
        lp = vehicle.pose.apply([xbounds[1],ybounds[0]+light_size*2])
        ax.add_patch(patches.Circle(lp,radius=light_size,fc=light_color,ec=light_color,fill=True,zorder=10))
        lp = vehicle.pose.apply([xbounds[1],ybounds[1]-light_size*2])
        ax.add_patch(patches.Circle(lp,radius=light_size,fc=light_color,ec=light_color,fill=True,zorder=10))
    

def plot_path(path : Path, color='k', linewidth=1, linestyle='-', ax=None):
    if ax is None:
        ax = plt.gca()
    xs = [p[0] for p in path.points]
    ys = [p[1] for p in path.points]
    ax.plot(xs,ys,color=color,linewidth=linewidth,linestyle=linestyle)

def plot_curve(curve : RoadgraphCurve, color=None, linewidth=None, linestyle=None, ax=None):
    if ax is None:
        ax = plt.gca()
    style = CURVE_TO_STYLE.get(curve.type,CURVE_TO_STYLE[None])
    if curve.crossable and curve.type == RoadgraphCurveEnum.LANE_BOUNDARY:
        style['linestyle'] = '--'
    if color is not None:
        style['color'] = color
    if linewidth is not None:
        style['linewidth'] = linewidth
    if linestyle is not None:
        style['linestyle'] = linestyle
    for seg in curve.segments:
        xs = [p[0] for p in seg]
        ys = [p[1] for p in seg]
        ax.plot(xs,ys,**style)

def plot_lane(lane : RoadgraphLane, on_route=False, ax=None):
    if lane.surface != RoadgraphSurfaceEnum.PAVEMENT:
        style = SURFACE_TO_STYLE.get(lane.surface,SURFACE_TO_STYLE[None])
        outline = lane.outline()
        x = [p[0] for p in outline]
        y = [p[1] for p in outline]
        ax.fill(x,y,**style)
    if lane.left is not None:
        plot_curve(lane.left,ax=ax)
    if lane.right is not None:
        plot_curve(lane.right,ax=ax)

def plot_region(region : RoadgraphRegion, color=None, linewidth=None, linestyle=None, ax=None):
    if ax is None:
        ax = plt.gca()
    style = REGION_TO_STYLE.get(region.type,REGION_TO_STYLE[None])
    points = region.outline()
    xs = [p[0] for p in points] + [points[0][0]]
    ys = [p[1] for p in points] + [points[0][1]]
    if color is not None:
        style['color'] = color
    if linewidth is not None:
        style['linewidth'] = linewidth
    if linestyle is not None:
        style['linestyle'] = linestyle
    ax.plot(xs,ys,**style)

def plot_roadgraph(roadgraph : Roadgraph, route : Route = None, ax=None):
    if ax is None:
        ax = plt.gca()
    #plot lanes
    for k,l in roadgraph.lanes.items():
        if route is not None and k in route.lanes:
            plot_lane(l,on_route=True,ax=ax)
        else:
            plot_lane(l,ax=ax)
    for c in roadgraph.curves.values():
        plot_curve(c,color='k',ax=ax)
    #plot intersections
    for r in roadgraph.regions.values():
        plot_region(r,ax=ax)
    #plot 
    for k,o in roadgraph.static_obstacles.items():
        plot_object(o,ax=ax)

def plot_scene(scene : SceneState, xrange=None, yrange=None, ax=None, title = None, show=True):
    if ax is None:
        ax = plt.gca()
    ax.cla()
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    #set plot range
    if xrange is not None:
        if isinstance(xrange,(tuple,list)):
            ax.set_xlim(xrange[0],xrange[1])
        else:
            ax.set_xlim(-xrange*0.2,xrange*0.8)
    if yrange is not None:
        if isinstance(yrange,(tuple,list)):
            ax.set_ylim(yrange[0],yrange[1])
        else:
            ax.set_ylim(-yrange*0.5,yrange*0.5)
    #plot roadgraph
    plot_roadgraph(scene.roadgraph,scene.route,ax=ax)
    #plot vehicle and objects
    plot_vehicle(scene.vehicle,ax=ax)
    for k,a in scene.agents.items():
        plot_object(a,ax=ax)
    for k,o in scene.obstacles.items():
        plot_object(o,ax=ax)
    if title is None:
        if show:
            ax.set_title("Scene at t = %.2f" % scene.t)
    else:
        ax.set_title(title)
    if show:
        plt.show(block=False)

def plot(state : AllState, xrange=None, yrange=None,ax=None, title=None, show=True):
    if ax is None:
        ax = plt.gca()
    plot_scene(state, xrange=xrange, yrange=yrange, ax=ax, title=title, show=show)
    if state.route is not None:
        plot_path(state.route,color='k',linestyle='--',ax=ax)
    if state.trajectory is not None:
        plot_path(state.trajectory,color='r',linestyle='--',linewidth=2,ax=ax)

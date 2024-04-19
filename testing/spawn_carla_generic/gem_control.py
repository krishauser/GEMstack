#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

import std_msgs.msg

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from novatel_gps_msgs.msg import Inspva
from pacmod_msgs.msg import VehicleSpeedRpt
from pacmod_msgs.msg import SystemRptFloat
from pacmod_msgs.msg import PacmodCmd
from pacmod_msgs.msg import PositionWithSpeed
from ros_numpy import msgify # new import > sudo apt-get install ros-noetic-ros-numpy
import std_msgs
from cv_bridge import CvBridge

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import rospy

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except Exception:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
    return []


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):
        self.carla_world = carla_world
        self.sync = args.sync
        self.actor_role_name = args.rolename
        try:
            self.map = self.carla_world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.gnss_value = {}
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        rospy.init_node('manual_control')
        self.r = rospy.Rate(10) # 10hz
        self.rgb = rospy.Publisher('carla/front/rgb', Image, queue_size=10)
        self.gnss = rospy.Publisher('carla/gnss', Inspva, queue_size=10)
        self.lidar = rospy.Publisher('carla/top_lidar', PointCloud2, queue_size=10)
        self.depth = rospy.Publisher('/carla/front/depth', Image, queue_size=10)
        self.speed = rospy.Publisher('/carla/parsed_tx/vehicle_speed_rpt', VehicleSpeedRpt, queue_size=10)
        self.steer = rospy.Publisher('/carla/parsed_tx/steer_rpt', SystemRptFloat, queue_size=10)
        # watch out for restart method
        self.restart()
        self.carla_world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint_list = get_actor_blueprints(self.carla_world, self._actor_filter, self._actor_generation)
        if not blueprint_list:
            raise ValueError("Couldn't find any blueprints with the specified filters")
        
        blueprint = None
        for car in blueprint_list:
            if (car.id == "vehicle.mini.cooper_s_2021"):
                blueprint = car

        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('terramechanics'):
            blueprint.set_attribute('terramechanics', 'true')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.carla_world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.carla_world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.gnss_sensor.add_callback(self.publish_gnss_message)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.radar_sensor = RadarSensor(self.player)
        self.radar_sensor.add_callback(self.publish_gnss_message)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        self.camera_manager.set_fixed_rgb()
        self.camera_manager.set_fixed_lidar()
        self.camera_manager.set_fixed_depth()
        self.camera_manager.add_rgb_callback(self.publish_rgb_image_message)
        self.camera_manager.add_lidar_callback(self.publish_lidar_message)
        self.camera_manager.add_depth_callback(self.publish_depth_image_message)
        actor_type = get_actor_display_name(self.player)
        self.hud.add_vehicle_speed_callback(self.publish_vehicle_speed)
        self.hud.add_vehicle_steer_callback(self.publish_vehicle_steer)
        self.hud.notification(actor_type)

        if self.sync:
            self.carla_world.tick()
        else:
            self.carla_world.wait_for_tick()

    def publish_rgb_image_message(self, image):
        self.rgb.publish(image)

    def publish_depth_image_message(self, image):
        self.depth.publish(image)

    def publish_lidar_message(self, pc):
        self.lidar.publish(pc)

    def publish_vehicle_speed(self, speed):
        msg = VehicleSpeedRpt()
        msg.vehicle_speed = speed
        self.speed.publish(msg)

    def publish_vehicle_steer(self, steer):
        msg = SystemRptFloat()
        msg.output = steer
        self.steer.publish(msg)
        
    def publish_gnss_message(self, value) :
        for x in value:
            self.gnss_value[x] = value[x]
        # compose your gnss values here
        inspva_message = Inspva()
        inspva_message.latitude = self.gnss_value['lat'] if 'lat' in self.gnss_value else 0.0
        inspva_message.latitude = self.gnss_value['lon'] if 'lon' in self.gnss_value else 0.0
        inspva_message.height = 0.0
        inspva_message.pitch = self.gnss_value['pitch'] if 'pitch' in self.gnss_value else 0.0
        inspva_message.azimuth = self.gnss_value['yaw'] if 'yaw' in self.gnss_value else 0.0
        inspva_message.roll = self.gnss_value['roll'] if 'roll' in self.gnss_value else 0.0
        v = self.player.get_velocity()
        velocity = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
        angle = math.radians(inspva_message.azimuth)
        sin_theta = math.sin(angle)
        cos_theta = math.cos(angle)
        inspva_message.east_velocity =velocity * cos_theta
        inspva_message.north_velocity = velocity * sin_theta
        self.gnss.publish(inspva_message)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.carla_world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.carla_world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.fixed_front_rgb_sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            self.camera_manager.fixed_front_rgb_sensor, 
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()



# ==============================================================================
# -- GEMControl ----------------------------------------------------------------
# ==============================================================================

class GEMControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world):
        self._control = carla.VehicleControl()
        self._lights = carla.VehicleLightState.NONE
        self.world = world
        world.player.set_autopilot(False, 8002)
        world.player.set_light_state(self._lights)
        self._steer_cache = 0.0
        self._gear_cache = self._control.gear
        self.throttle = 0.0
        self.brake = 0.0
        self.state = "none"
        rospy.Subscriber('/carla/as_rx/shift_cmd', PacmodCmd, self.gear_shift_callback)
        rospy.Subscriber('/carla/as_rx/brake_cmd', PacmodCmd, self.brake_callback)
        rospy.Subscriber('/carla/as_rx/accel_cmd', PacmodCmd, self.acceleration_callback)
        rospy.Subscriber('/carla/as_rx/turn_cmd', PacmodCmd, self.turn_signal_callback)
        rospy.Subscriber('/carla/as_rx/steer_cmd', PositionWithSpeed, self.steer_callback)


    def step_ahead(self):
        if (self.state == "MOVE"):
            self._control.throttle = self.throttle
        if (self.state == "BRAKE"):
            self._control.brake = self.brake
        self.world.player.apply_control(self._control)

    def gear_shift_callback(self, message: PacmodCmd):
        if (self._gear_cache == 0 and message.ui16_cmd == message.SHIFT_FORWARD):
            self._control.gear = self._gear_cache + 1
            self._gear_cache = self._control.gear
            self.world.player.apply_control(self._control)

    def acceleration_callback(self, message: PacmodCmd):
        v = self.world.player.get_velocity()
        if (self._control.throttle < 0.1 and (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)) < 0.1):
            self.throttle = 1
        else:
            self.throttle = message.f64_cmd
        self.state = "MOVE"

    def brake_callback(self, message: PacmodCmd):
        self.state = "BRAKE"
        self.brake = message.f64_cmd

    def turn_signal_callback(self, message: PacmodCmd):
        current_lights = self._lights
        if (message.ui16_cmd == PacmodCmd.TURN_HAZARDS):
            current_lights ^= carla.VehicleLightState.LeftBlinker
            current_lights ^= carla.VehicleLightState.RightBlinker
        elif (message.ui16_cmd == PacmodCmd.TURN_LEFT):
            current_lights ^= carla.VehicleLightState.LeftBlinker
        elif (message.ui16_cmd == PacmodCmd.TURN_RIGHT):
            current_lights ^= carla.VehicleLightState.RightBlinker
        elif (message.ui16_cmd == PacmodCmd.TURN_NONE):
            current_lights ^= carla.VehicleLightState.NONE
        
        if current_lights != self._lights: # Change the light state only if necessary
            self._lights = current_lights
            self.world.player.set_light_state(carla.VehicleLightState(self._lights))

    def steer_callback(self, message: PositionWithSpeed):
        self._control.steer = round(message.angular_position, 1)
        self.world.player.apply_control(self._control)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        self.vehicle_speed_callbacks = []
        self.vehicle_steer_callbacks = []
        self._show_ackermann_info = False
        self._ackermann_control = carla.VehicleAckermannControl()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.carla_world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
            if self._show_ackermann_info:
                self._info_text += [
                    '',
                    'Ackermann Controller:',
                    '  Target speed: % 8.0f km/h' % (3.6*self._ackermann_control.speed),
                ]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))
        
        for cb in self.vehicle_speed_callbacks:
            cb(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))   

        for cb in self.vehicle_steer_callbacks:
            cb(c.steer)
        


    def add_vehicle_speed_callback(self, callback):
        if (callback not in self.vehicle_speed_callbacks):
            self.vehicle_speed_callbacks.append(callback)
        
    def add_vehicle_steer_callback(self, callback):
        if (callback not in self.vehicle_steer_callbacks):
            self.vehicle_steer_callbacks.append(callback)
        
    def show_ackermann_info(self, enabled):
        self._show_ackermann_info = enabled

    def update_ackermann_control(self, ackermann_control):
        self._ackermann_control = ackermann_control

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        self.callbacks = []
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude
        for cb in self.callbacks:
            value = {'lat': self.lat, 'lon' : self.lon}
            cb(value)
    
    def add_callback(self, callback):
        if (callback not in self.callbacks):
            self.callbacks.append(callback)


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._imu_callback(weak_self, sensor_data))

    @staticmethod
    def _imu_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.callbacks = []
        self.ctr = 0 # remove later
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._radar_callback(weak_self, radar_data))

    @staticmethod
    def _radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        self.ctr += 1
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))

        current_rot = radar_data.transform.rotation
        value = {'pitch': current_rot.pitch, 'roll' : current_rot.roll}
        for cb in self.callbacks:
            cb(value)
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

    def add_callback(self, callback):
        if (callback not in self.callbacks) :
            self.callbacks.append(callback)
# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.fixed_front_rgb_sensor = None
        self.fixed_front_rgb_sensor_name = 'sensor.camera.rgb'
        self.fixed_top_lidar_sensor = None
        self.fixed_top_lidar_sensor_name = 'sensor.lidar.ray_cast'
        self.fixed_front_depth_sensor = None
        self.fixed_front_depth_sensor_name = 'sensor.camera.depth'
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self.gamma_correction = gamma_correction
        
        self.rgb_callbacks = []
        self.lidar_callbacks = []
        self.depth_callbacks = []

        self.bridge = CvBridge() 
        
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        attachment = carla.AttachmentType

        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), attachment.SpringArmGhost),
        ]

        # these values are hardcoded based on gem_e4 information
        lidar_x_pos = 2.06
        lidar_z_pos = 2.305
        camera_x_pos = 1.33
        camera_z_pos = 1.855
        self.transform_index = 1
        self.fixed_front_rgb_info = [self.fixed_front_rgb_sensor_name, cc.Raw, 'Camera RGB', {}]
        self.fixed_front_rgb_transform = (carla.Transform(carla.Location(x=+camera_x_pos, y=+0.0*bound_y, z=camera_z_pos)), attachment.Rigid)
        self.fixed_front_depth_info = [self.fixed_front_depth_sensor_name, cc.Depth, 'Camera Depth (Raw)', {}]
        self.fixed_front_depth_transform = (carla.Transform(carla.Location(x=camera_x_pos, y=+0.0*bound_y, z=camera_z_pos)), attachment.Rigid)
        self.fixed_top_lidar_info = [self.fixed_top_lidar_sensor_name, None, 'Lidar (Ray-Cast)', {'range': '50'}]
        self.fixed_top_lidar_transform = (carla.Transform(carla.Location(x=lidar_x_pos, y=+0.0*bound_y, z=lidar_z_pos)), attachment.Rigid)


        self.sensors = [
            [self.fixed_front_rgb_sensor_name, cc.Raw, 'Camera RGB', {}],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
            item.append(bp)

        self.assign_agent_from_blue_print(self.fixed_front_rgb_info)
        self.assign_agent_from_blue_print(self.fixed_front_depth_info)

        bp = bp_library.find(self.fixed_top_lidar_info[0])
        self.lidar_range = 50
        for attr_name, attr_value in self.fixed_top_lidar_info[3].items():
            bp.set_attribute(attr_name, attr_value)
            if attr_name == 'range':
                self.lidar_range = float(attr_value)
        self.fixed_top_lidar_info.append(bp)

        self.index = None

    def assign_agent_from_blue_print(self, sensor_info):
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        bp = bp_library.find(sensor_info[0])
        bp.set_attribute('image_size_x', str(self.hud.dim[0]))
        bp.set_attribute('image_size_y', str(self.hud.dim[1]))
        if bp.has_attribute('gamma'):
            bp.set_attribute('gamma', str(self.gamma_correction))
        sensor_info.append(bp)

    def add_rgb_callback(self, callback):
        if (callback not in self.rgb_callbacks):
            self.rgb_callbacks.append(callback)
    
    def add_lidar_callback(self, callback):
        if (callback not in self.lidar_callbacks):
            self.lidar_callbacks.append(callback)
    
    def add_depth_callback(self, callback):
        if (callback not in self.depth_callbacks):
            self.depth_callbacks.append(callback)
        
    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def set_fixed_rgb(self):
        if self.fixed_front_rgb_sensor is not None:
            self.fixed_front_rgb_sensor.destroy()
        self.fixed_front_rgb_sensor = self._parent.get_world().spawn_actor(
            self.fixed_front_rgb_info[-1],
            self.fixed_front_rgb_transform[0],
            attach_to=self._parent,
            attachment_type=self.fixed_front_rgb_transform[1])
        # We need to pass the lambda a weak reference to self to avoid
        # circular reference.
        weak_self = weakref.ref(self)
        self.fixed_front_rgb_sensor.listen(lambda image: CameraManager._parse_image(weak_self, image,callback=True, caller=self.fixed_front_rgb_info[0]))
    
    def set_fixed_lidar(self):
        if self.fixed_top_lidar_sensor is not None:
            self.fixed_top_lidar_sensor.destroy()
        self.fixed_top_lidar_sensor = self._parent.get_world().spawn_actor(
            self.fixed_top_lidar_info[-1],
            self.fixed_top_lidar_transform[0],
            attach_to=self._parent,
            attachment_type=self.fixed_top_lidar_transform[1])
        # We need to pass the lambda a weak reference to self to avoid
        # circular reference.
        weak_self = weakref.ref(self)
        self.fixed_top_lidar_sensor.listen(lambda image: CameraManager._parse_image(weak_self, image,callback=True, caller=self.fixed_top_lidar_info[0]))

    def set_fixed_depth(self):
        if self.fixed_front_depth_sensor is not None:
            print('fixed sensor destroyed')
        self.fixed_front_depth_sensor = self._parent.get_world().spawn_actor(
            self.fixed_front_depth_info[-1],
            self.fixed_front_depth_transform[0],
            attach_to=self._parent,
            attachment_type=self.fixed_front_depth_transform[1])
        # We need to pass the lambda a weak reference to self to avoid
        # circular reference.
        weak_self = weakref.ref(self)
        self.fixed_front_depth_sensor.listen(lambda image: CameraManager._parse_image(weak_self, image,callback=True, caller=self.fixed_front_depth_info[0]))

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image, callback = False, caller = 'sensor.camera.rgb'):
        self = weak_self()
        if not self:
            return
        if not callback:
            if self.sensors[self.index][0].startswith('sensor.lidar'):
                points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
                points = np.reshape(points, (int(points.shape[0] / 4), 4))
                lidar_data = np.array(points[:, :2])
                lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
                lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
                lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
                lidar_data = lidar_data.astype(np.int32)
                lidar_data = np.reshape(lidar_data, (-1, 2))
                lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
                lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
                lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
                self.surface = pygame.surfarray.make_surface(lidar_img)
            elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
                # Example of converting the raw_data from a carla.DVSEventArray
                # sensor into a NumPy array and using it as an image
                dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                    ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', bool)]))
                dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
                # Blue is positive, red is negative
                dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
                self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
            elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'):
                image = image.get_color_coded_flow()
                array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
                array = np.reshape(array, (image.height, image.width, 4))
                array = array[:, :, :3]
                array = array[:, :, ::-1]
                self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            else:
                image.convert(self.sensors[self.index][1])
                array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
                array = np.reshape(array, (image.height, image.width, 4))
                array = array[:, :, :3]
                array = array[:, :, ::-1]
                self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        
        if (callback and caller == self.fixed_front_rgb_sensor_name) :
            image.convert(self.fixed_front_rgb_info[1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            for cb in self.rgb_callbacks:
                cb(self.bridge.cv2_to_imgmsg(array))
        elif (callback and caller == self.fixed_front_depth_sensor_name) :
            image.convert(self.fixed_front_depth_info[1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            for cb in self.depth_callbacks:
                cb(self.bridge.cv2_to_imgmsg(array))
        elif (callback and caller == self.fixed_top_lidar_sensor_name):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            for cb in self.lidar_callbacks:
                cb(self.lidar_points_to_pc2(points))

        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)
    
    def lidar_points_to_pc2(self, pc):
        header = std_msgs.msg.Header(frame_id='lidar', stamp=rospy.Time.now())
        pc_array = np.zeros(len(pc), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
        ])
        pc_array['x'] = pc[:, 0]
        pc_array['y'] = pc[:, 1]
        pc_array['z'] = pc[:, 2]
        pc_array['intensity'] = pc[:, 3]

        return msgify(PointCloud2, pc_array, stamp=header.stamp, frame_id=header.frame_id)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)

        sim_world = client.get_world()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            
            traffic_manager.set_synchronous_mode(True)

        if args.autopilot and not sim_world.get_settings().synchronous_mode:
            print("WARNING: You are currently in asynchronous mode and could "
                  "experience some issues with the traffic simulation")

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        hud = HUD(args.width, args.height)
        world = World(sim_world, hud, args)
        controller = GEMControl(world)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        while True:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYUP and ((event.key == K_ESCAPE) or (event.key == K_q and pygame.key.get_mods() & KMOD_CTRL))):
                    return
            world.tick(clock)
            world.render(display)
            controller.step_ahead()
            pygame.display.flip()

    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
            
#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Trying to reply tutorial ego.
# Major Task: Spawn a new vehicle, that starts moving with autopilot.
# A camera is set to the driver position seat.
# Goal: Try to move inside the world with HTC Vive.
# Do you feel motion sickness?
# Further research are needed...

import glob
import os
import sys

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

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
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
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)


    try:

        world = client.get_world()
        ego_vehicle = None
        ego_cam = None
        ego_col = None
        ego_lane = None
        ego_obs = None
        ego_gnss = None
        ego_imu = None

        # -----------
        # Trying to use tick to update driver image
        settings = world.get_settings()
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        # -----------

        # --------------
        # Start recording
        # --------------
        """
        client.start_recorder('~/tutorial/recorder/recording01.log')
        """

        # --------------
        # Spawn ego vehicle
        # --------------
        
        # ego_bp = world.get_blueprint_library().find('vehicle.tesla.cybertruck')
        ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name','ego')
        print('\nEgo role_name is set')
        ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
        ego_bp.set_attribute('color',ego_color)
        print('\nEgo color is set')

        ego_transform = carla.Transform(carla.Location(x=52, y=-6, z=2), carla.Rotation(yaw=180))
        ego_vehicle = world.spawn_actor(ego_bp,ego_transform)

        # --------------
        # Add a RGB camera sensor to ego vehicle. 
        # --------------
        cam_bp = None
        # find RGB cameras on the blueprint library
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        # Camera Attributes:
        cam_bp.set_attribute("image_size_x",str(2160))
        cam_bp.set_attribute("image_size_y",str(1200))
        cam_bp.set_attribute("fov",str(110))
        # Define Camera frame, by its transform, location and orientation wrt vehicle

        #TOWN 3
        cam_transform= carla.Transform(
               carla.Location(x=0.25, y=-0.45, z=1.2), carla.Rotation(pitch=-5))
        #TOWN 10
        # cam_transform= carla.Transform(
        #         carla.Location(x=-80, y=12, z=2), carla.Rotation(pitch=-5))
        # ego_cam = world.spawn_actor(cam_bp,cam_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.SpringArm)
        # ---
        # ego_camera.listen(image_queue.put)
        # ---
        # ego_cam.listen(lambda image: image.save_to_disk('~/tutorial/output/%.6d.jpg' % image.frame))

        # --------------
        # Collision sensor to ego vehicle. DELETED.
        # --------------
        # Add Lane invasion sensor to ego vehicle. DELETED.
        # --------------
        # Add Obstacle sensor to ego vehicle. DELETED.
        # --------------
        # Add GNSS sensor to ego vehicle. DELETED.
        # --------------
        # Add IMU sensor to ego vehicle. DELETED.

        # --------------
        # Enable autopilot for ego vehicle
        # --------------
        
        # ego_vehicle.set_autopilot(True)
        
        # --------------
        # Game loop. Prevents the script from finishing.
        # --------------
        while True:
            world_snapshot = world.wait_for_tick()
            # ---
            # world.tick()
            # image = image_queue.get()
            # ---
            
    finally:
        # --------------
        # Stop recording and destroy actors
        # --------------
        client.stop_recorder()
        if ego_vehicle is not None:
            if ego_cam is not None:
                ego_cam.stop()
                ego_cam.destroy()
            if ego_col is not None:
                ego_col.stop()
                ego_col.destroy()
            if ego_lane is not None:
                ego_lane.stop()
            if ego_obs is not None:
                ego_obs.stop()
                ego_obs.destroy()
            if ego_gnss is not None:
                ego_gnss.stop()
                ego_gnss.destroy()
            if ego_imu is not None:
                ego_imu.stop()
                ego_imu.destroy()
            ego_vehicle.destroy()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with tutorial_ego.')

#!/usr/bin/env python3

from drone_api import Drone_api
from math import pi

drone = Drone_api()
drone.start()
height = 0
while not drone.is_shutdown():
    height += 0.2
    drone.set_local_pose(z=height)
    drone.sleep(1)

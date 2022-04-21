#!/usr/bin/env python3

from drone_api import Drone_api

try:
    drone = Drone_api()
    drone.start()
    while not drone.is_shutdown():
        x = float(input('Enter x:'))
        y = float(input('Enter y:'))
        z = float(input('Enter z:'))
        yaw = float(input('Enter yaw:'))
        drone.set_local_pose(x, y, z, yaw)
        print()
except ValueError:
    pass

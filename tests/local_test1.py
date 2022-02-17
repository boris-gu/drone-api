#!/usr/bin/env python3

from drone_api import Drone_api

try:
    drone = Drone_api()
    drone.start()
    while True:
        if drone.is_shutdown():
            break
        x = float(input('Введите x:'))
        y = float(input('Введите y:'))
        z = float(input('Введите z:'))
        yaw = float(input('Введите yaw:'))
        drone.set_local_pose(x, y, z, yaw)
        print()
except ValueError:
    pass
drone.stop()

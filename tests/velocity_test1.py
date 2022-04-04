#!/usr/bin/env python3

from math import pi, cos, sin
from drone_api import Drone_api

speed = 2  # m/s
angles = [i/180*pi for i in range(0, 360, 5)] * 2
vel_x = [cos(i)*speed for i in angles]
vel_y = [sin(i)*speed for i in angles]
vel_z_up = 2
vel_z_down = -1

drone = Drone_api()
drone.start()
drone.sleep(3)
drone.set_velocity(0, 0, speed, 0)
drone.sleep(5)
drone.set_velocity(0, 0, 0, 0)
drone.sleep(10)
for i in range(len(vel_x)):
    if drone.is_shutdown():
        break
    drone.set_velocity(vel_x[i], vel_y[i], vel_z_up)
    drone.sleep(0.1)
drone.set_velocity(0, 0, 0, 0)
drone.sleep(10)
for i in range(len(vel_x)):
    if drone.is_shutdown():
        break
    drone.set_velocity(vel_x[i], vel_y[i], vel_z_down)
    drone.sleep(0.1)

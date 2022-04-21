#!/usr/bin/env python3

from drone_api import Drone_api
from math import pi

speed_z = 0.2
x_start = 0
y_start = 0

drone = Drone_api(0.2)
drone.start()
drone.set_local_pose(0, 0, 1)
while not drone.point_is_reached():
    print('OK')
    drone.sleep(0.1)
while not drone.is_shutdown():
    real_pose = drone.get_local_pose()
    speed_x = (x_start - real_pose[0])*2
    speed_y = (y_start - real_pose[1])*2
    drone.set_velocity(speed_x, speed_y, speed_z)
    drone.sleep(0.05)

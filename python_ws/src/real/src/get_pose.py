#!/usr/bin/env python3

from drone_api import Drone_api


def toFixed(numObj):
    return f'{numObj:.2f}'


drone = Drone_api()
drone.start()
while not drone.is_shutdown():
    pose = drone.get_local_pose()
    pose = list(map(toFixed, pose))
    print(pose)
    drone.sleep(0.5)

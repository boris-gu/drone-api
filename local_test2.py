#!/usr/bin/env python3

from drone_api import Drone_api

points = [[0, 0, 2]] + [[0, 0, 2],
                        [4, 0, 6],
                        [4, 4, 2],
                        [0, 4, 6]] * 2 + [[0, 0, 2]]
yaws = [0] + [90, 0, -90, 180]*2 + [0]

drone = Drone_api()
drone.start()
drone.sleep(5)
for i in range(len(points)):
    if drone.is_shutdown():
        break
    print('POINT ', i)
    drone.set_local_pose(*points[i], yaws[i])
    drone.sleep(5)
drone.stop()

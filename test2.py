from drone_api import Drone_api
from time import sleep

try:
    drone = Drone_api()
    drone.start()
    drone.set_pose(0, 0, 2)
    sleep(10)
    for i in range(2):
        print('POINT 1', i)
        drone.set_pose(0, 0, 2)
        sleep(3)
        drone.set_pose(yaw=0)
        sleep(1)
        print('POINT 2', i)
        drone.set_pose(4, 0, 6)
        sleep(3)
        drone.set_pose(yaw=-90)
        sleep(1)
        print('POINT 3', i)
        drone.set_pose(4, 4, 2)
        sleep(3)
        drone.set_pose(yaw=180)
        sleep(1)
        print('POINT 4', i)
        drone.set_pose(0, 4, 6)
        sleep(3)
        drone.set_pose(yaw=90)
        sleep(1)
    drone.set_pose(0, 0, 2, 0)
    sleep(10)
    drone.finish()
except KeyboardInterrupt:
    pass

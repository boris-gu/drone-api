from drone_api import Drone_api

drone = Drone_api()
while True:
    a = int(input('Введите высоту:'))
    drone.set_pose(z=a)

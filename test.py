from drone_api import Drone_api

drone = Drone_api()
drone.start()
while True:
    x = float(input('Введите x:'))
    y = float(input('Введите y:'))
    z = float(input('Введите z:'))
    yaw = float(input('Введите yaw:'))
    drone.set_pose(x, y, z, yaw)
    print()

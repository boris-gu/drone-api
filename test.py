from drone_api import Drone_api

try:
    drone = Drone_api()
    drone.start()
    for i in range(2):
        x = float(input('Введите x:'))
        y = float(input('Введите y:'))
        z = float(input('Введите z:'))
        yaw = float(input('Введите yaw:'))
        drone.set_pose(x, y, z, yaw)
        print()
    drone.finish()
except KeyboardInterrupt:
    pass

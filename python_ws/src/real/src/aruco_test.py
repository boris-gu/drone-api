#!/usr/bin/env python3

# =====================
#  Зависнуть у маркера
# =====================

import numpy as np
import cv2
import cv2.aruco as aruco
import time
from aruco_calibration import Calibration as clb
from drone_api import *  # закомментировать для тестирования без ROS

fps = 30.0
image_size = (848, 480)
time_now = time.gmtime(time.time())
video_file = f'{time.strftime("%Y.%m.%d %H:%M:%S", time.gmtime())}.avi'

FONT = cv2.FONT_HERSHEY_PLAIN


def toFixed(numObj, digits=0):
    return f'{numObj:.{digits}f}'


camera_matrix, dist_coef = clb.loadCoefficients('calibration_save.yaml')
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
# cap.set(cv2.CAP_PROP_EXPOSURE, 0.5) # работает не везде
cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_size[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size[1])
cap.set(cv2.CAP_PROP_FPS, fps)
out = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc(*'MJPG'),
                      fps, image_size)

if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict,
                                                          parameters=parameters,
                                                          cameraMatrix=camera_matrix,
                                                          distCoeff=dist_coef)

    print('\n\n\n')
    if np.all(ids is not None):
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners, 0.048, camera_matrix,
                                                                   dist_coef)
        aruco.drawDetectedMarkers(frame, corners)
        aruco.drawAxis(frame, camera_matrix,
                       dist_coef, rvec[0], tvec[0], 0.06)
        cv2.putText(frame, ' id' + str(ids[0])[1:-1], (20, 30), FONT,
                    1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, ' id' + str(ids[0])[1:-1], (20, 30), FONT,
                    1, (0, 0, 0), 1, cv2.LINE_AA)

        x, y, z, roll, pitch, yaw = Camera_api.marker_cam_pose(rvec[0][0],
                                                               tvec[0][0])
        # x, y, z, roll, pitch, yaw = (1, 2, 3, 4, 5, 6)  # для отладки без ROS
        marker_pose = [x, y, z, roll, pitch, yaw]
        # Белая обводка и черный текст
        print(toFixed(x, 3), toFixed(y, 3), toFixed(z, 3))
        print(toFixed(roll, 3), toFixed(pitch, 3), toFixed(yaw, 3))
        cv2.putText(frame, str(toFixed(x, 3)+'    ' +
                               toFixed(y, 3) + '    ' +
                               toFixed(z, 3) + '    '), (20, 90),
                    FONT, 1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, str(toFixed(x, 3) + '    ' +
                               toFixed(y, 3) + '    ' +
                               toFixed(z, 3) + '    '), (20, 90),
                    FONT, 1, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, str(toFixed(roll, 3)+'    ' +
                               toFixed(pitch, 3) + '    ' +
                               toFixed(yaw, 3)), (20, 120),
                    FONT, 1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, str(toFixed(roll, 3) + '    ' +
                               toFixed(pitch, 3) + '    ' +
                               toFixed(yaw, 3)), (20, 120),
                    FONT, 1, (0, 0, 0), 1, cv2.LINE_AA)
    else:
        print('NOT FOUND')
        cv2.putText(frame, 'NOT FOUND', (20, 30), FONT,
                    1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, 'NOT FOUND', (20, 30), FONT,
                    1, (0, 0, 0), 1, cv2.LINE_AA)

    out.write(frame)
    # cv2.imshow('frame', frame) # для отладки. На headless-дистрибутивах крашится
    # if cv2.waitKey(1) != -1:
    #    break
cap.release()
cv2.destroyAllWindows()

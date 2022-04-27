#!/usr/bin/env python3

# =====================
#  Зависнуть у маркера
# =====================

import numpy as np
import cv2
import cv2.aruco as aruco
import time
from aruco_calibration import Calibration as clb
import argparse

# VIDEO SETTINGS
fps = 30.0
image_size = (620, 480)

# ARGPARSER
parser = argparse.ArgumentParser()
parser.add_argument('--write', dest='write', action='store_true',
                    help='if set, video stream is written to a file')
parser.add_argument('--show', dest='show', action='store_true',
                    help='if set, video stream is displayed in the window')
parser.add_argument('--no-ros', dest='no_ros', action='store_true',
                    help='if set, video stream is displayed in the window')
args = parser.parse_args()
parser.set_defaults(write=False)
parser.set_defaults(show=False)
parser.set_defaults(no_ros=False)

if not args.no_ros:
    from drone_api import *  # закомментировать для тестирования без ROS


def toFixed(numObj, digits=0):
    return f'{numObj:.{digits}f}'


time_now = time.gmtime(time.time())
video_file = f'{time.strftime("%Y.%m.%d %H:%M:%S", time.gmtime())}.avi'

FONT = cv2.FONT_HERSHEY_PLAIN

camera_matrix, dist_coef = clb.loadCoefficients('calibration_save.yaml')
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_size[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size[1])
cap.set(cv2.CAP_PROP_FPS, fps)
if args.write:
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
        if args.no_ros:
            x, y, z, roll, pitch, yaw = (1, 2, 3, 4, 5, 6)
        else:
            x, y, z, roll, pitch, yaw = Camera_api.marker_cam_pose(rvec[0][0],
                                                                   tvec[0][0])
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
    if args.write:
        out.write(frame)
    # Для отладки. На headless-дистрибутивах крашится
    if args.show:
        cv2.imshow('frame', frame)
cap.release()
if args.write:
    out.release()
cv2.destroyAllWindows()

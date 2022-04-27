#!/usr/bin/env python3

# =====================
#  Зависнуть над маркером
# =====================

from turtle import circle
import numpy as np
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from aruco_calibration import Calibration as clb
from drone_api import *
from math import pi, sqrt, sin, cos, atan2
import argparse
import time

FONT = cv2.FONT_HERSHEY_PLAIN


class Circle:
    def __init__(self):
        self.__buffer = [None] * 25
        self.__yaw = 0
        self.__counter = 0

    def write(self, pose):
        self.__buffer[self.__counter] = pose
        self.__yaw = pose[5]
        self.__counter += 1
        if self.__counter >= len(self.__buffer):
            self.__counter = 0

    def mean(self):
        num = 0
        mean_pose = [0] * 6
        for i in self.__buffer:
            if i is not None:
                num += 1
                for j in range(len(mean_pose)):
                    mean_pose[j] += i[j]
        if num != 0:
            for i in range(len(mean_pose) - 1):
                mean_pose[i] = mean_pose[i] / num
            mean_pose[5] = self.__yaw
        return mean_pose

    def erase_yaw(self):
        self.__yaw = 0


def toFixed(numObj, digits=0):
    return f'{numObj:.{digits}f}'


# ARGPARSER
parser = argparse.ArgumentParser()
parser.add_argument('--write', dest='write', action='store_true',
                    help='if set, video stream is written to a file')
parser.add_argument('--show', dest='show', action='store_true',
                    help='if set, video stream is displayed in the window')
parser.add_argument('--output', dest='output', action='store_true',
                    help='if set, ArUco recognition process is output to the terminal')
args = parser.parse_args()
parser.set_defaults(write=False)
parser.set_defaults(show=False)
parser.set_defaults(output=False)

# OPEN VIDEO
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
if args.write:
    import time
    time_now = time.gmtime(time.time())
    video_file = f'{time.strftime("%Y.%m.%d %H:%M:%S", time.gmtime())}.avi'
    while True:
        ret, frame = cap.read()
        if ret:
            image_size = frame.shape[:2]
            print(f'Resolution: {image_size[1]}x{image_size[0]}')
            break
    fps = 25.0
    out = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc(*'MJPG'),
                          fps, (image_size[1], image_size[0]))


# calibration_save.yaml - уже проведена калибровка
camera_matrix, dist_coef = clb.loadCoefficients('calibration_save.yaml')
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

# DRONE PREPARATION
drone = Drone_api()
drone.start()
rospy.loginfo('Drone armed')
drone.sleep(5)
drone.set_local_pose(0, 0, 4, 0)
while not drone.point_is_reached() and not drone.is_shutdown():
    drone.sleep(0.5)
cir = Circle()

# CYCLE
while not drone.is_shutdown():
    # Get ArUco pose
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict,
                                                          parameters=parameters,
                                                          cameraMatrix=camera_matrix,
                                                          distCoeff=dist_coef)
    if np.all(ids is not None):
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners, 0.2, camera_matrix,
                                                                   dist_coef)

        aruco.drawDetectedMarkers(frame, corners)
        aruco.drawAxis(frame, camera_matrix,
                       dist_coef, rvec[0], tvec[0], 0.2)
        cv2.putText(frame, ' id' + str(ids[0])[1:-1], (20, 30), FONT,
                    1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, ' id' + str(ids[0])[1:-1], (20, 30), FONT,
                    1, (0, 0, 0), 1, cv2.LINE_AA)
        drone_pose = drone.get_local_pose()
        x, y, z, roll, pitch, yaw = Camera_api.marker_local_pose(rvec[0][0], tvec[0][0],
                                                                 drone_pose, (0, 0, 0, 0, pi/2, 0))
        cir.write([x, y, z, roll, pitch, yaw])
        if args.output:
            rospy.loginfo(str(toFixed(x, 3) + ' ' +
                              toFixed(y, 3) + ' ' +
                              toFixed(z, 3) + ' ' +
                              toFixed(roll, 3) + ' ' +
                              toFixed(pitch, 3) + ' ' +
                              toFixed(yaw, 3)))
        # Белая обводка и черный текст
        cv2.putText(frame, str(toFixed(x, 3)+'    ' +
                               toFixed(y, 3) + '    ' +
                               toFixed(z, 3) + '    '), (20, 70+20),
                    FONT, 1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, str(toFixed(x, 3) + '    ' +
                               toFixed(y, 3) + '    ' +
                               toFixed(z, 3) + '    '), (20, 70+20),
                    FONT, 1, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, str(toFixed(roll, 3)+'    ' +
                               toFixed(pitch, 3) + '    ' +
                               toFixed(yaw, 3)), (20, 100+20),
                    FONT, 1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, str(toFixed(roll, 3) + '    ' +
                               toFixed(pitch, 3) + '    ' +
                               toFixed(yaw, 3)), (20, 100+20),
                    FONT, 1, (0, 0, 0), 1, cv2.LINE_AA)
    else:
        # Сбрасываем yaw маркера, чтобы, в случае потери маркера, дрон не продолжал кружиться
        cir.erase_yaw()
        if args.output:
            rospy.loginfo('NOT FOUND')
        cv2.putText(frame, 'NOT FOUND', (20, 30), FONT,
                    1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, 'NOT FOUND', (20, 30), FONT,
                    1, (0, 0, 0), 1, cv2.LINE_AA)
    if args.write:
        out.write(frame)
    if args.show:
        cv2.imshow('frame', frame)
        cv2.waitKey(1)
    # Set drone pose
    marker_pose = cir.mean()
    drone_pose = drone.get_local_pose()
    correct_drone_yaw = marker_pose[5] + drone_pose[5]
    # Чтобы исключить "дерганье" дрона из-за небольшой ошибки определения местоположения маркера:
    drone.set_local_pose(marker_pose[0], marker_pose[1],
                         marker_pose[2] + 3, correct_drone_yaw)
    drone.sleep(0.5)
rospy.loginfo('Drone disarmed')
cap.release()
if args.write:
    out.release()
    rospy.loginfo(f'Create{video_file}')
cv2.destroyAllWindows()

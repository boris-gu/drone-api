#!/usr/bin/env python3

# =====================
#  Зависнуть у маркера
# =====================

import numpy as np
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from aruco_calibration import Calibration as clb
from drone_api import *
from math import pi, sqrt, sin, cos, atan2


FONT = cv2.FONT_HERSHEY_PLAIN


def toFixed(numObj, digits=0):
    return f'{numObj:.{digits}f}'


def callback(data):
    # Мост для преобразования изображения из формата ROS в формат OpenCV
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    except Exception as e:
        rospy.loginfo(e)
        return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict,
                                                          parameters=parameters,
                                                          cameraMatrix=camera_matrix,
                                                          distCoeff=dist_coef)
    global marker_pose
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
                                                                 drone_pose, (0, 0, -0.05, 0, pi/2, 0))
        marker_pose = [x, y, z, roll, pitch, yaw]
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
        marker_pose[5] = 0
        cv2.putText(frame, 'NOT FOUND', (20, 30), FONT,
                    1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, 'NOT FOUND', (20, 30), FONT,
                    1, (0, 0, 0), 1, cv2.LINE_AA)
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
    except Exception as e:
        rospy.loginfo(e)


# calibration_save.yaml - уже проведена калибровка
camera_matrix, dist_coef = clb.loadCoefficients('calibration_save.yaml')
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

marker_pose = [0, 0, 0, 0, 0, 0]
drone = Drone_api()
drone.start()
rospy.loginfo('Drone armed')
image_sub = rospy.Subscriber('/iris_bottom_fpv/usb_cam/image_raw',
                             Image, callback, queue_size=1)
rospy.loginfo('Start Subscriber')
image_pub = rospy.Publisher('/iris_bottom_fpv/usb_cam/location_img',
                            Image, queue_size=1)
rospy.loginfo('Start Publisher')

drone.sleep(5)
drone.set_local_pose(0, 0, 5, 0)
while not drone.point_is_reached() and not drone.is_shutdown():
    drone.sleep(0.5)
# Т.к. после закрытия топика с изображением сохраняется последний кадр,
# необходимо перед стартом основной части скрипта "сбросить" значения маркера
marker_pose = [0, 0, 0, 0, 0, 0]
current_drone_x = 0
current_drone_y = 0
while not drone.is_shutdown():
    drone_pose = drone.get_local_pose()
    # Чтобы исключить "дерганье" дрона из-за небольшой ошибки определения местоположения маркера:
    current_drone_x = (current_drone_x + marker_pose[0])/2
    current_drone_y = (current_drone_y + marker_pose[1])/2
    correct_drone_yaw = marker_pose[5] + drone_pose[5]
    drone.set_local_pose(current_drone_x, current_drone_y,
                         marker_pose[2] + 4, correct_drone_yaw)
    drone.sleep(0.5)
rospy.loginfo('Drone disarmed')

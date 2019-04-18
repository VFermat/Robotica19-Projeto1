#! /usr/bin/env python
# -*- coding:utf-8 -*-


# IMPORTS:
import math
import time
from math import cos, radians, sin

import numpy as np
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import UInt8

import analiza_cor
import cv2
import visao_module

# VARIÁVEIS GLOBAIS:
bridge = CvBridge()

# Tempo máximo de atraso, em nano-segundos.
atraso = 1.5E9

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados.
# Descarta imagens que chegam atrasadas demais.
check_delay = True

# Variáveis globais do MobileNet
cv_image = None
cat_seen = False
cat_center = (0, 0)
image_center = (0, 0)

# Variáveis Globais do Bumper
bump = None
v = 0.1
w = 0.15

# Variáveis globais do LaserScan
min_distance = 0.2
close_scan = False
close_scan_velocity = None
scan_v = None
scan_w = None

# Variáveis globais do Reconhecimento de Cor
max_area_accepted = 25
seen_color = False
color_velocity = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1.5))


def roda_todo_frame(imagem):
    """
        A função a seguir é chamada sempre que chega um novo frame
    """

    global cv_image
    global cat_seen
    global cat_center
    global image_center
    global atraso

    now = rospy.get_rostime()

    imgtime = imagem.header.stamp

    lag = now - imgtime  # calcula o lag

    delay = lag.nsecs
    #print("-----------DELAY: ", delay)

    if delay > atraso and check_delay == True:
        print("Descartando por causa do delay do frame:", delay)
        return

    try:
        antes = time.clock()

        # Script para o MobileNet
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        centro, imagem, resultados = visao_module.processa(cv_image)

        image_center = centro

        for r in resultados:
            if r[0] == "bottle":
                cat_seen = True

                # Pegando o centro da imagem do objeto:
                cX = (r[2][0] + r[3][0]) / 2
                cY = (r[2][1] + r[3][1]) / 2
                cat_center = (cX, cY)

                #print("[LOG] OBJECT FOUND! CENTER AT: ", cat_center)

        if cat_seen == True:
            cat_seen = True

        else:
            cat_seen = False

        # Script para a detecção de cores
        color_media, color_center, color_area = analiza_cor.identifica_cor(
            cv_image)
        if color_area >= max_area_accepted:
            seen_color = True
        else:
            seen_color = False

        depois = time.clock()

    except CvBridgeError as e:
        print('ex', e)


def get_bump(datas):
    global bump
    bump = datas.data
    print("[LOG] Bumper: ", bump)


def analyze_scan(datas):
    global close_scan
    global close_scan_velocity
    global scan_v
    global scan_w

    scan_ranges = datas.ranges
    scan_cone_front = list(scan_ranges[:45] + scan_ranges[315:])

    smallest_distance = min(scan_cone_front)
    print(smallest_distance)

    if smallest_distance <= min_distance and smallest_distance != 0.0 and smallest_distance != inf:
        print("[LOG] CLOSEST OBJECT(m): ", smallest_distance)
        close_scan = True

    else:
        close_scan = False


if __name__ == "__main__":

    rospy.init_node("projeto")

    topico_imagem = "/kamera"
    print("Usando ", topico_imagem)

    # Para renomear a *webcam*
    #   Primeiro instale o suporte https://github.com/Insper/robot19/blob/master/guides/debugar_sem_robo_opencv_melodic.md
    #
    #	Depois faça:
    #
    #	rosrun cv_camera cv_camera_node
    #
    # 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
    #
    #
    # Para renomear a câmera simulada do Gazebo
    #
    # 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
    #
    # Para renomear a câmera da Raspberry
    #
    # 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
    #

    image_receiver = rospy.Subscriber(
        topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size=2**24)

    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
    bump_receiver = rospy.Subscriber("/bumper", UInt8, get_bump)
    scan_receiver = rospy.Subscriber("/scan", LaserScan, analyze_scan)

    while not rospy.is_shutdown():
        stop = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        vel_forward = Twist(Vector3(v, 0, 0), Vector3(0, 0, 0))
        vel_backward = Twist(Vector3(-v, 0, 0), Vector3(0, 0, 0))
        vel_right = Twist(Vector3(0, 0, 0), Vector3(0, 0, -w))
        vel_left = Twist(Vector3(0, 0, 0), Vector3(0, 0, w))
        time_to_walk = 0.01

        # Checking MobileNet
        if cat_seen:
            dif = cat_center[0] - image_center[0]
            print("[LOG] DISTANCE: ", dif)

            dif_range = 20

            if dif > dif_range:
                print("[LOG] Turning RIGHT.")
                publisher.publish(vel_right)
                rospy.sleep(time_to_walk)
                cat_seen = False

            elif dif < - dif_range:
                print("[LOG] Turning LEFT.")
                publisher.publish(vel_left)
                rospy.sleep(time_to_walk)
                cat_seen = False

            elif close_scan == False:
                print("[LOG] Moving FORWARD.")
                publisher.publish(vel_forward)
                rospy.sleep(0.2)
                cat_seen = False

        # Checking Bumpers
        time_forward_backward = 0.5
        time_turn = 2

        if bump == 1:
            publisher.publish(vel_backward)
            rospy.sleep(time_forward_backward)

            publisher.publish(vel_right)
            rospy.sleep(time_turn)

            publisher.publish(stop)
            rospy.sleep(time_forward_backward)

            bump = None

        elif bump == 2:
            publisher.publish(vel_backward)
            rospy.sleep(time_forward_backward)

            publisher.publish(vel_left)
            rospy.sleep(time_turn)

            publisher.publish(stop)
            rospy.sleep(time_forward_backward)

            bump = None

        elif bump == 3:
            publisher.publish(vel_forward)
            rospy.sleep(time_forward_backward)

            publisher.publish(vel_left)
            rospy.sleep(time_turn)

            publisher.publish(stop)
            rospy.sleep(time_forward_backward)

            bump = None

        elif bump == 4:
            publisher.publish(vel_forward)
            rospy.sleep(time_forward_backward)

            publisher.publish(vel_right)
            rospy.sleep(time_turn)

            publisher.publish(stop)
            rospy.sleep(time_forward_backward)

            bump = None

        # Checking Laser Scan
        if close_scan == True:
            publisher.publish(vel_backward)
            rospy.sleep(0.5)

            close_scan = False

        if seen_color:
            publisher.publish(color_velocity)
            rospy.sleep(4)
            publisher.publish(stop)

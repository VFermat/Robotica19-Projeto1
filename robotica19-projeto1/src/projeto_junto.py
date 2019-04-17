#! /usr/bin/env python
# -*- coding:utf-8 -*-



# IMPORTS:
import rospy
import numpy as np
from math import sin, cos, radians
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan

import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import visao_module



# VARIÁVEIS GLOBAIS:
bridge = CvBridge()

# Tempo máximo de atraso, em nano-segundos.
atraso = 1.5E9

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais.
check_delay = False

cv_image = None
cat_seen = False
cat_center = (0, 0)
image_center = (0, 0)

bump = None
v = 0.1
w = 0.3
min_distance = 0.3
close_scan = False
close_scan_velocity = None
scan_v = None
scan_w = None



def roda_todo_frame(imagem):
    """
        A função a seguir é chamada sempre que chega um novo frame
    """

    global cv_image
    global cat_seen
    global cat_center
    global image_center

    now = rospy.get_rostime()

    imgtime = imagem.header.stamp

    lag = now - imgtime # calcula o lag

    delay = lag.nsecs

    if delay > atraso and check_delay == True:
        print("Descartando por causa do delay do frame:", delay)
        return

    try:
        antes = time.clock()

        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        centro, imagem, resultados =  visao_module.processa(cv_image)

        image_center = (len(imagem) / 2, len(imagem[0]) / 2)
        #print("Image centers: ", image_center)

        for r in resultados:
            if r[0] == "cat" or r[0] == "dog":
                cat_seen = True

                # Pegando o centro da imagem do gato:
                cX = (r[2][0] + r[3][0]) / 2
                cY = (r[2][1] + r[3][1]) / 2
                cat_center = (cX, cY)

                print("Cat seen! Center at: ", cat_center)

        depois = time.clock()

    except CvBridgeError as e:
        print('ex', e)



def get_bump(datas):
    global bump
    bump = datas.data



def analyze_scan(datas):
    global close_scan
    global close_scan_velocity
    global scan_v
    global scan_w

    scan_ranges = datas.ranges
    scan_cone_front = list(scan_ranges[:45] + scan_ranges[315:])

    smallest_distance = min(scan_cone_front)

    while smallest_distance <= 0.1:
        del scan_cone_front[scan_cone_front.index(smallest_distance)]

        smallest_distance = min(scan_cone_front)



    if smallest_distance <= min_distance:
        print("[LOG] CLOSEST OBJECT(m): ", smallest_distance)    
        close_scan = True

    else:
        close_scan = False



if __name__=="__main__":

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

    image_receiver = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

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
            dif = image_center[0] - cat_center[0]
            print("[LOG] DISTANCE: ", dif)

            if dif > -120 and dif < -70 and close_scan == False:
                print("[LOG] Moving FORWARD.")
                publisher.publish(vel_forward)
                rospy.sleep(0.2)
                cat_seen = False
                continue

            elif dif > - 120:
                print("[LOG] Turning LEFT.")
                publisher.publish(vel_left)
                rospy.sleep(time_to_walk)
                cat_seen = False

            elif dif < - 70:
                print("[LOG] Turning RIGHT.")
                publisher.publish(vel_right)
                rospy.sleep(time_to_walk)
                cat_seen = False

        # Checking Bumpers
        if bump == 1:
            publisher.publish(vel_forward)
            rospy.sleep(1)
            publisher.publish(vel_right)
            rospy.sleep(1)
            bump = None

        elif bump == 2:
            publisher.publish(vel_forward)
            rospy.sleep(1)
            publisher.publish(vel_left)
            rospy.sleep(1)
            bump = None

        elif bump == 3:
            publisher.publish(vel_backward)
            rospy.sleep(1)
            publisher.publish(vel_right)
            rospy.sleep(1)
            bump = None

        elif bump == 4:
            publisher.publish(vel_forward)
            rospy.sleep(1)
            publisher.publish(vel_backward)
            rospy.sleep(1)
            bump = None


        # Checking Laser Scan
        if close_scan:
            publisher.publish(stop)
            rospy.sleep(0.5)
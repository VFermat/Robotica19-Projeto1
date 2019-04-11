#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import visao_module


bridge = CvBridge()

cv_image = None
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global cv_image
    global cat_seen
    global cat_center
    global image_center

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    #print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return

    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        centro, imagem, resultados =  visao_module.processa(cv_image)

        image_center = (len(imagem) / 2, len(imagem[0]) / 2)
        #print("Image centers: ", image_center)

        for r in resultados:
            if r[0] == "cat":
                cat_seen = True

                # Pegando o centro da imagem do gato:
                cX = (r[2][0] + r[3][0]) / 2
                cY = (r[2][1] + r[3][1]) / 2
                cat_center = (cX, cY)

                print("Cat seen! Center at: ", cat_center)

        depois = time.clock()

    except CvBridgeError as e:
        print('ex', e)



cat_seen = False
cat_center = (0, 0)
image_center = (0, 0)


    
if __name__=="__main__":
    rospy.init_node("cor")

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

    receiver = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    try:

        while not rospy.is_shutdown():
            stop = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            turn_right_slow = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.2))
            turn_left_slow = Twist(Vector3(0, 0, 0), Vector3(0, 0, - 0.2))

            if cat_seen:
                dif = image_center[0] - cat_center[0]
                print("Distance: ", dif)

                if dif > -120 and dif < -70:
                    print("STOP TURNING!")
                    publisher.publish(stop)
                    continue

                elif dif > - 120:
                    publisher.publish(turn_right_slow)

                elif dif < - 70:
                    publisher.publish(turn_left_slow)

                else:
                    publisher.publish(stop)

            else:
                publisher.publish(stop)

            rospy.sleep(0.1)            

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")



#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8


bump = None
v = 0.1
w = 0.3

def get_bump(datas):
    global bump
    bump = datas.data


if __name__=="__main__":

    rospy.init_node("projeto")

    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
    bump_receiver = rospy.Subscriber("/bumper", UInt8, get_bump)

    while not rospy.is_shutdown():
        
        vel_forward = Twist(Vector3(v, 0, 0), Vector3(0, 0, 0))
        vel_backward = Twist(Vector3(-v, 0, 0), Vector3(0, 0, 0))
        vel_right = Twist(Vector3(0, 0, 0), Vector3(0, 0, w))
        vel_left = Twist(Vector3(0, 0, 0), Vector3(0, 0, -w))

        if bump == 1:
            velocity_publisher.publish(vel_forward)
            rospy.sleep(1)
            velocity_publisher.publish(vel_right)
            rospy.sleep(1)
            bump = None

        elif bump == 2:
            velocity_publisher.publish(vel_forward)
            rospy.sleep(1)
            velocity_publisher.publish(vel_left)
            rospy.sleep(1)
            bump = None

        elif bump == 3:
            velocity_publisher.publish(vel_backward)
            rospy.sleep(1)
            velocity_publisher.publish(vel_right)
            rospy.sleep(1)
            bump = None

        elif bump == 4:
            velocity_publisher.publish(vel_forward)
            rospy.sleep(1)
            velocity_publisher.publish(vel_backward)
            rospy.sleep(1)
            bump = None

        else:
            vel = Twist(Vector3(-v, 0, 0), Vector3(0, 0, 0))
            velocity_publisher.publish(vel)
            rospy.sleep(0.1)
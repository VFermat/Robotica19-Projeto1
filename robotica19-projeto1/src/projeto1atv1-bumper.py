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
    print(bump)


if __name__=="__main__":

    rospy.init_node("bump")

    velocity = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
    bump_scan = rospy.Subscriber("/bumper", UInt8, get_bump)

    while not rospy.is_shutdown():
        if bump == 1:
            vel = Twist(Vector3(v, 0, 0), Vector3(0, 0, 0))
            velocity.publish(vel)
            rospy.sleep(1)

            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, +w))
            velocity.publish(vel)
            rospy.sleep(1)

            bump = None

        elif bump == 2:
            vel = Twist(Vector3(v, 0, 0), Vector3(0, 0, 0))
            velocity.publish(vel)
            rospy.sleep(1)

            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -w))
            velocity.publish(vel)
            rospy.sleep(1)

            bump = None

        elif bump == 3:
            vel = Twist(Vector3(-v, 0, 0), Vector3(0, 0, 0))
            velocity.publish(vel)
            rospy.sleep(1)

            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, w))
            velocity.publish(vel)
            rospy.sleep(1)

            bump = None

        elif bump == 4:
            vel = Twist(Vector3(-v, 0, 0), Vector3(0, 0, 0))
            velocity.publish(vel)
            rospy.sleep(1)

            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -w))
            velocity.publish(vel)
            rospy.sleep(1)

            bump = None

        else:
            vel = Twist(Vector3(-v, 0, 0), Vector3(0, 0, 0))
            velocity.publish(vel)
            rospy.sleep(0.1)
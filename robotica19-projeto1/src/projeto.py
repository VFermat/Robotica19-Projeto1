#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
from math import sin, cos, radians
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan


bump = None
v = 0.1
w = 0.3
min_distance = 0.3
close_scan = False
close_scan_velocity = None
scan_v = None
scan_w = None



def get_bump(datas):
    global bump
    bump = datas.data



def analyze_scan(datas):
    global close_scan
    global close_scan_velocity
    global scan_v
    global scan_w

    scan_ranges = datas.ranges
    #print("SCAN: ", scan_ranges)
    scan_cone_front = list(scan_ranges[:45] + scan_ranges[315:])
    print("SCAN CONE: ", scan_cone_front)

    smallest_distance = min(scan_cone_front)

    while smallest_distance <= 0.1:
        del scan_cone_front[scan_cone_front.index(smallest_distance)]

        smallest_distance = min(scan_cone_front)


    print("SCAN CONE: ", scan_cone_front)
    print("DIST: ", smallest_distance)    

    if smallest_distance <= min_distance:
        close_scan = True

    else:
        close_scan = False



if __name__=="__main__":

    rospy.init_node("projeto")

    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
    bump_receiver = rospy.Subscriber("/bumper", UInt8, get_bump)
    scan_receiver = rospy.Subscriber("/scan", LaserScan, analyze_scan)

    while not rospy.is_shutdown():
        stop = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        vel_forward = Twist(Vector3(v, 0, 0), Vector3(0, 0, 0))
        vel_backward = Twist(Vector3(-v, 0, 0), Vector3(0, 0, 0))
        vel_right = Twist(Vector3(0, 0, 0), Vector3(0, 0, w))
        vel_left = Twist(Vector3(0, 0, 0), Vector3(0, 0, -w))

        # Checking Bumpers
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


        # Checking Laser Scan
        if close_scan:
            velocity_publisher.publish(stop)
            rospy.sleep(1)

        else:
            velocity_publisher.publish(vel_forward)
            rospy.sleep(0.1)
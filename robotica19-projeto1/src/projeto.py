#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
from math import sin, cos, radians
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8, LaserScan


bump = None
v = 0.1
w = 0.3
min_distance = 10
close_scan = False
close_scan_velocity = None

def get_bump(datas):
    global bump
    bump = datas.data

def analyze_scan(datas):
    global close_scan
    global close_scan_velocity
    scan_ranges = datas.ranges
    smallest_distance = min(scan_ranges)
    if smallest_distance <= min_distance:
        close_scan = True
        angle = close_scan.index(smallest_distance)
        scan_v = -sin(radians(angle))
        scan_w = cos(radians(angle))
        close_scan_velocity = Twist(Vector3(scan_v, 0, 0), Vector3(0, 0, scan_w))
    else:
        close_scan = False

if __name__=="__main__":

    rospy.init_node("projeto")

    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
    bump_receiver = rospy.Subscriber("/bumper", UInt8, get_bump)
    scan_receiver = rospy.Subscriber("/scan", LaserScan, analyze_scan)

    while not rospy.is_shutdown():
        
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

        else:
            vel = Twist(Vector3(-v, 0, 0), Vector3(0, 0, 0))
            velocity_publisher.publish(vel)
            rospy.sleep(0.1)

        # Checking Laser Scan
        if close_scan:
            velocity_publisher.publish(close_scan_velocity)
            rospy.sleep(0.5)
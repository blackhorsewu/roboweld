#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import sys
sys.path.append('../urx/')
import urx

# for ROS
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker

# Logging
import logging

from math import pi
import numpy as np
import math

def callback(mk_points):
    # The received message has a list of points.
    # Each point of mk_points.points has x, y, z of the point.
    # We need to construct the orientations for each of them.
    # So, we are constructing a Pose List. Let's call it pose_list.
    # Technically this Pose List is a Path.

    # Home joint angles
    home = [0, -pi/2, 0, -pi/2, 0, 0]

    # Once the list of Poses is constructed
    # Move to the beginning of the Path. Before we start moving, we need
    # to switch on the Welding Torch.
    # So we Move To Single Pose using the URx
    #print(len(mk_points.points))
    i = 0
    pose_list = []
    length=len(mk_points.points)
    print(length)
    while i < length:
        pose = []
        pose.append(mk_points.points[i].x)
        pose.append(mk_points.points[i].y)
        pose.append(mk_points.points[i].z+0.03)
    # Since the welding torch has been changed from a bent one to a straight
    # one, so the orientation of the TCP needs to be changed as well.
        # r = 0.073
        # p = -0.039
        # y = 3.119
        r = 2.0617
        p = -2.4148
        y = 0.0
        pose.append(r)
        pose.append(p)
        pose.append(y)
        #print("Press Enter to go to the point")
        #raw_input()
        if i == 0:
            ur5.movej_to_pose(pose)
            ur5.set_digital_out(0, True)
        #ur5.movej_to_pose(pose)
        pose_list.append(pose)
        i+=1
    # ur5.movej_to_pose_list(pose_list)
    # ur5.set_digital_out(0, False)
    # ur5.movej(home)

def listerner():
    print("Subscribed to the wayPoints marker")

    # Subscribe to the topic "wayPoints" published by perception_node
    # the received message is of type visualization_msgs::Marker.
    # When the message is received, call "callback" defined above.
    # rospy.Subscriber("wayPoints", Marker, callback)
    rospy.Subscriber("rviz_visual_tools", Marker, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    try:
        logging.basicConfig(level=logging.WARN)
        ur5 = urx.Robot("192.168.0.103")

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous = True flag means that rospy will choose a unique
        # name for our 'listerner' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('move_ur5', anonymous=True)

        # Before doing anything, make sure it is running
        print("RoboWeld motion URx node started\n")

        while not rospy.is_shutdown():
            # rospy.Subscriber(the path topic)
            listerner() # calls the listerner defined above

    except rospy.ROSInterruptEcxeption:
        rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")

    finally:
        ur5.close()


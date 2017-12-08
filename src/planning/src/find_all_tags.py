#!/usr/bin/env python
from __future__ import division
import rospy
import tf
import sys
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
import geometry_msgs
from moveit_commander import MoveGroupCommander
from baxter_interface import gripper as robot_gripper
import time
import numpy as np

def get_artag_location(listener, ar_marker):
    trans = None
    # print 'Locating %s...' % ar_marker
    # while not trans:
    try:
        trans, rot = listener.lookupTransform("base", str(ar_marker), rospy.Time(0))
    except:
        pass
    return trans

def main():
    # Initiate node, listener, and gripper
    rospy.init_node("testing")
    listener = tf.TransformListener()
    right_gripper = robot_gripper.Gripper('right')

    while not rospy.is_shutdown():
        try:
            tags = []
            for i in range(12):
                tags.append(get_artag_location(listener, "ar_marker_%d" % i))
            # print all(tags)
            if all(tags):
            	trans, rot = listener.lookupTransform("base", "left_gripper", rospy.Time(0))
            	print trans, rot
            	sys.exit()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    sys.exit()


if __name__ == '__main__':
    main()
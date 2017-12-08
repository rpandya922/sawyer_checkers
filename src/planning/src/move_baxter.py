#!/usr/bin/env python

import rospy
import sys
from functools import partial
sys.path.insert(0,'~/ros_workspaces/sawyer_checkers/src/planning/src')
from ik import *

upper_left = []
lower_right = []
board_size = 8

def location(self, coordinate):
    i, j = coordinate
    left, upper, z1 = upper_left
    right, lower, z2 = lower_right
    x = i * (left - right)/(board_size - 1) + right
    y = j * (upper - lower)/(board_size - 1) + lower
    z = (z1 + z2) / 2.0
    return x, y, z

def location_helper(self, pos):
    j = (pos - 1) // 4
    i = ((pos - 1) % 4) * 2 + (1 - j % 2)
    return j, i

def robot_make_move(group, gripper, listener, start, end):

    # Calculate cartesian positions based on given position
    x, y, z = location(location_helper(start))
    end_x, end_y, end_z = location(location_helper(end))

    move_group_to(group, x, y, z + 0.1)
    raw_input("wait")

    # Move to AR tag and pick up with gripper
    (temp_x, temp_y, temp_z), _ = listener.lookupTransform("base", "right_gripper", 
                                                            rospy.Time(0))
    move_group_to(group, temp_x, temp_y, temp_z-0.1)
    gripper.close()
    time.sleep(1)

    # Move up with AR tag and release gripper
    move_group_to(group, temp_x, temp_y, temp_z)
    move_group_to(group, end_x, end_y, end_z + 0.1)
    gripper.open()
    time.sleep(1)

    move_group_to(group, INTER_POS[0], INTER_POS[1], INTER_POS[2])

def move_callback(group, gripper, listener, message):
    robot_make_move(group, gripper, listener, message.start, message.end)

def calibration_callback(message):
    global upper_left
    global lower_right
    upper_left = message.x_ul, message.y_ul, message.z_ul
    lower_right = message.x_lr, message.y_lr, message.z_lr

def listener():
    rospy.init_node('moving_listener', anonymous=True)
    listener = tf.TransformListener()
    right_gripper = robot_gripper.Gripper('right')
    group = MoveGroupCommander("right_arm")
    init_move_group(group)
    obs_group = MoveGroupCommander("left_arm")
    init_move_group(obs_group)

    robot_move_callback = partial(move_callback, group, gripper, listener)
    rospy.Subscriber("robot_moves", CheckersMove, robot_move_callback)
    rospy.Subscriber("board_calibration", BoardCalibration, calibration_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
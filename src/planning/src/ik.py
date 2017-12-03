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
import copy

NEUTRAL_POS = [0.646, -0.115, -0.042]

class RobotCheckers():
    """docstring for CheckersGame"""
    def __init__(self, upper_left_marker, lower_right_marker, board_size=8):
        self.board_size = board_size
        self.upper_left = upper_left_marker
        self.lower_right = lower_right_marker
        self.opponent_pieces = {}
        for i in range(12):
            self.opponent_pieces[i] = i + 21

    def location(self, coordinate):
        i, j = coordinate
        left, upper, z1 = self.upper_left
        right, lower, z2 = self.lower_right
        x = i * (left - right)/(self.board_size - 1) + right
        y = j * (upper - lower)/(self.board_size - 1) + lower
        # z = j * (z1 - z2)/(self.board_size - 1) + z2
        z = (z1 + z2) / 2.0
        return x, y, z

    def location_helper(self, pos):
        j = (pos - 1) // 4
        i = ((pos - 1) % 4) * 2 + (1 - j % 2)
        return j, i

    def detect_opponent_move(self, listener, wait_time=5):
        # Give users time to move piece
        # time.sleep(wait_time)
        raw_input("wait for human move")

        # Iterate through all the pieces and record new location
        move = -1
        for piece in self.opponent_pieces:
            if self.opponent_pieces[piece] != -1:
                x, y, _ = get_artag_location(listener, "ar_marker_%s" % piece)
                pos = self.location_to_position(x, y)
                print piece, pos

                # This piece has been moved
                if self.opponent_pieces[piece] != pos:
                    move = (self.opponent_pieces[piece], pos)
                    self.opponent_pieces[piece] = pos
                    break
        print "move", move

        # return self.convert_to_bin(move)
        return move

    def location_to_position(self, x, y):
        upper, left, _ = self.upper_left
        lower, right, _ = self.lower_right

        i = int(np.round((x - lower) * (self.board_size - 1) / (upper - lower)))
        j = int(np.round((y - right) * (self.board_size - 1) / (left - right)))

        print "coordinates", i, j
        return (i * 4) + (j // 2) + 1

    def convert_to_bin(self, move):
        assert move != -1

        pos1, pos2 = move
        if pos1 > pos2:
            s = "1" + "0"*(pos1 - pos2 - 1) + "1" + "0"*pos2
        else:
            s = "1" + "0"*(pos2 - pos1 - 1) + "1" + "0"*pos1
        return int(s, 2)

    def robot_make_move(self, group, gripper, listener, start, end, taken_pieces):
        # Detect taken pieces
        for piece in taken_pieces:
            self.opponent_pieces[piece] = -1

        # Calculate cartesian positions based on given position
        x, y, z = self.location(self.location_helper(start))
        end_x, end_y, end_z = self.location(self.location_helper(end))

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


def move_checkers_piece(group, gripper, listener, checkers_number, end_location=None):
    # Get location of AR tag
    x, y, z = get_artag_location(listener, "ar_marker_%d" % checkers_number)

    # Move to place above AR tag
    move_group_to(group, x - 0.02, y + 0.02, z + 0.1)
    raw_input("wait")

    # Move to AR tag and pick up with gripper
    (temp_x, temp_y, temp_z), _ = listener.lookupTransform("base", "right_gripper", 
                                                            rospy.Time(0))
    move_group_to(group, temp_x, temp_y, temp_z-0.1)
    gripper.close()
    time.sleep(1)

    # Move up with AR tag and release gripper
    move_group_to(group, temp_x, temp_y, temp_z)

    if end_location:
        end_x, end_y, end_z = end_location
    else:
        end_x, end_y, end_z = x, y, z

    raw_input("wait")
    move_group_to(group, end_x, end_y + 0.02, end_z + 0.05)
    gripper.open()

def get_artag_location(listener, ar_marker):
    trans = None
    print 'Locating %s...' % ar_marker
    while not trans:
        try:
            trans, rot = listener.lookupTransform("base", str(ar_marker), rospy.Time(0))
        except:
            continue
    return trans

def init_move_group(group, position_tolerance=0.01, 
                    orientation_tolerance=0.1, velocity_factor=0.5):
    group.set_planner_id('RRTConnectkConfigDefault')
    group.set_start_state_to_current_state()
    group.set_goal_position_tolerance(position_tolerance)
    group.set_goal_orientation_tolerance(orientation_tolerance)
    group.set_max_velocity_scaling_factor(velocity_factor)

def move_group_to_deprecated(group, x, y, z, ox=0, oy=1, oz=0, ow=0):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = ox
    pose_target.orientation.y = oy
    pose_target.orientation.z = oz
    pose_target.orientation.w = ow
    pose_target.position.x = x 
    pose_target.position.y = y
    pose_target.position.z = z

    result = False
    while not result:
        group.set_pose_target(pose_target)
        result = group.go()

def move_group_to(group, x, y, z, ox=0, oy=1, oz=0, ow=0):
    waypoints = []

    # start with the current pose
    waypoints.append(group.get_current_pose().pose)

    # first orient gripper and move forward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.x = ox
    wpose.orientation.y = oy
    wpose.orientation.z = oz
    wpose.orientation.w = ow
    wpose.position.x = NEUTRAL_POS[0]
    wpose.position.y = NEUTRAL_POS[1]
    wpose.position.z = NEUTRAL_POS[2]
    waypoints.append(copy.deepcopy(wpose))

    # second move down
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = ox
    pose_target.orientation.y = oy
    pose_target.orientation.z = oz
    pose_target.orientation.w = ow
    pose_target.position.x = x 
    pose_target.position.y = y
    pose_target.position.z = z
    waypoints.append(copy.deepcopy(pose_target))

    ## We want the cartesian path to be interpolated at a resolution of 1 cm
    ## which is why we will specify 0.01 as the eef_step in cartesian
    ## translation.  We will specify the jump threshold as 0.0, effectively
    ## disabling it.
    (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0) # jump_threshold
    group.set_pose_target(pose_target)
    print plan3
    print waypoints
    rospy.sleep(10)
    group.execute(plan3)

def main():
    # Initiate node, listener, and gripper
    rospy.init_node("testing")
    listener = tf.TransformListener()
    right_gripper = robot_gripper.Gripper('right')

    while not rospy.is_shutdown():
        right_gripper.open()

        try:
            # Create and initialize the MoveGroup
            group = MoveGroupCommander("right_arm")
            init_move_group(group, position_tolerance=0.01)
            print group.get_end_effector_link()
            # for dbugging
            #################
            # end_location = get_artag_location(listener, "ar_marker_20")
            # upper_left = get_artag_location(listener, "ar_marker_23")
            # lower_right = get_artag_location(listener, "ar_marker_22")  
            # cg = RobotCheckers(upper_left, lower_right)
            # end_location = cg.location(1, 1)
            # print end_location
            #################
            # move = cg.detect_opponent_move(listener)
            # print move
            move_group_to(group, 0.609, -0.131, -0.157)

            # move_checkers_piece(group, right_gripper, listener, 0, end_location=end_location)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        sys.exit()


if __name__ == '__main__':
    main()
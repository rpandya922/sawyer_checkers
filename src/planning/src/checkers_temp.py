#!/usr/bin/env python
import rospy
import tf
import sys
from ik import get_artag_location, move_checkers_piece


class CheckersGame(object):
    """docstring for CheckersGame"""
    def __init__(self, board_size, upper_left_marker, lower_right_marker):
        super(CheckersGame, self).__init__()
        self.board_size = board_size
        self.upper_left = upper_left_marker
        self.lower_right = lower_right_marker

    def location(self, i, j):
        left, upper, z1 = self.upper_left
        right, lower, z2 = self.lower_right
        x = i * (left - right)/(self.board_size - 1) + right
        y = j * (upper - lower)/(self.board_size - 1) + lower
        z = j * (z1 - z2)/(self.board_size - 1)+ z2
        return x, y, z


def main():
    # Initiate node, listener, and gripper
    rospy.init_node("checkers")
    listener = tf.TransformListener()
    right_gripper = robot_gripper.Gripper('right')

    while not rospy.is_shutdown():
        right_gripper.open()

        try:
            # Create and initialize the MoveGroup
            group = MoveGroupCommander("right_arm")
            init_move_group(group)

            # Create checkers game
            upper_left = get_artag_location(listener, "ar_marker_9")
            lower_right = get_artag_location(listener, "ar_marker_20")            
            cg = CheckersGame(3, upper_left, lower_right)

            # Move to end location
            end_location = cg.location(1, 1)            
            move_checkers_piece(group, right_gripper, listener, 0, end_location=end_location)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        sys.exit()


if __name__ == '__main__':
    main()

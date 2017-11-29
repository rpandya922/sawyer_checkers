#!/usr/bin/env python
import checkers
import agent
import sys
sys.path.insert(0,'~/ros_workspaces/sawyer_checkers/src/planning/src')
# import ik
from ik import *
# import rospy
# import tf
# from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
# import geometry_msgs
# from moveit_commander import MoveGroupCommander
# from baxter_interface import gripper as robot_gripper
# import time

BLACK, WHITE = 0, 1

def main():
    # Initiate node, listener, and gripper
    rospy.init_node("testing")
    listener = tf.TransformListener()
    right_gripper = robot_gripper.Gripper('right')

    # Sets the AI agent (please type "arthur")
    # agent_module = raw_input("Enter name of agent module: ");
    agent_module = "arthur"
    __import__(agent_module)
    agent_module = sys.modules[agent_module]
    cpu = agent.CheckersAgent(agent_module.move_function)

    # while True:
    #     choice = raw_input("Enter 0 to go first and 1 to go second: ")
    #     try:
    #         choice = int(choice)
    #         break
    #     except ValueError:
    #         print "Please input 0 or 1."
    #         continue

    # Initialize game
    choice = 1
    turn = 0
    B = checkers.CheckerBoard()
    current_player = B.active
    print "Black moves first."

    while not rospy.is_shutdown():
        right_gripper.open()

        try:
            # Create and initialize the MoveGroup
            group = MoveGroupCommander("right_arm")
            init_move_group(group)
            
            upper_left = get_artag_location(listener, "ar_marker_7")
            lower_right = get_artag_location(listener, "ar_marker_5")  

            cg = RobotCheckers(upper_left, lower_right)
            
            # Game loop
            while not B.is_over():
                print B

                # Human's turn
                if turn % 2 == choice:
                    legal_moves = B.get_moves()
                    if B.jump:
                        print "Make jump."
                        print ""
                    else:
                        print "Turn %i" % (turn + 1)
                        print ""
                    for (i, move) in enumerate(get_move_strings(B)):
                        print "Move " + str(i) + ": " + move
                    while True:
                        move_idx = raw_input("Enter your move number: ")
                        try:
                            move_idx = int(move_idx)
                        except ValueError:
                            print "Please input a valid move number."
                            continue
                        if move_idx in range(len(legal_moves)):
                            break
                        else:
                            print "Please input a valid move number."
                            continue
                    B.make_move(legal_moves[move_idx])
                    # If jumps remain, then the board will not update current player
                    if B.active == current_player:
                        print "Jumps must be taken."
                        continue
                    else:
                        current_player = B.active
                        turn += 1
                else:
                    # Baxter's turn
                    start, end = B.make_move(cpu.make_move(B))
                    print start, end
                    cg.robot_make_move(group, right_gripper, listener, start, end)

                    if B.active == current_player:
                        print "Jumps must be taken."
                        continue
                    else:
                        current_player = B.active
                        turn += 1
        
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        # sys.exit()
        
    # Game end
    print B
    if B.active == WHITE:
        print "Congrats Black, you win!"
    else:
        print "Congrats Red, you win!"
    return 0


def game_over(board):
    return len(board.get_moves()) == 0

def get_move_strings(board):
    rfj = board.right_forward_jumps()
    lfj = board.left_forward_jumps()
    rbj = board.right_backward_jumps()
    lbj = board.left_backward_jumps()

    if (rfj | lfj | rbj | lbj) != 0:
        rfj = [(1 + i - i//9, 1 + (i + 8) - (i + 8)//9)
                    for (i, bit) in enumerate(bin(rfj)[::-1]) if bit == '1']
        lfj = [(1 + i - i//9, 1 + (i + 10) - (i + 8)//9)
                    for (i, bit) in enumerate(bin(lfj)[::-1]) if bit == '1']
        rbj = [(1 + i - i//9, 1 + (i - 8) - (i - 8)//9)
                    for (i, bit) in enumerate(bin(rbj)[::-1]) if bit ==  '1']
        lbj = [(1 + i - i//9, 1 + (i - 10) - (i - 10)//9)
                    for (i, bit) in enumerate(bin(lbj)[::-1]) if bit == '1']

        if board.active == BLACK:
            regular_moves = ["%i to %i" % (orig, dest) for (orig, dest) in rfj + lfj]
            reverse_moves = ["%i to %i" % (orig, dest) for (orig, dest) in rbj + lbj]
            return regular_moves + reverse_moves
        else:
            reverse_moves = ["%i to %i" % (orig, dest) for (orig, dest) in rfj + lfj]
            regular_moves = ["%i to %i" % (orig, dest) for (orig, dest) in rbj + lbj]
            return reverse_moves + regular_moves


    rf = board.right_forward()
    lf = board.left_forward()
    rb = board.right_backward()
    lb = board.left_backward()

    rf = [(1 + i - i//9, 1 + (i + 4) - (i + 4)//9)
                for (i, bit) in enumerate(bin(rf)[::-1]) if bit == '1']
    lf = [(1 + i - i//9, 1 + (i + 5) - (i + 5)//9)
                for (i, bit) in enumerate(bin(lf)[::-1]) if bit == '1']
    rb = [(1 + i - i//9, 1 + (i - 4) - (i - 4)//9)
                for (i, bit) in enumerate(bin(rb)[::-1]) if bit ==  '1']
    lb = [(1 + i - i//9, 1 + (i - 5) - (i - 5)//9)
                for (i, bit) in enumerate(bin(lb)[::-1]) if bit == '1']

    if board.active == BLACK:
        regular_moves = ["%i to %i" % (orig, dest) for (orig, dest) in rf + lf]
        reverse_moves = ["%i to %i" % (orig, dest) for (orig, dest) in rb + lb]
        return regular_moves + reverse_moves
    else:
        regular_moves = ["%i to %i" % (orig, dest) for (orig, dest) in rb + lb]
        reverse_moves = ["%i to %i" % (orig, dest) for (orig, dest) in rf + lf]
        return reverse_moves + regular_moves

if __name__ == '__main__':
    try:
        status = main()
        sys.exit(status)
    except KeyboardInterrupt:
        print ""
        print "Game terminated."
        sys.exit(1)

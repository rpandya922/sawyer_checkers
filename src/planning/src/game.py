#!/usr/bin/env python
import checkers
import agent
import sys
sys.path.insert(0,'~/ros_workspaces/sawyer_checkers/src/planning/src')
from ik import *
import arthur

BLACK, WHITE = 0, 1

def main():
    # Initiate node, listener, and gripper
    rospy.init_node("game")
    listener = tf.TransformListener()
    right_gripper = robot_gripper.Gripper('right')

    # Publishers for human and robot moves
    human_pub = rospy.Publisher('human_moves', CheckersMove, queue_size=10)
    robot_pub = rospy.Publisher('robot_moves', CheckersMove, queue_size=10)
    calibration_pub = rospy.Publisher('board_calibration', BoardCalibration, queue_size=10)
    cpu = agent.CheckersAgent(arthur.move_function)

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
            obs_group = MoveGroupCommander("left_arm")
            init_move_group(obs_group)
            
            move_group_to(obs_group, UL_POS[0], UL_POS[1], UL_POS[2], 
            UL_ORIENTATION[0], UL_ORIENTATION[1], UL_ORIENTATION[2], UL_ORIENTATION[3])
            raw_input("wait")
            upper_left = get_artag_location(listener, "ar_marker_23")

            move_group_to(obs_group, LR_POS[0], LR_POS[1], LR_POS[2], 
            LR_ORIENTATION[0], LR_ORIENTATION[1], LR_ORIENTATION[2], LR_ORIENTATION[3])
            raw_input("wait")
            lower_right = get_artag_location(listener, "ar_marker_22")

            raw_input("wait")
            move_group_to(obs_group, OBS_POS[0], OBS_POS[1], OBS_POS[2], 
            OBS_ORIENTATION[0], OBS_ORIENTATION[1], OBS_ORIENTATION[2], OBS_ORIENTATION[3])

            cg = RobotCheckers(upper_left, lower_right)

            calibration_pub.publish(upper_left[0], upper_left[1], upper_left[2], \
                                    lower_right[0], lower_right[1], lower_right[2])
            
            # Game loop
            while not B.is_over():
                print B

                # Human's turn
                if turn % 2 == choice:
                    moves = zip(B.get_moves(), get_move_strings(B))
                    human_move = cg.detect_opponent_move(listener, obs_group)
                    human_move = [move for (move, move_tuple) in moves if move_tuple == human_move]
                    if human_move != -1:
                        human_pub.publish(human_move[0], human_move[1])
                    # print human_move
                    if len(human_move) <= 0:
                        legal_moves = B.get_moves()
                        if B.jump:
                            print "Make jump."
                            print ""
                        else:
                            print "Turn %i" % (turn + 1)
                            print ""
                        for (i, move) in enumerate(get_move_strings(B)):
                            print "Move " + str(i) + ": " + str(move)
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
                    else:
                        B.make_move(human_move[0])

                    # If jumps remain, then the board will not update current player
                    if B.active == current_player:
                        print "Jumps must be taken."
                        continue
                    else:
                        current_player = B.active
                        turn += 1
                else:
                    # Baxter's turn
                    move, move_tuple = cpu.make_move(B)
                    taken_pieces = B.make_move(move)
                    start, end = move_tuple
                    print start, end
                    robot_pub.publish(start, end)
                    cg.set_taken_pieces(B.state)
                    # cg.robot_make_move(group, right_gripper, listener, start, end, taken_pieces)

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
            return rfj + lfj + rbj + lbj
        else:
            return rfj + lfj + rbj + lbj


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
        return rf + lf + rb + lb
    else:
        return rf + lf + rb + lb

if __name__ == '__main__':
    try:
        status = main()
        sys.exit(status)
    except KeyboardInterrupt:
        print ""
        print "Game terminated."
        sys.exit(1)

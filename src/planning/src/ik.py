#!/usr/bin/env python
import rospy
import tf
import sys
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
import geometry_msgs
from moveit_commander import MoveGroupCommander
from baxter_interface import gripper as robot_gripper

def move_checkers_piece(group, gripper, listener, checkers_number, end_location=None):
    # Get location of AR tag
    x, y, z = get_artag_location("ar_marker_%d" % checkers_number)

    # Move to place above AR tag
    move_group_to(group, x, y+0.02, z+0.1)
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
    move_group_to(group, end_x, end_y, end_z + 0.1)
    gripper.open()

def get_artag_location(ar_marker):
    trans = None
    while not trans:
        try:
            trans, rot = listener.lookupTransform("base", ar_marker, rospy.Time(0))
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

def move_group_to(group, x, y, z, ox=0, oy=1, oz=0, ow=0):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = ox
    pose_target.orientation.y = oy
    pose_target.orientation.z = oz
    pose_target.orientation.w = ow
    pose_target.position.x = x 
    pose_target.position.y = y
    pose_target.position.z = z

    group.set_pose_target(pose_target)
    group.go(wait=True)

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
            init_move_group(group)
            move_checkers_piece(group, right_gripper, listener, 0)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        sys.exit()


if __name__ == '__main__':
    main()
#!/usr/bin/env python
import rospy
import tf
import sys
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
import geometry_msgs
from moveit_commander import MoveGroupCommander
from baxter_interface import gripper as robot_gripper
import time

if __name__ == '__main__':
    rospy.init_node("testing")
    listener = tf.TransformListener()
    right_gripper = robot_gripper.Gripper('right')

    while not rospy.is_shutdown():
        right_gripper.open()

        # x, y, z = [0.854, -0.214, -0.128]
        trans = None
        while not trans:
            try:
                trans, rot = listener.lookupTransform("base", "ar_marker_0", rospy.Time(0))
            except:
                continue
        # trans.extend(rot)
        x, y, z = trans

        try:
            group = MoveGroupCommander("right_arm")
            group.set_planner_id('RRTConnectkConfigDefault')
            group.set_start_state_to_current_state()
            group.set_goal_position_tolerance(0.01)
            group.set_goal_orientation_tolerance(0.1)
            # group.set_max_velocity_scaling_factor(0.5)

            # Setting position and orientation target
            # group.set_pose_target(request.ik_request.pose_stamped)


            # pose_target = geometry_msgs.msg.Pose()
            # pose_target.orientation.w = 1.0
            # pose_target.position.x = 0.7
            # pose_target.position.y = -0.05
            # pose_target.position.z = 1.1
            # group.set_pose_target(pose_target)

            pose_target = geometry_msgs.msg.Pose()
            pose_target.orientation.x = 0
            pose_target.orientation.y = 1
            pose_target.orientation.z = 0
            pose_target.orientation.w = 0
            pose_target.position.x = x 
            pose_target.position.y = y + 0.02
            pose_target.position.z = z + 0.1
            group.set_pose_target(pose_target)
            # set_position_target works fine, pose is tricky
            # group.set_position_target(trans)
            # the following hard-coded values work fine
            # group.set_pose_target([0.784, -0.391, -0.088, 0.978, 0.206, 0.004, 0.025])

            group.go(wait=True)
            raw_input("wait")

            (x, y, z), _ = listener.lookupTransform("base", "right_gripper", rospy.Time(0))
            group.set_pose_target([x, y, z - 0.1, 0, 1, 0, 0])
            # group.set_pose_target([x, y+ 0.01, z, 0, 1, 0, 0])
            group.go(wait=True)

            right_gripper.close()
            time.sleep(1)

            (x, y, z), _ = listener.lookupTransform("base", "right_gripper", rospy.Time(0))
            group.set_pose_target([x, y, z + 0.1, 0, 1, 0, 0])
            # group.set_pose_target([x, y+ 0.01, z, 0, 1, 0, 0])
            group.go(wait=True)

            right_gripper.open()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        sys.exit()
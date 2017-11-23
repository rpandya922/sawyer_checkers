#!/usr/bin/env python
import rospy
import tf
import sys
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
import geometry_msgs
from moveit_commander import MoveGroupCommander
from baxter_interface import gripper as robot_gripper

if __name__ == '__main__':
    rospy.init_node("testing")

    while not rospy.is_shutdown():
        x, y, z = [0.854, -0.214, -0.128]

        try:
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            # group.set_pose_target(request.ik_request.pose_stamped)


            # pose_target = geometry_msgs.msg.Pose()
            # pose_target.orientation.w = 1.0
            # pose_target.position.x = 0.7
            # pose_target.position.y = -0.05
            # pose_target.position.z = 1.1
            # group.set_pose_target(pose_target)

            pose_target = geometry_msgs.msg.Pose()
            pose_target.orientation.x = 0.978
            pose_target.orientation.y = 0.206
            pose_target.orientation.z = 0.004
            pose_target.orientation.w = 0.025
            pose_target.position.x = 0.784
            pose_target.position.y = -0.391
            pose_target.position.z = -0.088
            group.set_pose_target(pose_target)

            group.go(wait=True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        sys.exit()
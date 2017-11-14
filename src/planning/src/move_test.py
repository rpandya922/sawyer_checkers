#!/usr/bin/env python
import rospy
import tf
import sys
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander

if __name__ == '__main__':
    rospy.wait_for_service('compute_ik')
    print("finished waiting for compute_ik")
    rospy.init_node("testing")
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    listener = tf.TransformListener()
    # rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        trans = None
        while not trans:
            try:
                trans, rot = listener.lookupTransform("base", "ar_marker_9", rospy.Time(0))
            except:
                continue
        print trans
        x, y, z = trans

        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_wrist"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        # x, y, z = [0.825, -0.241, 0.071]
        # [0.001, 0.999, -0.028, 0.017]
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = -y
        request.ik_request.pose_stamped.pose.position.z = z
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0

        try:
            #Send the request to the service
            response = compute_ik(request)
            
            #Print the response HERE
            print response
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK and execute
            group.go()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        sys.exit()
        # rate.sleep()

import sys
import rospy
from sensor_msgs.msg import Image

def main():
	rospy.init_node('camera_display', anonymous=True)
    camera = intera_interface.Cameras()
    if not camera.verify_camera_exists("head_camera"):
        rospy.logerr("Invalid camera name, exiting the example.")
        return
    camera.start_streaming("head_camera")
    # rectify_image = not args.raw
    # use_canny_edge = args.edge
    camera.set_callback("head_camera", show_image_callback,
        rectify_image=rectify_image, callback_args=(use_canny_edge, args.camera))
 
    def clean_shutdown():
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()
 
    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Camera_display node running. Ctrl-c to quit")
    rospy.spin()
#!/usr/bin/env python
import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


if __name__ == "__main__":
    rospy.init_node("darknet_test_node")

    pub = rospy.Publisher(
        "/camera/image_raw",
        Image,
        queue_size=1
    )

    rospy.sleep(2)

    bridge = CvBridge();
    f = "/home/bwilab/horse.png"
    img = cv2.imread(f, 1)
    pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

    rospy.loginfo("Image sent!")

    rospy.spin()

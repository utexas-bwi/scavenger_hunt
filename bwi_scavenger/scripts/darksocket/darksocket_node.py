#!/usr/bin/env python
import cv_bridge
import cv2
import darksocket
import os
import rospy
from sensor_msgs.msg import Image


bridge = cv_bridge.CvBridge()
server = darksocket.Server()
img_path = os.getcwd() + "/__darksocket_node_out.jpeg"
t_begin = 0
t_end = 0


def process(msg):
    t_begin = rospy.get_time()
    cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imwrite(img_path, cv_img)

    with open(img_path, "rb") as img:
        img_bytes = img.read()
        server.send(darksocket.pack(darksocket.Packet.IMAGE, img_bytes))

    rospy.loginfo("Server is listening...")

    res = server.recv()
    t_end = rospy.get_time()

    rospy.loginfo("FPS: %s" % (1 / (t_end - t_begin)))

    # if res == darksocket.StreamEvent.DETECTIONS:
    #     dets = server.stream.detections
    #     for det in dets:
    #         rospy.loginfo(det)


if __name__ == "__main__":
    rospy.init_node("darksocket_node")
    server.launch()

    rospy.Subscriber(
        "/camera/rgb/image_color", Image, process
    )

    rospy.spin()

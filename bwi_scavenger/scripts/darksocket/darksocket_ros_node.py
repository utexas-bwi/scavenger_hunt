#!/usr/bin/env python
import cv_bridge
import cv2
import darksocket
import os
import rospy
from sensor_msgs.msg import Image


bridge = cv_bridge.CvBridge()
server = darksocket.Server()
img_path = os.getcwd() + "/__darksocket_ros_node_out__.jpeg"
t_begin = 0
t_end = 0
t_last_img = -1


def recv_img(msg):
    global t_last_img
    t = rospy.get_time()

    if t_last_img == -1 or t - t_last_img > 0.1:
        process(msg)
        t_last_img = t


def process(msg):
    t_begin = rospy.get_time()
    cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imwrite(img_path, cv_img)

    with open(img_path, "rb") as img:
        img_bytes = img.read()
        server.send(darksocket.pack(darksocket.Packet.IMAGE, img_bytes))

    rospy.loginfo("Image sent. Waiting for response...")

    res = server.recv()
    t_end = rospy.get_time()

    if res == darksocket.StreamEvent.DETECTIONS:
        dets = server.stream.detections
        for det in dets:
            rospy.loginfo(det)

    rospy.loginfo("FPS: %s" % (1 / (t_end - t_begin)))


if __name__ == "__main__":
    rospy.init_node("darksocket_ros_node")
    server.launch()
    rospy.sleep(2)
    rospy.Subscriber(
        "/camera/rgb/image_color", Image, recv_img
    )
    rospy.spin()

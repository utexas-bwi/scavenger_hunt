#!/usr/bin/env python
from darknet_ros_msgs.msg import BoundingBox
from darksocket_ros.msg import Detections
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import darksocket
import os
import rospy


bridge = cv_bridge.CvBridge()
server = None
img_path = os.getcwd() + "/__darksocket_ros_node_out__.jpeg"
pub_detections = None
t_begin = 0
t_end = 0
cls = lambda : os.system("cls" if os.name == "nt" else "clear")
telemetry = True


def process(msg):
    t_begin = rospy.get_time()
    cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imwrite(img_path, cv_img)

    with open(img_path, "rb") as img:
        img_bytes = img.read()
        server.send(darksocket.pack(darksocket.Packet.IMAGE, img_bytes))

    res = server.recv()
    t_end = rospy.get_time()

    if telemetry:
        cls()
        rospy.loginfo("%s FPS" % (1 / (t_end - t_begin)))

    if res == darksocket.StreamEvent.DETECTIONS:
        dets = Detections()
        dets.image = msg

        for det in server.stream.detections:
            bbox = BoundingBox()
            bbox.Class = det[0]
            bbox.probability = det[1]
            bbox.xmin = det[2]
            bbox.ymin = det[3]
            bbox.xmax = det[2] + det[4]
            bbox.ymax = det[3] + det[5]

            dets.bboxes.append(bbox)

            if telemetry:
                rospy.loginfo("%s %s" % (bbox.Class, bbox.probability))

        pub_detections.publish(dets)


if __name__ == "__main__":
    rospy.init_node("darksocket_ros_node")

    telemetry = rospy.get_param("telemetry", True)
    conf_path = rospy.get_param("conf_path", os.getcwd() + "/darksocket.conf")
    print(conf_path)
    tpc_camera = rospy.get_param(
        "darksocket_ros/topics/camera", "/camera/rgb/image_color"
    )
    tpc_detections = rospy.get_param(
        "darksocket_ros/topics/detections", "/darksocket_ros/detections"
    )
    pub_detections = rospy.Publisher(
        tpc_detections, Detections, queue_size=0
    )
    server_ip = rospy.get_param("server/host", "127.0.0.1")
    server_port = rospy.get_param("server/port", 51820)
    server = darksocket.Server(host=server_ip, port=server_port)

    darksocket.load_conf(conf_path)
    server.launch()
    rospy.sleep(2)
    rospy.loginfo("Waiting for client...")
    rospy.Subscriber(tpc_camera, Image, process)
    rospy.spin()

    if os.path.exists(img_path):
        os.remove(img_path)

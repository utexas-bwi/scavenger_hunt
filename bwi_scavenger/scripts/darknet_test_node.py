#!/usr/bin/env python
import cv2
import os
import rospy

from bwi_scavenger_msgs.msg import DarknetAddTrainingFile
from bwi_scavenger_msgs.msg import DarknetStartTraining
from cv_bridge import CvBridge, CvBridgeError
from globals import *
from sensor_msgs.msg import Image


if __name__ == "__main__":
    rospy.init_node("darknet_test_node")

    pub_add_training_file = rospy.Publisher(
        TPC_DARKNET_NODE_ADD_TRAINING_FILE,
        DarknetAddTrainingFile,
        queue_size=1
    )

    pub_start_training = rospy.Publisher(
        TPC_DARKNET_NODE_START_TRAINING,
        DarknetStartTraining,
        queue_size=1
    )

    rospy.sleep(2)

    # a = DarknetStartTraining()
    # a.network_name = "Find Object"
    # start_training_cb(a)

    a = DarknetAddTrainingFile()
    b = Image()
    b.encoding = "bgr8"
    a.image = b
    a.network_name = "Find Object"
    a.label = "can"
    a.xmin = 0
    a.xmax = 10
    a.ymin = 0
    a.ymax = 10
    a.image_width = 100
    a.image_height = 100
    pub_add_training_file.publish(a)

    rospy.loginfo("spinning...")

    rospy.spin()

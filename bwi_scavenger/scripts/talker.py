#!/usr/bin/env python
import rospy
from std_msgs.msg import String


if __name__ == "__main__":
    rospy.init_node("talker")

    pub = rospy.Publisher("/hello_from_kane", String, queue_size=1)

    rospy.sleep(2)

    rospy.loginfo("talking...")

    while not rospy.is_shutdown():
        pub.publish("aaaaa")

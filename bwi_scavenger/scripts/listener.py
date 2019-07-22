#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def cb(msg):
    rospy.loginfo(msg.data)


if __name__ == "__main__":
    rospy.init_node("listener")

    rospy.Subscriber("/hello_from_kane", String, cb)

    rospy.sleep(2)

    rospy.spin()

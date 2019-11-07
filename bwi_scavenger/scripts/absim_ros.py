#!/usr/bin/env python

import bwi_scavenger_msgs.srv
import rospy
import socket


ip = "127.0.0.1" # TODO change to robot IP
port = 5005

# string[] objects_found
#---
# string next_location
def update(req):

    res = bwi_scavenger_msgs.srv.GetNextLocationResponse()
   
    """Send objects found over socket to obtain the next location calculated by the absim
    """
    sock.sendto(",".join(req.objects_found), (ip, port))
    next_location, addr = sock.recvfrom(1000) # socket will send back the next_location

    res.next_location = next_location.decode('utf-8')

    return res


if __name__ == "__main__":
    rospy.init_node("absim_node")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    proxy = rospy.Service(
        "/absim",
        bwi_scavenger_msgs.srv.GetNextLocation,
        update
    )

    rospy.sleep(3)
    rospy.spin()
